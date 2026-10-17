"""Isolated output-only SPARK variant; original source/install/manifest stay sealed."""
import hashlib,json,shutil,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sys.path.insert(0,str(HERE.parent/'mid360'))
from preflight import MANIFEST as RAW_MANIFEST,verify_build
from core import sha
BASE=ROOT/'src/third_party/spark-fast-lio/spark_fast_lio'
SOURCE=ROOT/'build_mid360_lio_phased/src/spark_fast_lio'
INSTALL=ROOT/'install_mid360_lio_phased'
MANIFEST=INSTALL/'phase_input_manifest.json'
CONTRACT='scan_corrected_primary_v1'
PINS={'src/spark_fast_lio.cpp':'53510556a495e0d7e57939f62331b0e010622387563ad328a568bacfbb11262e',
      'include/spark_fast_lio.h':'e3a6d63ee6de55ab6a3ca7b5c6829a20e262a228069fc2d705b63faaf14f7c34'}

def replace_once(text,old,new):
    if text.count(old)!=1:raise RuntimeError('Source anchor mismatch: '+old[:100])
    return text.replace(old,new,1)

def patched(relative,text):
    if relative=='include/spark_fast_lio.h':
        text=replace_once(text,'void publishOdometry(const state_ikfom &state, const rclcpp::Time &stamp);',
            'void publishOdometry(const state_ikfom &state, const rclcpp::Time &stamp, bool imu_prediction = false);')
        text=replace_once(text,'  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;',
            '  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;\n'
            '  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_prediction_;\n'
            '  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_mixed_;')
    elif relative=='src/spark_fast_lio.cpp':
        text=replace_once(text,'  pub_odom_                 = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);',
            '  pub_odom_                 = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);\n'
            '  pub_odom_prediction_ = create_publisher<nav_msgs::msg::Odometry>("odometry_predicted", qos);\n'
            '  pub_odom_mixed_ = create_publisher<nav_msgs::msg::Odometry>("odometry_mixed_legacy", qos);\n'
            '  auto contract = declare_parameter<std::string>("common.odometry_output_contract",\n'
            '      "scan_corrected_primary_v1", rcl_interfaces::msg::ParameterDescriptor().set__read_only(true));\n'
            '  if (contract != "scan_corrected_primary_v1") throw std::invalid_argument("Output contract override rejected");')
        text=replace_once(text,'  publishOdometry(integrated_state, stamp);','  publishOdometry(integrated_state, stamp, true);')
        begin=text.index('void SPARKFastLIO2::publishOdometry(')
        end=text.index('\nvoid SPARKFastLIO2::publishPath(',begin)
        chunk=text[begin:end]
        chunk=replace_once(chunk,'const rclcpp::Time &stamp) {',
            'const rclcpp::Time &stamp, bool imu_prediction) {\n'
            '  // Local output message: prediction/correction calls cannot overwrite each other.\n'
            '  nav_msgs::msg::Odometry output;')
        chunk=chunk.replace('odomAftMapped_','output')
        chunk=replace_once(chunk,'  pub_odom_->publish(output);',
            '  // Preserve EVERY original output and original timestamp for diagnosis.\n'
            '  pub_odom_mixed_->publish(output);\n'
            '  if (imu_prediction) {\n'
            '    pub_odom_prediction_->publish(output);\n'
            '    return;  // A predicted state never broadcasts the corrected-frame TF.\n'
            '  }\n'
            '  pub_odom_->publish(output);')
        text=text[:begin]+chunk+text[end:]
    return text

def prepare():
    verify_build()
    for relative,expected in PINS.items():
        if sha(BASE/relative)!=expected:raise RuntimeError('Unsupported original producer: '+relative)
    expected={str(p.relative_to(BASE)):p.read_bytes() for p in BASE.rglob('*') if p.is_file()}
    for relative in PINS:expected[relative]=patched(relative,expected[relative].decode()).encode()
    if SOURCE.exists():
        actual={str(p.relative_to(SOURCE)):p.read_bytes() for p in SOURCE.rglob('*') if p.is_file()}
        if actual!=expected:raise RuntimeError('Existing variant source differs; preserving it without deleting/overwriting')
    else:
        SOURCE.mkdir(parents=True,exist_ok=False)
        for relative,data in expected.items():
            target=SOURCE/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    return SOURCE

def inputs():
    paths=[p for directory in [HERE,SOURCE,INSTALL] for p in directory.rglob('*')
           if p.is_file() and p!=MANIFEST and '__pycache__' not in p.parts]
    paths.append(HERE.parent/'mid360_lio/lio_mid360_static.yaml')
    paths.extend(HERE.parent/'mid360_lio'/name for name in ['start.py','inspect_raw.py','static_check.py'])
    return {str(p.relative_to(ROOT)):dict(sha256=sha(p),resolved=str(p.resolve())) for p in sorted(set(paths))}

def seal():
    verify_build()
    record=dict(contract=CONTRACT,root=str(ROOT),original_raw_manifest_sha256=sha(RAW_MANIFEST),
                inputs=inputs(),scope='Output-only sensor-local variant; no estimation changes or navigation integration')
    if MANIFEST.exists():
        if json.loads(MANIFEST.read_text())!=record:raise RuntimeError('Existing variant manifest differs; preserve and investigate')
    else:MANIFEST.write_text(json.dumps(record,indent=2)+'\n')
    return record

def verify():
    verify_build();record=json.loads(MANIFEST.read_text())
    if record['root']!=str(ROOT) or record['contract']!=CONTRACT or record['original_raw_manifest_sha256']!=sha(RAW_MANIFEST) or record['inputs']!=inputs():
        raise RuntimeError('Variant source/install/tool inputs changed; original raw baseline is preserved')
    return record

if __name__=='__main__':
    mode=sys.argv[1]
    result=prepare() if mode=='prepare' else seal() if mode=='seal' else verify()
    print(json.dumps(dict(mode=mode,source=str(SOURCE),manifest=str(MANIFEST),inputs=len(result['inputs']) if isinstance(result,dict) else None)))
