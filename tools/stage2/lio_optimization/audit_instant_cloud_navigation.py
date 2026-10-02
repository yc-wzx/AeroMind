"""Preserved strict mission audit plus actual corrected component and point set."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(Path(__file__).parent))
from audit_all_point_frames import run as audit_clouds


def evaluate(directory,point_report):
    path=ROOT/'tools/stage2/lio_safe_audit/audit_safe_navigation.py'
    spec=importlib.util.spec_from_file_location('sealed_safe_evaluator',path)
    original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
    result=original.evaluate(directory)
    loaded=json.loads((directory/'actual_instant_cloud_LIO_loaded.json').read_text())
    manifest=json.loads((directory/'input_manifest.json').read_text())['inputs']
    key='install/stage2_lio_sim/lib/lio_point_boundary/libspark_lio_component.so'
    expected=manifest[key]
    result['checks']['actual_instantaneous_cloud_component_loaded']=(len(loaded)==1 and
        loaded[0]['libraries']==[expected['resolved_path']] and list(loaded[0]['sha256'].values())==[expected['sha256']])
    instant='tools/results/stage2_step4_lio_optimization_20261002/instant_cloud_source'
    snapshot=directory/'input_snapshot'
    details=json.loads((snapshot/instant/'production_manifest.json').read_text())
    old=snapshot/'src/third_party/spark-fast-lio/spark_fast_lio'
    candidate=snapshot/instant
    changed=[]
    for file in old.rglob('*'):
        other=candidate/file.relative_to(old)
        if file.is_file() and other.is_file() and file.read_bytes()!=other.read_bytes():changed.append(str(file.relative_to(old)))
    header=(candidate/'include/imu_processing.hpp').read_text()
    result['checks']['only_declared_instantaneous_measurement_fix']=(
        details['full_3D'] and details['IMU_prediction_preserved'] and not details['observation_variance_changed'] and
        not details['sensor_timestamps_changed'] and not details['GT_input_added'] and
        changed==['include/imu_processing.hpp'] and 'point.curvature == 0.0f' in header and
        'if (it_pcl == pcl_out.points.begin()) return;' in header)
    point=audit_clouds(directory,None,point_report)
    result['point_set_evidence']=point
    result['checks']['every_recorded_corrected_cloud_rigid_consistent']=(
        point['frames']>500 and point['all_recorded_outputs_checked'] and
        point['all_points_rigid_consistent'] and point['unpaired_recorded_output_stamps']==[])
    result['all_pass']=all(result['checks'].values())
    result['instant_cloud_evaluator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['point_set_evaluator_sha256']=hashlib.sha256(Path(__file__).with_name('audit_all_point_frames.py').read_bytes()).hexdigest()
    result['instant_cloud_evaluator_command']=sys.argv
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--point-output',type=Path,required=True)
    a=p.parse_args()
    try:r=evaluate(a.result_dir,a.point_output)
    except Exception as e:r={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':repr(e)}
    serialized=json.dumps(r,indent=2,allow_nan=False)
    with a.output.open('x') as f:f.write(serialized)
    print(json.dumps({k:r.get(k) for k in ['all_pass','checks','error']},indent=2))
    raise SystemExit(0 if r['all_pass'] else 2)
