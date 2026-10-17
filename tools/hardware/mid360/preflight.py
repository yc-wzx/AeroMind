#!/usr/bin/env python3
"""Local build manifest and read-only MID-360 interface checks. Never edits networking."""
import argparse
import json
import re
import subprocess
from pathlib import Path
from core import sha

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
MANIFEST=ROOT/'install_mid360/mid360_input_manifest.json'

def inputs():
    source=[]
    for directory in [HERE,ROOT/'src/third_party/livox_ros_driver2',ROOT/'src/third_party/spark-fast-lio/spark_fast_lio']:
        source.extend(p for p in directory.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    products=[p for p in (ROOT/'install_mid360').rglob('*') if p.is_file() and p!=MANIFEST and '__pycache__' not in p.parts]
    products += [Path('/usr/local/lib/liblivox_lidar_sdk_shared.so'),Path('/usr/local/include/livox_lidar_api.h'),Path('/usr/local/include/livox_lidar_def.h')]
    return {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):
            dict(sha256=sha(p),resolved=str(p.resolve())) for p in sorted(set(source+products))}

def check_libraries():
    for p in [ROOT/'install_mid360/livox_ros_driver2/lib/liblivox_ros_driver2.so',ROOT/'install_mid360/spark_fast_lio/lib/libspark_lio_component.so']:
        result=subprocess.run(['ldd','-r',str(p)],text=True,capture_output=True)
        if result.returncode or any(v in result.stdout+result.stderr for v in ['not found','undefined symbol:','=> /opt/MVS/']):
            raise RuntimeError('Library loading check failed: '+str(p)+'\n'+result.stdout+result.stderr)

def verify_build():
    saved=json.loads(MANIFEST.read_text())
    if saved['build_root']!=str(ROOT) or saved['inputs']!=inputs():
        raise RuntimeError('Hardware source/install/SDK differs from local build; run hardware build.sh')
    return saved

def live_interface():
    addresses=json.loads(subprocess.check_output(['ip','-j','-4','addr','show'],text=True))
    matched=[item['ifname'] for item in addresses for v in item.get('addr_info',[]) if v.get('local')=='192.168.1.5']
    if not matched:
        raise RuntimeError('Host 192.168.1.5 is not configured. Confirm current wired address; this tool will not change it.')
    sockets=subprocess.check_output(['ss','-H','-lun'],text=True)
    occupied=[]
    for line in sockets.splitlines():
        fields=line.split()
        if len(fields)>3:
            port=fields[3].rsplit(':',1)[-1]
            if port in ['56101','56201','56301','56401']:occupied.append(int(port))
    if occupied:
        raise RuntimeError('MID360 receive ports already occupied '+str(occupied)+'; close Livox Viewer/previous driver yourself')
    return dict(host_ip='192.168.1.5',lidar_ip='192.168.1.173',interfaces=matched,network_changed=False)

def verify_capture(path):
    verify_build()
    path=Path(path); data=json.loads(path.read_text())
    if data.get('scope')!='REAL MID360 RAW INPUT PRECHECK' or data.get('pass_raw') is not True:
        raise RuntimeError('LIO requires a passing real raw capture first')
    if data['build_manifest_sha256']!=sha(MANIFEST):
        raise RuntimeError('Capture was made with a different local input build')
    for relative,expected in data['data_sha256'].items():
        if sha(path.parent/relative)!=expected:raise RuntimeError('Raw capture hash changed: '+relative)
    return data

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--record-build',action='store_true');parser.add_argument('--live',action='store_true');args=parser.parse_args()
    if args.record_build:
        check_libraries()
        record=dict(build_root=str(ROOT),git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                    inputs=inputs(),system_image=False,scope='Independent real sensor/LIO-only candidate; no navigation or motor commands')
        MANIFEST.write_text(json.dumps(record,indent=2)+'\n')
    record=verify_build();check_libraries()
    result=dict(build_inputs=len(record['inputs']),build_check=True,real_hardware_tested=False)
    if args.live:result['interface_preflight']=live_interface()
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
