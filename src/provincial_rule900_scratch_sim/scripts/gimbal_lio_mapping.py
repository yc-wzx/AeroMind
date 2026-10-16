#!/usr/bin/env python3
"""Explicit simulation-only rebuilt LIO; no real sensor entry in this package."""
import argparse,hashlib,json,os,sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
def main():
    p=argparse.ArgumentParser();p.add_argument('--simulation-only',choices=['true'],required=True);_,rosargs=p.parse_known_args()
    root=next((v for v in Path(__file__).resolve().parents if (v/'tools/deployment/rule900_deployment.py').is_file()),None)
    if root is None:raise RuntimeError('Source checkout required; rebuild after moving it')
    sys.path.insert(0,str(root/'tools/deployment'))
    from rule900_deployment import verify_manifest
    verify_manifest()
    prefix=Path(get_package_prefix('provincial_rule900_scratch_sim'));runtime=prefix/'lib/provincial_rule900_scratch_sim/lio';profile=root/'tools/stage2/lio_scratch_reuse_profile'
    env=dict(os.environ,AEROMIND_LIO_CONSISTENCY_FIX='1',AEROMIND_DECLARED_UNBIASED_IMU='1',AEROMIND_LIO_KEYFRAME_MAP='1',AEROMIND_PLANE_QUALITY_GUARD='0',LD_LIBRARY_PATH=str(runtime)+':'+os.environ.get('LD_LIBRARY_PATH',''))
    for key in ['AEROMIND_STATIC_SIM_STARTUP','AEROMIND_EXACT_LOCAL_GAUGE','LD_PRELOAD']:env.pop(key,None)
    command=['unshare','--user','--map-root-user','--mount','--propagation','private','bash',str(profile/'readonly_source.sh'),str(runtime/'spark_lio_mapping'),*rosargs]
    os.execvpe(command[0],command,env)
if __name__=='__main__':main()
