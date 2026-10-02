"""Preserved strict mission audit plus exclusive LIO voxel parameter change."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import yaml
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/stage2/lio_optimization'))
from audit_instant_cloud_navigation_v2 import evaluate as original


def evaluate(directory,point_output):
    r=original(directory,point_output)
    current=next(iter(yaml.safe_load((directory/'runtime_parameters/lio_mapping.yaml').read_text()).values()))['ros__parameters']
    previous=next(iter(yaml.safe_load((directory/'input_snapshot/tools/results/stage2_step4_lio_optimization_20261002/full_instant_cloud_01/runtime_parameters/lio_mapping.yaml').read_text()).values()))['ros__parameters']
    changes={k:{'previous':previous.get(k),'current':current.get(k)} for k in set(previous)|set(current) if previous.get(k)!=current.get(k)}
    r['LIO_parameter_changes']=changes
    r['checks']['only_LIO_voxel_parameter_changed']=changes=={'filter_size_map':{'previous':.1,'current':.05}}
    snapshot=directory/'input_snapshot/src/uav_bringup/launch'
    before=(snapshot/'provincial_stage2_lio_point_boundary.launch.py').read_text()
    after=(snapshot/'provincial_stage2_lio_fine_map.launch.py').read_text()
    old="parameters=[os.path.join(bringup,'config','spark_provincial_noise_model_sim.yaml'),sim], output='log',"
    new="parameters=[os.path.join(bringup,'config','spark_provincial_noise_model_sim.yaml'),sim,{'filter_size_map':0.05}], output='log',"
    r['checks']['launch_change_only_LIO_voxel']=before.count(old)==1 and before.replace(old,new)==after
    r['all_pass']=all(r['checks'].values())
    r['fine_map_evaluator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    r['fine_map_evaluator_command']=sys.argv
    return r


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--point-output',type=Path,required=True);a=p.parse_args()
    try:r=evaluate(a.result_dir,a.point_output)
    except Exception as e:r={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':repr(e)}
    with a.output.open('x') as f:json.dump(r,f,indent=2,allow_nan=False)
    print(json.dumps({k:r.get(k) for k in ['all_pass','checks','LIO_parameter_changes','error']},indent=2))
    raise SystemExit(0 if r['all_pass'] else 2)
