"""Read-only comparison with actual parameter and loaded-component evidence."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import yaml

ROOT=Path(__file__).resolve().parents[3]
TOOL=ROOT/'tools/stage2/lio_optimization/analyze_variance.py'
spec=importlib.util.spec_from_file_location('preserved_pose_comparison',TOOL)
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)


def flatten(values, prefix=''):
    result={}
    for name,value in values.items():
        key=prefix+name
        if isinstance(value,dict):result.update(flatten(value,key+'.'))
        else:result[key]=value
    return result


def evaluate(source,replay):
    r=old.analyze(source,replay)
    inp=json.loads((replay/'input_manifest.json').read_text())
    requested=inp['parameter_overrides']
    actual=yaml.safe_load((replay/'offline_lio__offline_lio_mapping.yaml').read_text())
    params=flatten(next(iter(actual.values()))['ros__parameters'])
    loaded=json.loads((replay/'actually_loaded_LIO_component.json').read_text())
    library=ROOT/'install/stage2_lio_sim/lib/lio_point_boundary/libspark_lio_component.so'
    r['checks']['only_LIO_voxel_parameter_changed']=set(requested)=={'filter_size_map'} and requested['filter_size_map'] in [.1,.05]
    r['checks']['actual_runtime_parameter']=params.get('filter_size_map')==requested['filter_size_map']
    r['checks']['actual_corrected_LIO_component']=loaded['paths']==[str(library.resolve())] and loaded['sha256']=={str(library.resolve()):old.sha(library)}
    r['checks']['full_3D_noise_and_extrinsics_unchanged']=all(
        params.get(k)==inp['original_parameters'].get(k) for k in
        ['common.planar_mode','mapping.acc_cov','mapping.gyr_cov','mapping.b_acc_cov','mapping.b_gyr_cov',
         'mapping.extrinsic_est_en','mapping.extrinsic_T','mapping.extrinsic_R','common.time_sync_en'])
    r['evidence_pass']=all(r['checks'].values())
    r['pose_evaluator_sha256']=old.sha(TOOL)
    r['voxel_evaluator_sha256']=old.sha(Path(__file__))
    r['voxel_evaluator_command']=sys.argv
    r['runtime_filter_size_map']=params.get('filter_size_map')
    return r


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--replay',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    try:r=evaluate(a.source,a.replay)
    except Exception as e:r={'evidence_pass':False,'error':repr(e),'status':'INCOMPLETE_EVIDENCE'}
    with a.output.open('x') as f:json.dump(r,f,indent=2,allow_nan=False)
    with a.output.with_suffix('.tool.py').open('xb') as f:f.write(Path(__file__).read_bytes())
    print(json.dumps({k:r.get(k) for k in ['evidence_pass','checks','replayed','error']},indent=2))
    raise SystemExit(0 if r['evidence_pass'] else 2)
