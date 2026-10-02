#!/usr/bin/env python3
"""Historical task audit plus the new STATIC lidar input contract.

Does not claim historical navigation ran the new guard, or simulate actual
receive-clock freshness through a recorder in place of the navigation node.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import types
from audit_localized_replay import verify, ROOT, sha, read, load_jsonl
from stage2_wall_localization import validate_scan_metadata, verify_lidar_contract


def verify_guard(directory):
    result=verify(directory);name=read(directory/'progress.json')['name']
    rows=load_jsonl(directory/(name+'.native.jsonl'))
    sdf=directory/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf'
    contract=verify_lidar_contract(sdf)
    used={round(json.loads(r['data']['data'])['stamp_s'],8) for r in rows
          if r['topic']=='/localization/diagnostics' and json.loads(r['data']['data']).get('accepted') is True}
    scans={round(r['message_stamp_s'],8):r for r in rows if r['topic']=='/scan'}
    failures=[]
    for stamp in used:
        r=scans.get(stamp)
        if r is None:
            failures.append({'stamp':stamp,'reason':'accepted scan missing'});continue
        d=r['data']
        try:
            validate_scan_metadata(types.SimpleNamespace(**d))
            if d['header']['frame_id']!=contract['frame_id']:
                raise ValueError('unexpected scan frame')
            h=d['header']['stamp']
            if h['sec']<0 or not 0<=h['nanosec']<1_000_000_000 or abs(h['sec']+h['nanosec']*1e-9-stamp)>1e-8:
                raise ValueError('invalid/inconsistent timestamp')
        except (ValueError,TypeError,AttributeError) as error:
            failures.append({'stamp':stamp,'reason':str(error)})
    result['static_scan_contract']={'pass':bool(used) and not failures,'checked_used_scans':len(used),
       'failures':failures,'sdf_contract':contract,
       'scope':'Metadata/frame/configured extrinsics only. Recorder receive clocks do not prove new production freshness handling.'}
    result['checks']['static_scan_contract']=result['static_scan_contract']['pass']
    result['all_pass']=all(result['checks'].values())
    manifest=read(directory/'input_manifest.json')
    result['current_workspace_differs_from_runtime']=[p for p,v in manifest['inputs'].items() if sha(ROOT/p)!=v['sha256']]
    result['historical_navigation_ran_new_input_guard']=False
    result['live_input_guard_verified']=False
    result['evaluator_sha256']=sha(Path(__file__))
    result['evaluator_dependency_sha256']['src/uav_planning/scripts/stage2_localized_interface.py']=sha(ROOT/'src/uav_planning/scripts/stage2_localized_interface.py')
    result['command']=sys.argv
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=verify_guard(a.result_dir)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'all_pass':result['all_pass'],'scan_contract':result['static_scan_contract'],'runtime_current_differences':result['current_workspace_differs_from_runtime']},ensure_ascii=False))
    raise SystemExit(0 if result['all_pass'] else 2)
