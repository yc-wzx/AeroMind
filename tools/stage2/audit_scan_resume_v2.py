#!/usr/bin/env python3
"""Versioned short-goal native audit; retain every strict retry/safety gate."""
import argparse
import json
from pathlib import Path
import sys
from audit_scan_resume import audit as first_audit
from audit_localized import ROOT, read, sha
from summarize_provincial_full_observed import native_audit
from analyze_provincial_forward_clearance import walls_from_sdf


def audit(directory):
    result=first_audit(directory)
    name=read(directory/'progress.json')['name']
    manifest=read(directory/'input_manifest.json')
    expected=manifest['mission_plan']['external_goal_xy']
    if expected!=[4.7,1.15]:raise ValueError('Not this short test specification')
    native=native_audit(directory,result['summary'],
        walls_from_sdf(directory/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf'),
        name,expected_goal=expected)
    native.pop('gt_trace',None)
    result['native']=native
    result['checks']['native_raw_evidence']=native['pass']
    result['all_pass']=all(result['checks'].values())
    result['live_scan_loss_guard_verified']=result['loss_resume']['pass'] and native['pass']
    result['strict_trial_verified']=result['all_pass']
    result['correction_note']='v1 omitted native_audit expected_goal and inherited Full coordinates. v2 supplies the sealed short-goal specification; no retry/safety/terminal threshold is removed.'
    result['evaluator_sha256']=sha(Path(__file__))
    result['evaluator_dependency_sha256']['tools/stage2/audit_scan_resume.py']=sha(Path(__file__).with_name('audit_scan_resume.py'))
    result['current_evaluator_not_runtime_input']='This version was added after the single live run. Runtime evaluator v1 is preserved in input_snapshot; runtime hashes are not replaced.'
    result['command']=sys.argv
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    try:r=audit(a.result_dir)
    except (OSError,ValueError,KeyError,TypeError,IndexError) as error:
        r={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':repr(error)}
    a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'all_pass':r['all_pass'],'checks':r.get('checks'),'loss_resume':r.get('loss_resume'),'error':r.get('error')},ensure_ascii=False))
    sys.exit(0 if r['all_pass'] else 2)
