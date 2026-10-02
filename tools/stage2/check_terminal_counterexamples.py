#!/usr/bin/env python3
"""Counterexamples on temporary copies only; no simulator or publishers."""
import copy
import json
from pathlib import Path
import shutil
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from audit_trial import stage2_waypoint_sequence
from stage2_terminal_evidence import stage2_native_terminal_audit
from summarize_provincial_stage15_terminal_v3 import load_jsonl

BASE=ROOT/'tools/results/stage2_steps123_20261001'
source=BASE/'scale';name='stage2_scale_01'
summary=json.loads((source/(name+'.summary.json')).read_text())
preflight=json.loads((source/(name+'.offline_preflight.json')).read_text())
results={}

def write_rows(path,rows):path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
def check(out):
    waypoint=stage2_waypoint_sequence(out,name,summary,preflight)
    return stage2_native_terminal_audit(out,summary,waypoint,name)

for scenario in ('valid','missing_first_completion','wrong_completion_order','stable_2s_gt_speed','post_5s_nonzero_output','final_truth_error','missing_window_gt'):
    with tempfile.TemporaryDirectory(prefix='aeromind_stage2_evidence_') as d:
        out=Path(d)
        for suffix in ('.events.jsonl','.native.jsonl','.offline_preflight.json'):
            shutil.copyfile(source/(name+suffix),out/(name+suffix))
        events=load_jsonl(out/(name+'.events.jsonl'));rows=load_jsonl(out/(name+'.native.jsonl'))
        start=summary['stable_stop_start_sim_s'];middle=summary['post_stop_observation_start_sim_s']
        if scenario=='missing_first_completion':
            events=[e for e in events if 'waypoint=R0001:W00' not in e.get('message','')]
            rows=[r for r in rows if not(r['topic']=='/rosout/relevant' and 'waypoint=R0001:W00' in r['data'].get('msg',''))]
        elif scenario=='wrong_completion_order':
            indices=[i for i,e in enumerate(events) if 'RMUC waypoint diagnostic completed' in e.get('message','')]
            events[indices[0]],events[indices[1]]=events[indices[1]],events[indices[0]]
        elif scenario=='stable_2s_gt_speed':
            for r in rows:
                if r['topic']=='/gazebo/odometry' and start<=r['message_stamp_s']<middle:r['data']['twist']['twist']['linear']['x']=.1
        elif scenario=='post_5s_nonzero_output':
            for r in rows:
                if r['topic']=='/model/omni_robot/cmd_vel' and r['receive_sim_s'] is not None and middle<=r['receive_sim_s']:r['data']['linear']['x']=.1
        elif scenario=='final_truth_error':
            for r in rows:
                if r['topic']=='/gazebo/odometry':r['data']['pose']['pose']['position']['x']+=.1
        elif scenario=='missing_window_gt':
            rows=[r for r in rows if not(r['topic']=='/gazebo/odometry' and start+.5<=r['message_stamp_s']<=start+.8)]
        write_rows(out/(name+'.events.jsonl'),events);write_rows(out/(name+'.native.jsonl'),rows)
        report=check(out);expected=scenario=='valid'
        results[scenario]={'actual_pass':report['pass'],'expected_pass':expected,'matches':report['pass']==expected,
                           'reason':report.get('reason'),'checks':report.get('checks')}
report={'all_pass':all(r['matches'] for r in results.values()),'cases':results,
        'original_files_modified':False,'scope':'Waypoints and independent native terminal audit; modifications made in automatically removed temporary copies.'}
target=BASE/'terminal_counterexamples_v1.json'
if target.exists():raise FileExistsError(target)
target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));sys.exit(0 if report['all_pass'] else 2)
