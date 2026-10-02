#!/usr/bin/env python3
"""Versioned read-only audit plus re-running matches on recorded raw scans."""
import argparse
import json
import math
from pathlib import Path
import sys
import types
import yaml
from audit_localized import audit, sha, read, load_jsonl, yaw, ROOT
from stage2_wall_localization import FaceMatcher


def replay(directory, name):
    rows=load_jsonl(directory/(name+'.native.jsonl'))
    start=read(directory/(name+'.summary.json'))['goal_delivery']['publish_monotonic_ns']
    raw={round(r['message_stamp_s'],8):r for r in rows if r['topic']=='/simulation/navigation_odometry'}
    scans={round(r['message_stamp_s'],8):r for r in rows if r['topic']=='/scan'}
    corrected=[r for r in rows if r['topic']=='/localization/navigation_odometry' and r['receive_monotonic_ns']>=start]
    diagnostics=[r for r in rows if r['topic']=='/localization/diagnostics' and r['receive_monotonic_ns']>=start]
    accepted=[json.loads(r['data']['data']) for r in diagnostics if json.loads(r['data']['data']).get('accepted') is True]
    nav=next(iter(yaml.safe_load((directory/'runtime_parameters/gazebo_navigation_interface.yaml').read_text()).values()))['ros__parameters']
    field=read(directory/'input_snapshot/src/uav_planning/config/provincial_2025_provisional.json')
    matcher=FaceMatcher(field['collision_segments'],nav['provincial_wall_thickness_m'])
    outcomes=[];maximum_delta_difference=0.;stamps=[]
    for d in accepted:
        source=raw.get(round(d['associated_odom_stamp_s'],8));scan=scans.get(round(d['stamp_s'],8))
        if source is None or scan is None:
            outcomes.append(False);continue
        p=source['data']['pose']['pose']
        pose=(p['position']['x'],p['position']['y'],yaw(p['orientation']))
        result=matcher.match(d['prior_pose'],types.SimpleNamespace(**scan['data']))
        difference=math.dist(result.get('delta',[math.inf]*3),d['delta'])
        maximum_delta_difference=max(maximum_delta_difference,difference)
        outcomes.append(math.dist(pose,d['raw_pose'])<1e-10 and result['accepted'] and difference<1e-7 and
                        result['inliers']==d['inliers'])
        stamps.append(d['stamp_s'])
    # Every task-period accepted raw source output must have a corrected
    # counterpart. Limit at the last recorded corrected stamp, report tail.
    keys={round(r['message_stamp_s'],8) for r in corrected}
    end=max((r['message_stamp_s'] for r in corrected),default=-math.inf)
    expected={k for k,r in raw.items() if r['receive_monotonic_ns']>=start and r['message_stamp_s']<=end}
    checks={'accepted_scan_matches_recomputed':bool(outcomes) and all(outcomes),
            'accepted_scan_stamps_increasing':bool(stamps) and all(b>a for a,b in zip(stamps,stamps[1:])),
            'all_task_raw_odometry_has_corrected_output':bool(expected) and expected<=keys,
            'corrected_stamps_unique':len(keys)==len(corrected)}
    return {'pass':all(checks.values()),'checks':checks,'replayed_matches':len(outcomes),
            'maximum_delta_difference':maximum_delta_difference,
            'missing_corrected_samples':len(expected-keys),
            'raw_tail_beyond_last_corrected':sum(r['message_stamp_s']>end for r in raw.values()),
            'scope':'Every received scan used for an accepted task-period correction; no invented discovery-prefix scans.'}


def verify(directory):
    result=audit(directory)
    result['localization_replay']=replay(directory,read(directory/'progress.json')['name'])
    result['checks']['raw_scan_matching_replayed']=result['localization_replay']['pass']
    result['all_pass']=all(result['checks'].values())
    result['evaluator_sha256']=sha(Path(__file__))
    result['evaluator_dependency_sha256']={str(p.relative_to(ROOT)):sha(p) for p in
        (Path(__file__).with_name('audit_localized.py'),Path(__file__).with_name('audit_trial.py'),
         Path(__file__).with_name('stage2_terminal_evidence.py'),ROOT/'src/uav_planning/scripts/stage2_wall_localization.py')}
    result['command']=sys.argv
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=verify(a.result_dir)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'all_pass':result['all_pass'],'checks':result['checks'],'replay':result['localization_replay']},ensure_ascii=False))
    sys.exit(0 if result['all_pass'] else 2)
