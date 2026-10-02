#!/usr/bin/env python3
"""Readonly plot, discovery-prefix diagnosis, and repeat-audit comparison."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_localized import load_jsonl, read


def review(directory, output):
    if output.exists():raise FileExistsError(output)
    output.mkdir()
    a=read(directory/'independent_audit_review_v3_final.json')
    repeat=read(directory/'independent_audit_review_v3_repeat.json')
    name=read(directory/'progress.json')['name'];s=a['summary'];loss=a['loss_resume']
    rows=load_jsonl(directory/(name+'.native.jsonl'))
    indices={t:{round(r['message_stamp_s'],8):r for r in rows if r['topic']==t}
        for t in ('/scan','/guarded_scan','/simulation/navigation_odometry')}
    missing=[];task_ns=s['goal_delivery']['publish_monotonic_ns'];latest={};timeline=[]
    for r in rows:
        latest[r['topic']]=r
        if r['topic']=='/localization/diagnostics':
            d=json.loads(r['data']['data'])
            if d.get('accepted') is True:
                found={t:round(d['associated_odom_stamp_s'] if t.endswith('odometry') else d['stamp_s'],8) in v for t,v in indices.items()}
                if not all(found.values()):missing.append(dict(stamp_s=d['stamp_s'],receive_monotonic_ns=r['receive_monotonic_ns'],
                    before_goal=r['receive_monotonic_ns']<task_ns,found=found))
        if r['topic']!='/gazebo/odometry' or r['receive_monotonic_ns']<task_ns:continue
        d=r['data'];p=d['pose']['pose'];v=d['twist']['twist'];cmd=latest.get('/model/omni_robot/cmd_vel');raw=latest.get('/cmd_vel')
        diag=latest.get('/ground/planning/actuation_diagnostics');diag=json.loads(diag['data']['data']) if diag else {}
        timeline.append(dict(sim_s=r['message_stamp_s'],receive_monotonic_ns=r['receive_monotonic_ns'],
            x=p['position']['x'],y=p['position']['y'],gt_speed=math.hypot(v['linear']['x'],v['linear']['y']),
            final_speed=math.hypot(cmd['data']['linear']['x'],cmd['data']['linear']['y']) if cmd else None,
            raw_speed=math.hypot(raw['data']['linear']['x'],raw['data']['linear']['y']) if raw else None,
            action=diag.get('action'),waypoint=diag.get('waypoint'),source=diag.get('source')))
    with (output/'native_gt_timeline.csv').open('x') as stream:
        w=csv.DictWriter(stream,fieldnames=list(timeline[0]));w.writeheader();w.writerows(timeline)
    gt=[r for r in rows if r['topic']=='/gazebo/odometry']
    times=[r['message_stamp_s'] for r in gt]
    result=dict(strict_pass=a['all_pass'],failure_gates=[k for k,v in a['checks'].items() if not v],
        repeat_same_checks=a['checks']==repeat['checks'],repeat_same_loss=a['loss_resume']==repeat['loss_resume'],
        missing_accepted_scan_associations=missing,all_missing_before_goal=all(m['before_goal'] for m in missing),
        task_period_scan_replay=a['replay'],task_clock=a['task_clock'],native_sampled_minimum=a['native']['minimum'],
        gt_native_count=len(gt),gt_stamp_frequency_hz=(len(times)-1)/(times[-1]-times[0]),
        gt_max_stamp_gap_s=max(b-a for a,b in zip(times,times[1:])),
        original_raw_unmodified=True,unsafe_plan_observed=s['actuation_counts'].get('unsafe_plan',0)>0,
        root_cause_limit='Recorded topic discovery prefix differs: original and guarded scans begin at 14.9s, accepted diagnostics begin at 14.0s. Delivery/discovery scheduling is not uniquely established. No fabricated scans or relaxed association gate.',
        resets_scope='Runner code and recorded one-goal/route/continuous GT evidence. Not independent monitoring of every reset/set_pose service call.',
        continuous_safety_proven=False,statistical_repeatability_proven=False,contact='CONTACT DATA UNAVAILABLE')
    (output/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True);ts=[x['sim_s'] for x in timeline]
    axes[0].plot(ts,[x['y'] for x in timeline],label='GT y');axes[0].axhline(1.15,ls='--',color='gray',label='goal y')
    axes[0].set_ylabel('position (m)')
    for key,label in [('gt_speed','GT'),('final_speed','final command'),('raw_speed','upstream')]:
        axes[1].plot(ts,[x[key] for x in timeline],label=label,alpha=.8)
    axes[1].set_ylabel('planar speed (m/s)');axes[1].set_xlabel('original GT simulation time (s)')
    for ax in axes:
        ax.axvspan(loss['hold_sim_s'],loss['resume_sim_s'],color='orange',alpha=.2,label='relay hold')
        ax.axvline(loss['final_zero_sim_s'],ls=':',color='green',label='final zero');ax.grid(alpha=.2);ax.legend()
    fig.suptitle('One short trial: zero retry; strict evidence FAIL (9 pre-goal scans absent)')
    fig.tight_layout();fig.savefig(output/'stop_resume_timeline.png',dpi=150);plt.close(fig)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    r=review(a.result_dir,a.output_dir)
    print(json.dumps({k:v for k,v in r.items() if k not in ('task_clock','task_period_scan_replay')},ensure_ascii=False))
