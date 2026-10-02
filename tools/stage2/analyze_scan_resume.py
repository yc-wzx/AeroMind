#!/usr/bin/env python3
"""Read-only native timeline and retry/plan geometry analysis. No ROS output."""
import csv
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_localized import ROOT, read, yaw, load_jsonl
from analyze_provincial_forward_clearance import walls_from_sdf
from provincial_safety_geometry import rectangle, polygon_distance


def analyze(directory, output):
    if output.exists():raise FileExistsError(output)
    output.mkdir()
    a=read(directory/'independent_audit_v2.json');name=read(directory/'progress.json')['name']
    rows=load_jsonl(directory/(name+'.native.jsonl'));s=a['summary'];loss=a['loss_resume']
    walls=walls_from_sdf(directory/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf')
    latest={};timeline=[];plans=[];retry_context=[]
    begin=s['goal_delivery']['publish_monotonic_ns']
    for r in rows:
        latest[r['topic']]=r
        if r['receive_monotonic_ns']<begin:continue
        if r['topic']=='/gazebo/odometry':
            d=r['data'];p=d['pose']['pose'];v=d['twist']['twist'];ang=yaw(p['orientation'])
            cmd=latest.get('/model/omni_robot/cmd_vel');raw=latest.get('/cmd_vel')
            diag=latest.get('/ground/planning/actuation_diagnostics')
            diag=json.loads(diag['data']['data']) if diag else {}
            cm=cmd['data'] if cmd else None;rawc=raw['data'] if raw else None
            cv=cm['linear'] if cm else {}
            timeline.append(dict(sim_s=r['message_stamp_s'],receive_monotonic_ns=r['receive_monotonic_ns'],
                 x=p['position']['x'],y=p['position']['y'],yaw_rad=ang,
                 gt_speed=math.hypot(v['linear']['x'],v['linear']['y']),
                 final_speed=math.hypot(cv['x'],cv['y']) if cm else None,
                 final_vx_world=(math.cos(ang)*cv['x']-math.sin(ang)*cv['y']) if cm else None,
                 final_vy_world=(math.sin(ang)*cv['x']+math.cos(ang)*cv['y']) if cm else None,
                 raw_speed=math.hypot(rawc['linear']['x'],rawc['linear']['y']) if rawc else None,
                 command_receive_age_s=(r['receive_monotonic_ns']-cmd['receive_monotonic_ns'])*1e-9 if cmd else None,
                 action=diag.get('action'),source=diag.get('source'),waypoint=diag.get('waypoint'),
                 planned_gap=diag.get('planned_path_min_gap_m')))
        elif r['topic']=='/ground/planning/planned_trajectory':
            d=r['data'];pts=d['points'];poses=latest.get('/ground/odometry',{}).get('data',{}).get('pose',{}).get('pose',{})
            angle=yaw(poses['orientation']) if poses else math.pi/2
            found=[]
            for i,p in enumerate(pts):
                q=rectangle(p['x'],p['y'],angle,.52,.42)
                values=[(polygon_distance(q,w['polygon']),w['name']) for w in walls]
                gap,label=min(values)
                found.append(dict(index=i,x=p['x'],y=p['y'],gap=gap,wall=label))
            plans.append(dict(stamp_s=r['message_stamp_s'],receive_sim_s=r['receive_sim_s'],
                              sampled_using_context_yaw_rad=angle,minimum=min(found,key=lambda v:v['gap']),
                              sample_count=len(pts),scope='Context yaw diagnostic, not an independent rerun of the production future-yaw guard'))
        elif r['topic']=='/rosout/relevant' and 'Retrying RMUC stage' in r['data'].get('msg',''):
            diag=latest.get('/ground/planning/actuation_diagnostics')
            gt=latest.get('/gazebo/odometry');relay=latest.get('/scan_relay/status')
            retry_context.append(dict(raw_log=r,
                  last_actuation=json.loads(diag['data']['data']) if diag else None,
                  last_actuation_receive_age_s=(r['receive_monotonic_ns']-diag['receive_monotonic_ns'])*1e-9 if diag else None,
                  gt_stamp=gt['message_stamp_s'] if gt else None,
                  gt=gt['data'] if gt else None,
                  relay=json.loads(relay['data']['data']) if relay else None))
    with (output/'native_gt_timeline.csv').open('x') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(timeline[0]));writer.writeheader();writer.writerows(timeline)
    data=dict(retry_context=retry_context,plans=plans,
         strict_all_pass=a['all_pass'],failure_gates=[k for k,v in a['checks'].items() if not v],
         preserved_thresholds=True,raw_gt_rows=len(timeline),sampled_minimum=a['native']['minimum'],
         scope='No new simulation, no interpolation. Latest prior-received context is not message generation-time synchronization.')
    (output/'analysis.json').write_text(json.dumps(data,indent=2)+'\n')
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    ts=[v['sim_s'] for v in timeline]
    axes[0].plot(ts,[v['y'] for v in timeline],label='GT y');axes[0].axhline(1.15,color='gray',linestyle='--',label='goal')
    axes[0].set_ylabel('y (m)')
    axes[1].plot(ts,[v['gt_speed'] for v in timeline],label='GT speed')
    axes[1].plot(ts,[v['final_speed'] for v in timeline],label='final command speed',alpha=.7)
    axes[1].plot(ts,[v['raw_speed'] for v in timeline],label='upstream speed',alpha=.5)
    axes[1].set_ylabel('speed (m/s)')
    axes[2].plot(ts,[v['planned_gap'] for v in timeline],label='guard planned gap')
    axes[2].axhline(.08,color='red',linestyle='--',label='0.08m gate');axes[2].set_ylabel('gap (m)')
    for ax in axes:
        ax.axvspan(loss['hold_sim_s'],loss['resume_sim_s'],color='orange',alpha=.2,label='relay held')
        ax.axvline(loss['final_zero_sim_s'],color='green',linestyle=':',label='final zero')
        if retry_context:ax.axvline(retry_context[0]['gt_stamp'],color='red',linestyle=':',label='internal retry')
        ax.legend(loc='upper left',ncol=3);ax.grid(alpha=.2)
    axes[2].set_xlabel('native simulation time (s)')
    fig.suptitle('Single short scan-loss/resume trial: guard exercised, strict FAIL (one retry)')
    fig.tight_layout();fig.savefig(output/'stop_resume_timeline.png',dpi=150);plt.close(fig)
    return data


if __name__=='__main__':
    directory=Path(sys.argv[1]);output=Path(sys.argv[2])
    print(json.dumps(analyze(directory,output),ensure_ascii=False))
