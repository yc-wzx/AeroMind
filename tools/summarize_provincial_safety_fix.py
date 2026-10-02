#!/usr/bin/env python3
"""Summarize the isolated Stage 1.5 trials with distinct error definitions."""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import reference_stages

OUT=ROOT/'tools/results/provincial_stage15_safety_fix'
FIELD=json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())
GOALS={'A':(4.7,1.15),'B':(5.8,1.8),'C':(8.7,4.25)}
NAMES={'A':['trial_a_02','trial_a_03','trial_a_04'],
       'B':['trial_b_05','trial_b_06','trial_b_07'],
       'C':['trial_c_04','trial_c_05','trial_c_06']}


def stats(values):
    vector=np.asarray([v for v in values if math.isfinite(v)],dtype=float)
    return {'mean':round(float(vector.mean()),5),
            'p95':round(float(np.percentile(vector,95)),5),
            'max':round(float(vector.max()),5)} if len(vector) else None


def nearest_segment(point, line):
    best=math.inf
    for a,b in zip(line,line[1:]):
        vx,vy=b[0]-a[0],b[1]-a[1]
        length2=vx*vx+vy*vy
        ratio=max(0.0,min(1.0,((point[0]-a[0])*vx+(point[1]-a[1])*vy)/length2))
        best=min(best,math.hypot(point[0]-a[0]-ratio*vx,
                                point[1]-a[1]-ratio*vy))
    return best


def summarize(name,letter):
    summary=json.loads((OUT/(name+'.summary.json')).read_text())
    with (OUT/(name+'.csv')).open(newline='') as stream:
        trace=[{key:float(value) for key,value in row.items()}
               for row in csv.DictReader(stream)]
    start=tuple(summary['initial_gt_pose'][:2])
    stages=reference_stages(start,GOALS[letter],FIELD['reference_route'])
    line=[start]+stages
    valid=[row for row in trace if math.isfinite(row['setpoint_x'])
           and math.isfinite(row['setpoint_y'])]
    tree=cKDTree([(row['setpoint_x'],row['setpoint_y']) for row in valid])
    spatial,_=tree.query([(row['gt_x'],row['gt_y']) for row in trace])
    return {
        'name':name,'result':summary['result'],'assessment':summary['assessment'],
        'yaw_mode':summary['yaw_mode'],
        'initial_gt_pose':summary['initial_gt_pose'],
        'sent_goal_yaw_rad':summary['sent_goal_yaw_rad'],
        'final_gt_yaw_rad':summary['final_gt_yaw_rad'],
        'gt_goal_error_m':round(summary['gt_goal_error_m'],5),
        'odom_goal_error_m':round(summary['odom_goal_error_m'],5),
        'duration_sim_s':round(summary['duration_sim_s'],3),
        'min_gt_body_wall_gap_m':round(summary['minimum_physical_wall_clearance_m'],5),
        'geometric_overlap_samples':summary['geometric_overlap_samples'],
        'contact_data':summary['contact_data'],
        'retry_count':summary['retry_count'],'stuck_count':summary['stuck_count'],
        'actuation_counts':summary['actuation_counts'],
        'actuation_source_counts':summary.get('actuation_source_counts',{}),
        'commanded_to_configured_reference_m':stats(
            nearest_segment((row['setpoint_x'],row['setpoint_y']),line)
            for row in valid),
        'gt_to_synchronous_commanded_m':stats(
            row['setpoint_error'] for row in valid),
        'gt_to_sampled_commanded_spatial_m':stats(spatial),
        'gt_to_nominal_corridor_center_m':stats(
            row['centerline_cte'] for row in trace),
    }


def main():
    result={'classification':'TRAINING-ONLY / PROVISIONAL',
            'final_arena_verified':False,
            'minimum_body_wall_trial_gate_m':.08,
            'invalidated_runs':{'trial_b_01':'Two Gazebo servers published same world; mixed odometry.'},
            'routes':{}}
    for letter,names in NAMES.items():
        runs=[summarize(name,letter) for name in names]
        result['routes'][letter]={'runs':runs,
            'all_gt_arrived':all(run['result']=='success' for run in runs),
            'min_gap_across_runs_m':min(run['min_gt_body_wall_gap_m'] for run in runs),
            'total_retries':sum(run['retry_count'] for run in runs),
            'position_and_geometry_gate':all(
                run['result']=='success' and run['min_gt_body_wall_gap_m']>=.08 and
                run['geometric_overlap_samples']==0 and run['stuck_count']==0
                for run in runs),
            'all_pass_existing_assessment':all(run['assessment']=='PASS' for run in runs)}
    (OUT/'aggregate.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(OUT/'aggregate.json')


if __name__=='__main__':
    main()
