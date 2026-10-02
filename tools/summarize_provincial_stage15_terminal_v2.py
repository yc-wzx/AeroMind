#!/usr/bin/env python3
"""Recalculate terminal PASS from saved Gazebo, SDF and direct command traces."""
import argparse
import csv
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import body_wall_gap, rectangle


def sdf_walls():
    walls=[]
    tree=ET.parse(ROOT/'src/uav_bringup/worlds/provincial_2025_training.sdf')
    for model in tree.findall('.//model'):
        if not model.get('name','').startswith('boundary_'):
            continue
        pose=[float(x) for x in model.findtext('pose').split()]
        size=[float(x) for x in model.findtext('link/collision/geometry/box/size').split()]
        walls.append(rectangle(pose[0],pose[1],pose[5],size[0],size[1]))
    return walls


def max_gap(values):
    return max((b-a for a,b in zip(values,values[1:])),default=math.inf)


def summarize(out,name,walls):
    s=json.loads((out/f'{name}.summary.json').read_text())
    with (out/f'{name}.csv').open(newline='') as f:
        rows=[{key:float(value) for key,value in row.items()}
              for row in csv.DictReader(f)]
    published_path=out/f'{name}.published_commands.jsonl'
    published=([json.loads(line) for line in published_path.read_text().splitlines()]
               if published_path.exists() else [])
    obs_start=s.get('post_stop_observation_start_sim_s')
    snapshot=s.get('terminal_snapshot') or {}
    end=snapshot.get('gt_stamp_s')
    obs=[row for row in rows if obs_start is not None and end is not None
         and obs_start<=row['sim_t']<=end]
    cmds=[item for item in published if item.get('gt_stamp_s') is not None
          and obs_start is not None and end is not None
          and obs_start<=item['gt_stamp_s']<=end]
    completion=s.get('navigation_completion') or {}
    goal=s.get('goal_xy')
    reported=snapshot.get('final_command')
    minimum=min((body_wall_gap(row['gt_x'],row['gt_y'],row['gt_yaw'],walls)
                 for row in rows),default=math.nan)
    yaw_errors=[abs(math.atan2(math.sin(row['gt_yaw']-s['sent_goal_yaw_rad']),
                                    math.cos(row['gt_yaw']-s['sent_goal_yaw_rad'])))
                for row in rows]
    critical=('gt_x','gt_y','gt_yaw','gt_speed','gt_yaw_speed','wall_clearance',
              'odom_x','odom_y','published_vx','published_vy','published_wz')
    finite=all(all(math.isfinite(row[key]) for key in critical) for row in rows)
    direct_finite=all(all(isinstance(item.get(key),(int,float)) and
                          math.isfinite(item[key]) for key in ('vx','vy','wz'))
                      for item in published)
    completion_valid=(s.get('goal_accepted') and
                      completion.get('waypoint_id')==s.get('final_waypoint_id') and
                      completion.get('gt_fresh') and
                      isinstance(completion.get('gt_goal_error_m'),(int,float)) and
                      completion['gt_goal_error_m']<=0.05)
    snapshot_valid=(goal is not None and snapshot.get('gt_xy') is not None and
                    math.isfinite(s['gt_goal_error_m']) and
                    abs(math.dist(snapshot['gt_xy'],goal)-s['gt_goal_error_m'])<1e-9)
    window_valid=(len(obs)>=2 and len(cmds)>=2 and end-obs_start>=5.0 and
                  obs[0]['sim_t']<=obs_start+0.1 and
                  obs[-1]['sim_t']>=end-0.1 and
                  cmds[0]['gt_stamp_s']<=obs_start+0.2 and
                  cmds[-1]['gt_stamp_s']>=end-0.2 and
                  max_gap([x['sim_t'] for x in obs])<=0.2 and
                  max_gap([x['wall_s'] for x in obs])<=0.2 and
                  max_gap([x['wall_s'] for x in cmds])<=0.2)
    speeds_valid=(len(obs)>=2 and len(cmds)>=2 and
                  max(x['gt_speed'] for x in obs)<=0.02 and
                  max(x['gt_yaw_speed'] for x in obs)<=0.03 and
                  max(math.hypot(x['vx'],x['vy']) for x in cmds)<=0.02 and
                  max(abs(x['wz']) for x in cmds)<=0.03)
    last_cmd_valid=(reported is not None and
                    all(abs(reported[key])<=0.02 if key!='wz'
                        else abs(reported[key])<=0.03 for key in ('vx','vy','wz')) and
                    snapshot.get('final_command_received_age_s') is not None and
                    snapshot['final_command_received_age_s']<=0.2)
    checks={
        'accepted_and_final_completion':bool(completion_valid),
        'fresh_snapshot_consistent':bool(snapshot_valid),
        'no_false_success_or_departure':not s.get('false_success') and
            not s.get('post_completion_departure'),
        'no_invalid_window_reason':not s.get('terminal_invalid_reasons'),
        'stable_2s_and_observed_5s':s.get('stable_stop_2s') and
            s.get('post_stop_observation_5s') and obs_start is not None and
            s.get('stable_stop_start_sim_s') is not None and
            obs_start-s['stable_stop_start_sim_s']>=2.0,
        'direct_command_trace_available':bool(published),
        'all_critical_samples_finite':finite and direct_finite,
        'continuous_observation_evidence':bool(window_valid),
        'observation_speed_and_output_limits':bool(speeds_valid),
        'final_direct_command_fresh_and_stopped':bool(last_cmd_valid),
        'sampled_geometry':math.isfinite(minimum) and minimum>=0.08 and
            abs(minimum-s['minimum_physical_wall_clearance_m'])<1e-9 and
            s['geometric_overlap_samples']==0,
        'no_retry_or_stuck':s['retry_count']==0 and s['stuck_count']==0,
    }
    return {
        'name':name,'route':s['route'],'result':s['result'],
        'checks':checks,'pass':s['result']=='terminal_pass' and all(checks.values()),
        'initial_gt_pose':s['initial_gt_pose'],
        'sent_goal_yaw_rad':s['sent_goal_yaw_rad'],
        'final_gt_yaw_rad':s['final_gt_yaw_rad'],
        'max_abs_yaw_error_rad':max(yaw_errors,default=None),
        'gt_error_m':s['gt_goal_error_m'],'odom_error_m':s['odom_goal_error_m'],
        'minimum_sampled_body_wall_gap_m':minimum,
        'sampled_overlap_count':s['geometric_overlap_samples'],
        'contact_data':s['contact_data'],
        'retry_count':s['retry_count'],'stuck_count':s['stuck_count'],
        'actuation_counts':s['actuation_counts'],
        'observation_gt_samples':len(obs),
        'observation_direct_command_samples':len(cmds),
        'observation_max_gt_speed_mps':max((x['gt_speed'] for x in obs),default=None),
        'observation_max_direct_speed_mps':max(
            (math.hypot(x['vx'],x['vy']) for x in cmds),default=None),
        'duration_sim_s':s['duration_sim_s'],
    }


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--names',nargs='+',required=True)
    args=parser.parse_args()
    out=args.output_dir.resolve()
    manifest=json.loads((out/'input_manifest.json').read_text())
    mismatches=[path for path,digest in manifest['sha256'].items()
                if hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=digest]
    walls=sdf_walls()
    runs=[summarize(out,name,walls) for name in args.names]
    result={'classification':'TRAINING-ONLY / PROVISIONAL',
            'input_hash_mismatches':mismatches,
            'runs':runs,
            'all_pass':not mismatches and all(run['pass'] for run in runs)}
    target=out/'terminal_evidence_v2.json'
    if target.exists():
        raise FileExistsError(target)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({run['name']:run['pass'] for run in runs}))
    if not result['all_pass']:
        raise SystemExit(2)


if __name__=='__main__':
    main()
