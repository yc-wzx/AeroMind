#!/usr/bin/env python3
"""Independently recheck final Stage 1.5 terminal trials against SDF boxes."""
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import body_wall_gap, rectangle

OUT = ROOT/'tools/results/provincial_stage15_terminal_verified_20260928'
NAMES = ('trial_a_31', 'trial_b_20', 'trial_c_27')
WORLD = ROOT/'src/uav_bringup/worlds/provincial_2025_training.sdf'


def walls_from_sdf():
    walls = []
    for model in ET.parse(WORLD).findall('.//model'):
        if not model.get('name', '').startswith('boundary_'):
            continue
        pose = [float(value) for value in model.findtext('pose').split()]
        size = [float(value) for value in
                model.findtext('link/collision/geometry/box/size').split()]
        walls.append(rectangle(pose[0],pose[1],pose[5],size[0],size[1]))
    return walls


def summarize(name, walls):
    summary = json.loads((OUT/f'{name}.summary.json').read_text())
    with (OUT/f'{name}.csv').open(newline='') as stream:
        rows = [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]
    diagnostics = [json.loads(line) for line in
                   (OUT/f'{name}.actuation.jsonl').read_text().splitlines()]
    minimum = min(body_wall_gap(r['gt_x'],r['gt_y'],r['gt_yaw'],walls)
                  for r in rows)
    snapshot = summary['terminal_snapshot']
    goal = summary['goal_xy']
    final5 = [r for r in rows if r['sim_t'] >= snapshot['gt_stamp_s']-5]
    applied = [d for d in diagnostics
               if d['gt_stamp_s'] is not None and
               d['gt_stamp_s'] >= snapshot['gt_stamp_s']-5]
    if not final5 or not applied:
        raise ValueError(f'incomplete stop window in {name}')
    snapshot_error = math.dist(snapshot['gt_xy'],goal)
    checks = {
        'snapshot_matches_summary': abs(snapshot_error-summary['gt_goal_error_m']) < 1e-9,
        'sdf_gap_matches_trace': abs(minimum-summary['minimum_physical_wall_clearance_m']) < 1e-9,
        'goal_and_completion': summary['goal_accepted'] and
            summary['navigation_completion'] is not None and
            summary['navigation_completion']['gt_fresh'],
        'no_false_success_or_departure': not summary['false_success'] and
            not summary['post_completion_departure'],
        'stable_stop': summary['stable_stop_2s'] and
            summary['post_stop_observation_5s'],
        'within_static_gap_gate': minimum >= 0.08 and
            summary['geometric_overlap_samples'] == 0,
        'no_retries_or_stuck': summary['retry_count'] == 0 and
            summary['stuck_count'] == 0,
        'final_applied_zero': snapshot['applied_command']['source'] ==
            'route_complete_stop' and all(
                abs(snapshot['applied_command'][key]) < 1e-9
                for key in ('vx','vy','wz')),
    }
    return {
        'name':name, 'route':summary['route'], 'result':summary['result'],
        'assessment':summary['assessment'], 'checks':checks,
        'pass':summary['result']=='terminal_pass' and all(checks.values()),
        'initial_gt_pose':summary['initial_gt_pose'],
        'final_gt_yaw_rad':summary['final_gt_yaw_rad'],
        'completion_sim_s':summary['navigation_completion']['gt_stamp_s'],
        'stable_stop_start_sim_s':summary['stable_stop_start_sim_s'],
        'observation_start_sim_s':summary['post_stop_observation_start_sim_s'],
        'final_gt_error_m':snapshot_error,
        'final_odom_error_m':summary['odom_goal_error_m'],
        'post_completion_max_error_m':summary['post_completion_max_error_m'],
        'min_sampled_body_wall_gap_m':minimum,
        'sampled_overlap_count':summary['geometric_overlap_samples'],
        'contact_data':summary['contact_data'],
        'retry_count':summary['retry_count'], 'stuck_count':summary['stuck_count'],
        'actuation_counts':summary['actuation_counts'],
        'last5_sim_coverage_s':final5[-1]['sim_t']-final5[0]['sim_t'],
        'last5_max_gt_speed_mps':max(r['gt_speed'] for r in final5),
        'last5_max_gt_yaw_speed_rad_s':max(r['gt_yaw_speed'] for r in final5),
        'last5_max_applied_planar_command_mps':max(
            math.hypot(d['vx'],d['vy']) for d in applied),
        'last5_max_applied_yaw_command_rad_s':max(abs(d['wz']) for d in applied),
        'last5_applied_sources':sorted(set(d['source'] for d in applied)),
        'last5_gt_net_displacement_m':math.dist(
            (final5[0]['gt_x'],final5[0]['gt_y']),
            (final5[-1]['gt_x'],final5[-1]['gt_y'])),
        'duration_sim_s':summary['duration_sim_s'],
    }


def main():
    walls = walls_from_sdf()
    runs = [summarize(name,walls) for name in NAMES]
    result = {
        'classification':'TRAINING-ONLY / PROVISIONAL',
        'final_arena_verified':False,
        'test_scope':'Isolated A/B/C, not Full or Return',
        'world':str(WORLD),
        'runs':runs,
        'all_terminal_pass':all(run['pass'] for run in runs),
        'limitations':['SDF/GT checks are sampled, not continuous contact data',
                       'Final waypoint diagnostic is the navigation-interface completion signal',
                       'Formal provincial course dimensions are unverified'],
    }
    target = OUT/'terminal_aggregate.json'
    if target.exists():
        raise FileExistsError(target)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(target)
    print(json.dumps({run['name']:run['pass'] for run in runs}))
    if not result['all_terminal_pass']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
