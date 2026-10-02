#!/usr/bin/env python3
"""Read-only analysis of the single observed Full; creates a new review folder."""
import csv
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import polygon_distance, rectangle
from analyze_provincial_forward_clearance import walls_from_sdf, nearest_plan

BASE = ROOT/'tools/results/provincial_stage15_full_delivery_20260929'
NAME = 'trial_full_delivery_01'
OUT = BASE/'offline_clearance_review_v1'
OLD = ROOT/'tools/results/provincial_stage15_forward_integration_20260928/single_full'
SDF = BASE/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open(newline='') as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def jsonl(path):
    return [json.loads(line) for line in path.open()]


def yaw(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                      1-2*(q['y']*q['y']+q['z']*q['z']))


def main():
    if OUT.exists():
        raise FileExistsError(OUT)
    manifest = json.loads((BASE/'input_manifest.json').read_text())
    world_relative = 'src/uav_bringup/worlds/provincial_2025_training.sdf'
    if sha(SDF) != manifest['inputs'][world_relative]['sha256']:
        raise ValueError('captured SDF hash mismatch')
    walls = walls_from_sdf(SDF)
    last = {}
    gaps = defaultdict(list)
    count = Counter()
    gt = []
    commands = []
    setpoints = []
    for line in (BASE/f'{NAME}.native.jsonl').open():
        entry = json.loads(line)
        topic = entry['topic']
        count[topic] += 1
        if topic in last:
            gaps[topic].append({
                'receive_gap_s': (entry['receive_monotonic_ns']-last[topic][0])*1e-9,
                'previous_sim_s': last[topic][1],
                'current_sim_s': entry['message_stamp_s']
                    if entry['message_stamp_s'] is not None else entry['receive_sim_s']})
        last[topic] = (entry['receive_monotonic_ns'],
                       entry['message_stamp_s'] if entry['message_stamp_s'] is not None
                       else entry['receive_sim_s'])
        if topic == '/gazebo/odometry':
            pose = entry['data']['pose']['pose']
            p, q = pose['position'], pose['orientation']
            angle = yaw(q)
            body = rectangle(p['x'], p['y'], angle, .52, .42)
            distance, wall = min(
                ((polygon_distance(body, item['polygon']), item['name']) for item in walls),
                key=lambda pair: pair[0])
            gt.append({'t': entry['message_stamp_s'], 'x': p['x'], 'y': p['y'],
                       'yaw': angle, 'gap': distance, 'wall': wall,
                       'receive_monotonic_ns': entry['receive_monotonic_ns']})
        elif topic == '/model/omni_robot/cmd_vel':
            commands.append({'t': entry['receive_sim_s'],
                             'vx': entry['data']['linear']['x'],
                             'vy': entry['data']['linear']['y'],
                             'wz': entry['data']['angular']['z']})
        elif topic == '/ground/planning/commanded_setpoint':
            setpoints.append({'t': entry['message_stamp_s'],
                              'x': entry['data']['pose']['position']['x'],
                              'y': entry['data']['pose']['position']['y']})
    summary = json.loads((BASE/f'{NAME}.summary.json').read_text())
    start = summary['terminal_snapshot']['gt_stamp_s']-summary['duration_sim_s']
    end = summary['terminal_snapshot']['gt_stamp_s']
    active = [item for item in gt if start <= item['t'] <= end]
    minimum = min(active, key=lambda item: item['gap'])
    old_rows = rows(OLD/'trial_full_01.csv')
    old_min = min(old_rows, key=lambda row: row['wall_clearance'])
    plans = jsonl(BASE/f'{NAME}.plans.jsonl')
    old_plans = jsonl(OLD/'trial_full_01.plans.jsonl')
    new_plan, new_point = nearest_plan(plans, minimum['t'], minimum['x'])
    old_plan, old_point = nearest_plan(old_plans, old_min['sim_t'], old_min['gt_x'])
    diag = jsonl(BASE/f'{NAME}.actuation.jsonl')

    OUT.mkdir()
    fig, ax = plt.subplots(figsize=(10, 6), dpi=160, constrained_layout=True)
    for wall in walls:
        poly = wall['polygon']
        if (max(x for x, _ in poly) < 4.55 or min(x for x, _ in poly) > 6.05 or
                max(y for _, y in poly) < 1.25 or min(y for _, y in poly) > 2.30):
            continue
        ax.add_patch(Polygon(poly, closed=True, facecolor='#9299a0', alpha=.7))
        if wall['name'] == minimum['wall']:
            ax.text(5.95, 2.24, wall['name'], ha='right', fontsize=9)
    band = [item for item in active if 4.6 <= item['x'] <= 6.05]
    old_band = [item for item in old_rows if 4.6 <= item['gt_x'] <= 6.05]
    ax.plot([item['x'] for item in band], [item['y'] for item in band],
            color='#c83232', lw=2, label='new Full native GT')
    ax.plot([item['gt_x'] for item in old_band],
            [item['gt_y'] for item in old_band], color='#22759f', lw=1.7,
            label='old Full sampled GT')
    ax.plot([4.7, 6.05], [1.8, 1.8], '--', color='black', lw=1,
            label='reference y=1.8')
    for plan, color, label in ((new_plan, '#ed9455', 'new local plan'),
                               (old_plan, '#4ea7c7', 'old local plan')):
        points = [point for point in plan['points'] if 4.6 <= point[0] <= 6.05]
        ax.plot([point[0] for point in points], [point[1] for point in points],
                ':', color=color, lw=2, label=label)
    body = rectangle(minimum['x'], minimum['y'], minimum['yaw'], .52, .42)
    ax.add_patch(Polygon(body, closed=True, fill=False,
                         edgecolor='#991515', linewidth=2))
    ax.scatter([minimum['x']], [minimum['y']], c='#991515', s=35)
    ax.annotate(f"{minimum['gap']:.4f} m", (minimum['x'], minimum['y']),
                xytext=(minimum['x']+.11, minimum['y']+.13),
                arrowprops={'arrowstyle': '->', 'color': '#991515'})
    ax.set(xlim=(4.55, 6.05), ylim=(1.25, 2.32),
           xlabel='world x (m)', ylabel='world y (m)',
           title='Native GT Full versus historical Full near boundary_7')
    ax.set_aspect('equal')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(alpha=.2)
    fig.savefig(OUT/'boundary_7_plan_view.png')
    plt.close(fig)

    window = [item for item in active if minimum['t']-2 <= item['t'] <= minimum['t']+2]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), dpi=160, sharex=True,
                             constrained_layout=True)
    axes[0].plot([item['t'] for item in window], [item['y'] for item in window],
                 color='#b93333', label='native GT y')
    nearby_setpoints = [item for item in setpoints if minimum['t']-2 <= item['t'] <= minimum['t']+2]
    axes[0].plot([item['t'] for item in nearby_setpoints],
                 [item['y'] for item in nearby_setpoints], '.', ms=2,
                 label='commanded setpoint y')
    axes[0].axhline(1.8, color='black', ls='--', lw=1, label='reference y')
    axes[0].set_ylabel('world y (m)')
    axes[1].plot([item['t'] for item in window], [item['gap'] for item in window],
                 color='#b93333', label='native sampled body-wall gap')
    diagnostics = [item for item in diag if item.get('gt_stamp_s') is not None and
                   minimum['t']-2 <= item['gt_stamp_s'] <= minimum['t']+2]
    axes[1].plot([item['gt_stamp_s'] for item in diagnostics],
                 [item.get('predicted_gap_m', math.nan) for item in diagnostics],
                 '.', ms=2, label='guard prediction')
    axes[1].axhline(.08, color='#991515', ls='--', lw=1, label='0.08 m gate')
    axes[1].set_ylabel('gap (m)')
    nearby_commands = [item for item in commands if item['t'] is not None and
                       minimum['t']-2 <= item['t'] <= minimum['t']+2]
    axes[2].plot([item['t'] for item in nearby_commands],
                 [math.hypot(item['vx'], item['vy']) for item in nearby_commands],
                 '.', ms=2, label='final body-frame speed magnitude')
    axes[2].set_ylabel('final speed (m/s)')
    axes[2].set_xlabel('simulation time (s)')
    for axis in axes:
        axis.axvline(minimum['t'], color='#991515', alpha=.5)
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    fig.savefig(OUT/'boundary_7_timeline.png')
    plt.close(fig)

    gap_report = {}
    for topic in ('/gazebo/odometry', '/ground/odometry',
                  '/model/omni_robot/cmd_vel', '/clock'):
        values = gaps[topic]
        positives = [item for item in values if item['receive_gap_s'] >= 0]
        gap_report[topic] = {
            'count': count[topic],
            'max_receive_gap': max(positives, key=lambda item: item['receive_gap_s']),
            'over_0_2_s': [item for item in positives if item['receive_gap_s'] > .2],
            'out_of_order_receive_count': len(values)-len(positives),
        }
    report = {
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'method': 'read-only native messages and captured SDF; figures are visualization',
        'input_sha256': {str(path.relative_to(ROOT)): sha(path)
                         for path in (BASE/f'{NAME}.native.jsonl',
                                      BASE/f'{NAME}.csv', BASE/f'{NAME}.plans.jsonl',
                                      BASE/f'{NAME}.actuation.jsonl', SDF,
                                      OLD/'trial_full_01.csv',
                                      OLD/'trial_full_01.plans.jsonl')},
        'analysis_script_sha256': sha(Path(__file__)),
        'new_native_minimum': minimum,
        'new_native_count': len(active),
        'new_native_rate_hz': (len(active)-1)/(active[-1]['t']-active[0]['t']),
        'old_sampled_minimum': {key: old_min[key] for key in
                                ('sim_t','gt_x','gt_y','gt_yaw','wall_clearance')},
        'old_to_new_min_gap_delta_m': minimum['gap']-old_min['wall_clearance'],
        'new_plan_near_min_xy': new_point,
        'old_plan_near_old_min_xy': old_point,
        'new_plan_reference_y_offset_m': new_point[1]-1.8,
        'new_gt_reference_y_offset_m': minimum['y']-1.8,
        'new_gt_to_setpoint_at_min_xy': (min(nearby_setpoints,
            key=lambda item: abs(item['t']-minimum['t'])) if nearby_setpoints else None),
        'diagnostic_count_near_min': len(diagnostics),
        'receive_gaps': gap_report,
        'continuous_time_clearance_proven': False,
        'statistical_repeatability_proven': False,
        'contact_data': 'CONTACT DATA UNAVAILABLE',
    }
    (OUT/'offline_review.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'native_minimum': minimum,
                      'old_minimum_gap_m': old_min['wall_clearance'],
                      'max_native_gt_receive_gap': gap_report['/gazebo/odometry']['max_receive_gap'],
                      'gt_gaps_over_0_2_s': len(gap_report['/gazebo/odometry']['over_0_2_s'])},
                     indent=2))


if __name__ == '__main__':
    main()
