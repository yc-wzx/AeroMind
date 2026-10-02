#!/usr/bin/env python3
"""Offline comparison around Full's closest approach; never publishes ROS data."""

import csv
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/uav_planning/scripts'))
from provincial_safety_geometry import polygon_distance, rectangle

BASE = ROOT / 'tools/results/provincial_stage15_forward_integration_20260928'
FULL = BASE / 'single_full'
CHAIN = BASE / 'continuous_abc'
OUT = BASE / 'offline_clearance_review_v2'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_rows(path):
    with path.open(newline='') as stream:
        return [{name: float(value) for name, value in row.items()}
                for row in csv.DictReader(stream)]


def load_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def walls_from_sdf(path):
    tree = ET.parse(path)
    walls = []
    for model in tree.findall('.//model'):
        name = model.get('name', '')
        if not name.startswith('boundary_'):
            continue
        pose = [float(value) for value in model.findtext('pose').split()]
        size = [float(value) for value in model.findtext(
            'link/collision/geometry/box/size').split()]
        walls.append({'name': name, 'pose': pose, 'size': size,
                      'polygon': rectangle(pose[0], pose[1], pose[5],
                                           size[0], size[1])})
    return walls


def body_world(vx, vy, yaw):
    return (math.cos(yaw)*vx-math.sin(yaw)*vy,
            math.sin(yaw)*vx+math.cos(yaw)*vy)


def nearest_plan(plans, sim_t, x):
    eligible = [plan for plan in plans if
                plan['marker_stamp_s'] <= sim_t <=
                plan['marker_stamp_s'] +
                plan['sample_interval_s']*(len(plan['points'])-1)]
    if not eligible:
        raise ValueError('no plan active at selected sample')
    plan = max(eligible, key=lambda item: item['marker_stamp_s'])
    nearest = min(plan['points'], key=lambda point: abs(point[0]-x))
    return plan, nearest


def main():
    if OUT.exists():
        raise FileExistsError(OUT)
    full_manifest = json.loads((FULL / 'input_manifest.json').read_text())
    chain_manifest = json.loads((CHAIN / 'input_manifest.json').read_text())
    sdf_relative = 'src/uav_bringup/worlds/provincial_2025_training.sdf'
    sdf = FULL / 'input_snapshot' / sdf_relative
    assert sha(sdf) == full_manifest['sha256'][sdf_relative]
    assert sha(sdf) == chain_manifest['sha256'][sdf_relative]
    walls = walls_from_sdf(sdf)
    f = load_rows(FULL / 'trial_full_01.csv')
    b = load_rows(CHAIN / 'trial_b_chain_01.csv')
    fplan = load_jsonl(FULL / 'trial_full_01.plans.jsonl')
    bplan = load_jsonl(CHAIN / 'trial_b_chain_01.plans.jsonl')
    diag = load_jsonl(FULL / 'trial_full_01.actuation.jsonl')
    events = load_jsonl(FULL / 'trial_full_01.events.jsonl')
    minimum = min(f, key=lambda row: row['wall_clearance'])
    t, x, y, yaw = (minimum[key] for key in
                    ('sim_t', 'gt_x', 'gt_y', 'gt_yaw'))
    body = rectangle(x, y, yaw, .52, .42)
    wall_distances = sorted(
        ((polygon_distance(body, wall['polygon']), wall)
         for wall in walls), key=lambda item: item[0])
    distance, wall = wall_distances[0]
    if abs(distance-minimum['wall_clearance']) > 1e-8:
        raise ValueError('SDF geometry and CSV minimum disagree')
    plan, point = nearest_plan(fplan, t, x)
    bclosest = min(b, key=lambda row: abs(row['gt_x']-x))
    bplan_at_x, bpoint = nearest_plan(bplan,
                                     bclosest['sim_t'], bclosest['gt_x'])
    common_x = (5.3, 5.6)
    fband = [row for row in f if common_x[0] <= row['gt_x'] <= common_x[1]]
    bband = [row for row in b if common_x[0] <= row['gt_x'] <= common_x[1]]
    window = [row for row in f if t-2 <= row['sim_t'] <= t+2]
    diagnostics = [row for row in diag if row.get('gt_stamp_s') is not None
                   and t-2 <= row['gt_stamp_s'] <= t+2]
    sampled_gaps = [after['sim_t']-before['sim_t']
                    for before, after in zip(f, f[1:])]
    around_gaps = [after['sim_t']-before['sim_t']
                   for before, after in zip(window, window[1:])]
    switches = [event for event in events if
                'RMUC waypoint diagnostic completed' in event.get('message', '')]
    last_switch = max((event for event in switches if event['wall_s'] <=
                       minimum['wall_s']), key=lambda item: item['wall_s'])

    # Draw the collision geometry and raw traces in their world coordinates.
    fig, ax = plt.subplots(figsize=(11, 7), dpi=150, constrained_layout=True)
    for item in walls:
        poly = item['polygon']
        if (min(pt[0] for pt in poly) > 6.05 or
                max(pt[0] for pt in poly) < 4.55 or
                min(pt[1] for pt in poly) > 2.32 or
                max(pt[1] for pt in poly) < 1.27):
            continue
        patch = Polygon(poly, closed=True, facecolor='#8a8f98',
                        edgecolor='#4a4f58', alpha=.65)
        ax.add_patch(patch)
        label_x = min(5.95, max(4.65, sum(pt[0] for pt in poly)/4))
        label_y = min(2.28, max(1.33, sum(pt[1] for pt in poly)/4))
        ax.text(label_x, label_y, item['name'], fontsize=8,
                ha='right' if label_x > 5.8 else 'center')
    ax.plot([4.7, 5.95], [1.8, 1.8], '--', color='black',
            lw=1.4, label='Reference y = 1.800 m')
    ax.plot([row['gt_x'] for row in f if 4.7 <= row['gt_x'] <= 6.05],
            [row['gt_y'] for row in f if 4.7 <= row['gt_x'] <= 6.05],
            color='#cb3434', lw=2.2, label='Full GT samples')
    ax.plot([row['gt_x'] for row in b if 4.7 <= row['gt_x'] <= 6.05],
            [row['gt_y'] for row in b if 4.7 <= row['gt_x'] <= 6.05],
            color='#187da0', lw=2.0, label='Segment B GT samples')
    for trace, color, label in ((plan, '#ea8c42', 'Full plan at min'),
                                (bplan_at_x, '#48b1ba', 'B plan near same x')):
        points = [pt for pt in trace['points'] if 4.7 <= pt[0] <= 6.05]
        ax.plot([pt[0] for pt in points], [pt[1] for pt in points],
                color=color, ls=':', lw=2.2, label=label)
    ax.add_patch(Polygon(body, closed=True, fill=False,
                         edgecolor='#a11212', linewidth=2.0))
    ax.scatter([x], [y], color='#a11212', s=50, zorder=5)
    ax.annotate(f'Full min {distance:.4f} m', (x, y),
                xytext=(x+.13, y+.16),
                arrowprops={'arrowstyle': '->', 'color': '#a11212'})
    ax.set(xlim=(4.55, 6.05), ylim=(1.27, 2.32),
           xlabel='world x (m)', ylabel='world y (m)',
           title='Provisional corridor: Full vs segmented B')
    ax.set_aspect('equal')
    ax.grid(alpha=.2)
    ax.legend(loc='lower right', fontsize=8)
    OUT.mkdir(parents=True)
    fig.savefig(OUT / 'full_min_clearance_plan_view.png')
    plt.close(fig)

    times = [row['sim_t'] for row in window]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), dpi=150,
                             sharex=True)
    axes[0].plot(times, [row['gt_y'] for row in window], label='GT y')
    axes[0].plot(times, [row['odom_y'] for row in window], '--', label='odom y')
    axes[0].plot(times, [row['setpoint_y'] for row in window], ':',
                 label='commanded setpoint y')
    axes[0].axhline(1.8, color='black', ls='--', alpha=.5)
    axes[0].set_ylabel('world y (m)')
    axes[0].legend(loc='best', fontsize=8)
    axes[1].plot(times, [row['wall_clearance'] for row in window],
                 label='sampled GT body-wall gap')
    axes[1].plot([row['gt_stamp_s'] for row in diagnostics],
                 [row['predicted_gap_m'] for row in diagnostics], '.', ms=3,
                 label='guard predicted gap')
    axes[1].plot([row['gt_stamp_s'] for row in diagnostics],
                 [row['planned_path_min_gap_m'] for row in diagnostics],
                 '.', ms=3, label='planned path min gap')
    axes[1].axhline(.08, color='#c52a2a', ls='--', label='0.08 m gate')
    axes[1].set_ylabel('gap (m)')
    axes[1].legend(loc='best', fontsize=8)
    world_final = [body_world(row['published_vx'], row['published_vy'],
                              row['gt_yaw']) for row in window]
    world_raw = [body_world(row['cmd_vx'], row['cmd_vy'], row['gt_yaw'])
                 for row in window]
    axes[2].plot(times, [value[0] for value in world_final],
                 label='final body cmd -> world vx')
    axes[2].plot(times, [value[0] for value in world_raw], '--',
                 label='raw body cmd -> world vx')
    gt_vx = [(after['gt_x']-before['gt_x'])/
             (after['sim_t']-before['sim_t'])
             for before, after in zip(window, window[1:])]
    axes[2].plot(times[1:], gt_vx, ':', label='GT finite-difference vx')
    axes[2].set_ylabel('world vx (m/s)')
    axes[2].legend(loc='best', fontsize=8)
    axes[3].plot(times, [row['centerline_cte'] for row in window],
                 label='reference-line error')
    axes[3].plot(times, [row['setpoint_error'] for row in window],
                 label='GT to setpoint distance')
    axes[3].plot(times, [row['odom_x']-row['gt_x'] for row in window],
                 label='odom x - GT x')
    axes[3].set(xlabel='simulation time (s)', ylabel='error (m)')
    axes[3].legend(loc='best', fontsize=8)
    for axis in axes:
        axis.axvline(t, color='#a11212', alpha=.6)
        axis.grid(alpha=.25)
    fig.suptitle(f'Full closest approach, t={t:.2f} s (raw samples)')
    fig.tight_layout()
    fig.savefig(OUT / 'full_min_clearance_timeline.png')
    plt.close(fig)

    report = {
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'method': 'read-only calculations from captured SDF and original CSV/JSONL',
        'input_sha256': {
            str(path.relative_to(ROOT)): sha(path) for path in
            (FULL / 'trial_full_01.csv', FULL / 'trial_full_01.plans.jsonl',
             FULL / 'trial_full_01.actuation.jsonl',
             CHAIN / 'trial_b_chain_01.csv',
             CHAIN / 'trial_b_chain_01.plans.jsonl')},
        'captured_sdf_sha256': sha(sdf),
        'sampled_minimum': {
            'sim_t': t, 'world_xy': [x, y], 'yaw_rad': yaw,
            'body_wall_gap_m': distance,
            'body_polygon': body,
            'nearest_wall': wall,
            'reference_y_m': 1.8,
            'reference_body_to_upper_wall_gap_m': 2.2-(1.8+.26),
            'cross_track_error_m': minimum['centerline_cte'],
            'setpoint_world_xy': [minimum['setpoint_x'],
                                  minimum['setpoint_y']],
            'setpoint_error_m': minimum['setpoint_error'],
            'odom_world_xy': [minimum['odom_x'], minimum['odom_y']],
            'raw_body_cmd': [minimum['cmd_vx'], minimum['cmd_vy'],
                             minimum['cmd_wz']],
            'raw_cmd_world_xy': body_world(minimum['cmd_vx'],
                                            minimum['cmd_vy'], yaw),
            'final_body_cmd': [minimum['published_vx'],
                               minimum['published_vy'],
                               minimum['published_wz']],
            'final_cmd_world_xy': body_world(minimum['published_vx'],
                                              minimum['published_vy'], yaw),
            'active_plan_stamp_s': plan['marker_stamp_s'],
            'active_plan_nearest_world_xy': point,
            'last_waypoint_completion_before_min': last_switch['message'],
        },
        'common_x_band_m': common_x,
        'same_spatial_band': {
            'full': {'samples': len(fband),
                     'max_world_y_m': max(row['gt_y'] for row in fband),
                     'min_body_wall_gap_m': min(row['wall_clearance']
                                                 for row in fband)},
            'segment_b': {'samples': len(bband),
                          'max_world_y_m': max(row['gt_y'] for row in bband),
                          'min_body_wall_gap_m': min(row['wall_clearance']
                                                      for row in bband),
                          'nearest_plan_stamp_s':
                              bplan_at_x['marker_stamp_s'],
                          'nearest_plan_world_xy': bpoint},
        },
        'sampling': {
            'full_gt_max_interval_s': max(sampled_gaps),
            'full_gt_interval_near_min_s': max(around_gaps),
            'window_start_sim_s': t-2,
            'window_end_sim_s': t+2,
            'window_gt_count': len(window),
            'window_guard_diagnostic_count': len(diagnostics),
        },
        'sampled_gap_below_0_08_m': any(
            row['wall_clearance'] < .08 for row in f),
        'continuous_time_lower_bound_verified': False,
        'contact_data': 'CONTACT DATA UNAVAILABLE',
    }
    (OUT / 'full_min_clearance_offline_audit.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'minimum_m': distance,
                      'nearest_wall': wall['name'],
                      'full_band_gap_m':
                          report['same_spatial_band']['full']['min_body_wall_gap_m'],
                      'b_band_gap_m':
                          report['same_spatial_band']['segment_b']['min_body_wall_gap_m'],
                      'sampled_below_gate':
                          report['sampled_gap_below_0_08_m']}))


if __name__ == '__main__':
    main()
