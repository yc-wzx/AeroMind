#!/usr/bin/env python3
"""Bounded one-world forward training trials; evaluator publishes goals only."""

import argparse
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from run_provincial_safety_fix_matrix import ROOT, existing_servers, wait_for_launch
from summarize_provincial_stage15_terminal_v3 import audit_run, sdf_walls

sys.path.insert(0, str(ROOT / 'src/uav_planning/scripts'))
from grid_route import GridRoute
from provincial_safety_geometry import body_wall_gap, reference_stages


BASE = ROOT / 'tools/results/provincial_stage15_forward_integration_20260928'
FIELD = ROOT / 'src/uav_planning/config/provincial_2025_provisional.json'
MAP = ROOT / 'src/uav_bringup/maps/provincial_2025_provisional.pgm'
INPUTS = (
    'tools/run_provincial_forward_integration.py',
    'tools/summarize_provincial_forward_integration.py',
    'tools/run_provincial_low_speed_validation.py',
    'tools/summarize_provincial_stage15_terminal_v3.py',
    'tools/summarize_provincial_stage15_terminal_v2.py',
    'tools/provincial_terminal_gate.py',
    'src/uav_planning/scripts/gazebo_navigation_interface.py',
    'src/uav_planning/scripts/ego_trajectory_executor.py',
    'src/uav_planning/scripts/grid_route.py',
    'src/uav_planning/scripts/provincial_safety_geometry.py',
    'src/uav_planning/config/provincial_2025_provisional.json',
    'src/uav_bringup/launch/provincial_2025_provisional.launch.py',
    'src/uav_bringup/config/ego_provincial_2025_provisional.yaml',
    'src/uav_bringup/maps/provincial_2025_provisional.pgm',
    'src/uav_bringup/maps/provincial_2025_provisional.yaml',
    'src/uav_bringup/worlds/provincial_2025_training.sdf',
    'install/uav_planning/lib/uav_planning/gazebo_navigation_interface.py',
    'install/ego_planner/lib/ego_planner/ego_planner_node',
)
STAGES = (
    ('trial_a_chain_01', 'start', (4.7, 0.5), (4.7, 1.15), 90),
    ('trial_b_chain_01', 'turn', (4.7, 1.15), (5.8, 1.8), 120),
    ('trial_c_chain_01', 'shoot', (5.8, 1.8), (8.7, 4.25), 160),
)
FULL = ('trial_full_01', 'full', (4.7, 0.5), (8.7, 4.25), 240)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stamp(st):
    return st.sec + st.nanosec * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))


class PoseObserver(Node):
    def __init__(self):
        super().__init__('stage15_forward_pose_observer')
        self.message = None
        self.received_wall = None
        self.create_subscription(Odometry, '/gazebo/odometry', self.on_gt, 100)

    def on_gt(self, message):
        self.message = message
        self.received_wall = time.monotonic()

    def fresh_pose(self, after_sim=None, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.message is None or self.received_wall is None or
                    time.monotonic()-self.received_wall > 0.2 or
                    (after_sim is not None and stamp(self.message.header.stamp)
                     <= after_sim)):
                continue
            pose = self.message.pose.pose
            result = [pose.position.x, pose.position.y, yaw(pose.orientation),
                      stamp(self.message.header.stamp)]
            if all(math.isfinite(value) for value in result):
                return result
        raise RuntimeError('fresh Gazebo pose not available for preflight')


def offline_leg(start, goal, grid, walls, reference):
    grid.route(start, goal)
    stages = reference_stages(start, goal, reference)
    points = [tuple(start)] + [tuple(stage) for stage in stages]
    if not all(grid.line_safe(a, b, clearance=0.40)
               for a, b in zip(points, points[1:])):
        raise ValueError(f'unsafe reference leg: {points}')
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    samples = [(a[0]+(b[0]-a[0])*index/max(1, math.ceil(length/0.025)),
                a[1]+(b[1]-a[1])*index/max(1, math.ceil(length/0.025)))
               for (a, b), length in zip(zip(points, points[1:]), lengths)
               for index in range(max(1, math.ceil(length/0.025))+1)]
    gaps = [body_wall_gap(x, y, math.pi/2, walls) for x, y in samples]
    minimum = min(gaps)
    if minimum < 0.08:
        raise ValueError(f'90-degree footprint violates static gap: {minimum}')
    return {'start': list(start), 'goal': list(goal), 'reference_stages': stages,
            'length_m': sum(lengths),
            'min_grid_clearance_m': min(grid.line_min_clearance(a, b)
                                        for a, b in zip(points, points[1:])),
            'min_sampled_body_wall_gap_m': minimum,
            'sample_step_m': 0.025}


def snapshot_inputs(out, mode):
    if sha(ROOT/'src/uav_planning/scripts/gazebo_navigation_interface.py') != sha(
            ROOT/'install/uav_planning/lib/uav_planning/gazebo_navigation_interface.py'):
        raise RuntimeError('navigation source and installed entry point differ')
    snapshot = out/'input_snapshot'
    hashes = {}
    for relative in INPUTS:
        source = ROOT/relative
        hashes[relative] = sha(source)
        if not relative.startswith('install/'):
            target = snapshot/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    manifest = {
        'classification': 'TRAINING-ONLY / PROVISIONAL', 'mode': mode,
        'git_head': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'sha256': hashes,
        'source_install_navigation_same_content': True,
    }
    (out/'input_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (out/'git_status_before.txt').write_text(subprocess.check_output(
        ['git', 'status', '--short'], cwd=ROOT, text=True))


def server_processes():
    listing = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
    return [line.strip() for line in listing.splitlines()
            if 'ign gazebo -s -r' in line and
            'provincial_2025_training.sdf' in line and
            'ps -eo' not in line]


def runner_one(out, trial, node, grid, walls, reference, after_sim):
    name, route, nominal_start, goal, timeout = trial
    actual = node.fresh_pose(after_sim=after_sim)
    if math.dist(actual[:2], nominal_start) > 0.20:
        raise ValueError(f'{name} actual start differs from intended stage: {actual}')
    if abs(math.atan2(math.sin(actual[2]-math.pi/2),
                      math.cos(actual[2]-math.pi/2))) > 0.05:
        raise ValueError(f'{name} initial yaw differs from 90 degrees: {actual}')
    preflight = offline_leg(actual[:2], goal, grid, walls, reference)
    (out/f'{name}.offline_preflight.json').write_text(
        json.dumps({'measured_start_gt': actual, **preflight}, indent=2)+'\n')
    cmd = [sys.executable, str(ROOT/'tools/run_provincial_low_speed_validation.py'),
           '--route', route, '--name', name, '--yaw-mode', 'hold-start',
           '--terminal-validation', '--timeout', str(timeout),
           '--output-dir', str(out)]
    with (out/f'{name}.runner.log').open('x') as logfile:
        try:
            completed = subprocess.run(cmd, stdout=logfile,
                                       stderr=subprocess.STDOUT,
                                       timeout=timeout+25)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(f'{name} runner process timeout') from error
    summary_path = out/f'{name}.summary.json'
    if not summary_path.exists():
        raise RuntimeError(f'{name} did not produce a summary')
    summary = json.loads(summary_path.read_text())
    audit = audit_run(out, name, walls)
    (out/f'{name}.stage_audit.json').write_text(json.dumps(
        audit, ensure_ascii=False, indent=2)+'\n')
    result = {'name': name, 'route': route, 'runner_exit_code': completed.returncode,
              'audit_pass': audit['pass'], 'initial_gt_pose': summary['initial_gt_pose'],
              'final_gt_pose': [*summary['final_gt_xy'], summary['final_gt_yaw_rad']],
              'goal': list(goal), 'gt_goal_error_m': summary['gt_goal_error_m'],
              'odom_goal_error_m': summary['odom_goal_error_m'],
              'completion_id': (summary.get('navigation_completion') or {}).get('waypoint_id'),
              'final_waypoint_id': summary['final_waypoint_id'],
              'final_gt_sim_s': summary['terminal_snapshot']['gt_stamp_s'],
              'duration_sim_s': summary['duration_sim_s'],
              'minimum_body_wall_gap_m': summary['minimum_physical_wall_clearance_m'],
              'retry_count': summary['retry_count'],
              'stuck_count': summary['stuck_count']}
    if completed.returncode or not audit['pass']:
        raise RuntimeError(f'{name} failed terminal/evidence gate: {result}')
    return result


def raw_hashes(out):
    files = sorted(set(out.glob('*.csv')) |
                   set(out.glob('*.jsonl')) |
                   set(out.glob('*.runner.log')) |
                   set(out.glob('*.summary.json')) |
                   {out/'launch.log', out/'input_manifest.json'})
    return {path.name: sha(path) for path in files if path.exists()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('chain', 'full'), required=True)
    args = parser.parse_args()
    out = BASE / ('continuous_abc' if args.mode == 'chain' else 'single_full')
    if out.exists():
        raise FileExistsError(out)
    if existing_servers():
        raise RuntimeError('another provincial Gazebo server is active')
    if args.mode == 'full':
        gate = json.loads((BASE/'continuous_abc/mission_verification.json').read_text())
        if not gate.get('all_pass'):
            raise RuntimeError('A-B-C integration gate has not passed')
    grid = GridRoute(MAP, clearance=0.40)
    walls = sdf_walls()
    reference = json.loads(FIELD.read_text())['reference_route']
    nominal_preflight = {
        trial[0]: offline_leg(trial[2], trial[3], grid, walls, reference)
        for trial in (STAGES if args.mode == 'chain' else (FULL,))}
    out.mkdir(parents=True)
    (out/'nominal_offline_preflight.json').write_text(
        json.dumps(nominal_preflight, indent=2)+'\n')
    snapshot_inputs(out, args.mode)
    launch_log = (out/'launch.log').open('x')
    launch = subprocess.Popen(['ros2', 'launch', 'uav_bringup',
        'provincial_2025_provisional.launch.py', 'gui:=false',
        'rviz:=false', 'auto_goal:=false'],
        stdout=launch_log, stderr=subprocess.STDOUT,
        start_new_session=True)
    node = None
    progress = {'classification': 'TRAINING-ONLY / PROVISIONAL',
                'mode': args.mode, 'launch_pid': launch.pid, 'runs': [],
                'status': 'RUNNING'}
    failure = None
    try:
        wait_for_launch()
        progress['gazebo_server_processes'] = server_processes()
        if len(progress['gazebo_server_processes']) != 1:
            raise RuntimeError('expected exactly one provincial Gazebo server')
        rclpy.init()
        node = PoseObserver()
        after = None
        for trial in (STAGES if args.mode == 'chain' else (FULL,)):
            result = runner_one(out, trial, node, grid, walls, reference, after)
            progress['runs'].append(result)
            after = result['final_gt_sim_s']
            (out/'progress.json').write_text(json.dumps(progress, indent=2)+'\n')
            print(json.dumps(result), flush=True)
        progress['status'] = 'RUNNERS_COMPLETED_PENDING_INDEPENDENT_AUDIT'
    except Exception as error:
        progress['status'] = 'FAILED_PRESERVED'
        progress['failure'] = f'{type(error).__name__}: {error}'
        failure = error
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        try:
            os.killpg(launch.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            launch.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGTERM)
            launch.wait(timeout=8)
        launch_log.close()
        progress['remaining_gazebo_servers'] = server_processes()
        (out/'progress.json').write_text(json.dumps(progress, indent=2)+'\n')
        (out/'raw_data_sha256.json').write_text(json.dumps(
            raw_hashes(out), indent=2)+'\n')
    print(json.dumps({'mode': args.mode, 'status': progress['status'],
                      'runs': [item['name'] for item in progress['runs']],
                      'failure': progress.get('failure')}), flush=True)
    if failure or progress['remaining_gazebo_servers']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
