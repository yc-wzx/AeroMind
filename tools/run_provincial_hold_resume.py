#!/usr/bin/env python3
"""One provisional A-lane obstacle hold/release trial with preserved telemetry."""

import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

from run_provincial_safety_fix_matrix import ROOT, existing_servers, wait_for_launch

sys.path.insert(0, str(ROOT / 'src/uav_planning/scripts'))
from grid_route import GridRoute
from provincial_safety_geometry import body_wall_gap, polygon_distance, rectangle


OUT = ROOT / 'tools/results/provincial_stage15_hold_resume_20260928/hold_trial_01'
WORLD = 'provincial_2025_training'
MODEL = 'stage15_hold_obstacle'
START = (4.7, 0.5)
GOAL = (4.7, 1.15)
OBSTACLE = (4.7, 1.15, 0.10, 0.10, 0.60)
TRIAL = 'trial_hold_01'


def sim_stamp(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))


class Observer(Node):
    def __init__(self):
        super().__init__('stage15_hold_observer')
        self.events = []
        self.last_gt = None
        self.last_gt_log = -math.inf
        self.latest_sim = None
        self.create_subscription(Odometry, '/gazebo/odometry', self.gt_cb, 100)
        self.create_subscription(Odometry, '/ground/odometry', self.odom_cb, 100)
        self.create_subscription(Twist, '/cmd_vel', self.raw_cb, 100)
        self.create_subscription(Twist, '/model/omni_robot/cmd_vel',
                                 self.final_cb, 100)
        self.create_subscription(String, '/ground/planning/actuation_diagnostics',
                                 self.diag_cb, 100)
        self.create_subscription(Log, '/rosout', self.log_cb, 100)

    def add(self, kind, **fields):
        self.events.append({'wall_monotonic_s': time.monotonic(),
                            'latest_gt_sim_s': self.latest_sim,
                            'kind': kind, **fields})

    def gt_cb(self, msg):
        self.last_gt = msg
        self.latest_sim = sim_stamp(msg)
        if self.latest_sim - self.last_gt_log >= 0.05:
            p = msg.pose.pose.position
            self.add('gt', x=p.x, y=p.y, yaw=yaw_of(msg.pose.pose.orientation),
                     speed=math.hypot(msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y))
            self.last_gt_log = self.latest_sim

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.add('odom', x=p.x, y=p.y, stamp_s=sim_stamp(msg))

    def raw_cb(self, msg):
        self.add('raw_cmd', vx=msg.linear.x, vy=msg.linear.y,
                 wz=msg.angular.z)

    def final_cb(self, msg):
        self.add('final_cmd', vx=msg.linear.x, vy=msg.linear.y,
                 wz=msg.angular.z)

    def diag_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.add('diagnostic', **payload)

    def log_cb(self, msg):
        if any(key in msg.msg for key in ('RMUC route diagnostic',
                    'RMUC waypoint diagnostic', 'GOAL REJECTED',
                    'All remaining RMUC route stages')):
            self.add('rosout', message=msg.msg)


def obstacle_sdf():
    x, y, sx, sy, sz = OBSTACLE
    return f'''<sdf version="1.8"><model name="{MODEL}">
<static>true</static><pose>{x} {y} {sz/2} 0 0 0</pose>
<link name="link"><collision name="collision"><geometry><box>
<size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size>
</box></geometry><material><ambient>0.9 0.1 0.1 1</ambient>
<diffuse>0.9 0.1 0.1 1</diffuse></material></visual></link>
</model></sdf>'''


def service(path, request_type, request):
    result = subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/{path}',
                             '--reqtype', request_type,
                             '--reptype', 'ignition.msgs.Boolean',
                             '--timeout', '5000', '--req', request],
                            capture_output=True, text=True, timeout=10)
    return {'returncode': result.returncode,
            'stdout': result.stdout, 'stderr': result.stderr,
            'pass': result.returncode == 0 and 'data: true' in result.stdout}


def static_preflight():
    grid = GridRoute(ROOT/'src/uav_bringup/maps/provincial_2025_provisional.pgm',
                     clearance=0.40)
    route = grid.route(START, GOAL)
    world = ET.parse(ROOT/'src/uav_bringup/worlds/provincial_2025_training.sdf')
    walls = []
    for model in world.findall('.//model'):
        if model.get('name', '').startswith('boundary_'):
            pose = [float(value) for value in model.findtext('pose').split()]
            size = [float(value) for value in model.findtext(
                'link/collision/geometry/box/size').split()]
            walls.append(rectangle(pose[0], pose[1], pose[5], size[0], size[1]))
    gap = min(body_wall_gap(START[0], START[1]+index*0.01,
                            math.pi/2, walls) for index in range(66))
    obstacle = rectangle(OBSTACLE[0], OBSTACLE[1], 0, OBSTACLE[2], OBSTACLE[3])
    obstacle_wall_gap = min(polygon_distance(obstacle, wall) for wall in walls)
    initial_robot_gap = polygon_distance(
        rectangle(*START, math.pi/2, 0.52, 0.42), obstacle)
    result = {'grid_route': route, 'grid_min_clearance_m':
              grid.line_min_clearance(START, GOAL),
              'sampled_body_wall_gap_m': gap,
              'obstacle_wall_gap_m': obstacle_wall_gap,
              'initial_robot_obstacle_gap_m': initial_robot_gap,
              'obstacle_xyz_size_m': OBSTACLE}
    if gap < 0.08 or obstacle_wall_gap < 0.08 or initial_robot_gap < 0.15:
        raise ValueError(f'unsafe hold trial preflight: {result}')
    return result, walls, obstacle


def wait_until(predicate, seconds, runner=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        if runner is not None and runner.poll() is not None:
            return None
        time.sleep(0.04)
    return None


def main(*, on_ready=None, raw_every_message=False):
    if OUT.exists():
        raise FileExistsError(OUT)
    if existing_servers():
        raise RuntimeError('another provisional Gazebo server is active')
    preflight, walls, obstacle = static_preflight()
    OUT.mkdir(parents=True)
    (OUT/'offline_preflight.json').write_text(json.dumps(preflight, indent=2)+'\n')
    (OUT/'obstacle.sdf').write_text(obstacle_sdf())
    inputs = ('tools/run_provincial_hold_resume.py',
              'tools/run_provincial_low_speed_validation.py',
              'tools/provincial_terminal_gate.py',
              'tools/summarize_provincial_stage15_terminal_v3.py',
              'src/uav_planning/scripts/gazebo_navigation_interface.py',
              'install/uav_planning/lib/uav_planning/gazebo_navigation_interface.py',
              'src/uav_bringup/worlds/provincial_2025_training.sdf',
              'src/uav_bringup/maps/provincial_2025_provisional.pgm')
    (OUT/'input_manifest.json').write_text(json.dumps({
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'sha256': {path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
                   for path in inputs}}, indent=2)+'\n')
    log = (OUT/'launch.log').open('x')
    launch = subprocess.Popen(['ros2', 'launch', 'uav_bringup',
        'provincial_2025_provisional.launch.py', 'gui:=false',
        'rviz:=false', 'auto_goal:=false'], stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    observer = None
    runner = None
    obstacle_created = False
    report = {'classification': 'TRAINING-ONLY / PROVISIONAL',
              'obstacle_created': False, 'obstacle_removed': False,
              'hold_state_exercised': False, 'pass': False}
    try:
        wait_for_launch()
        if on_ready is not None:
            on_ready()
        rclpy.init()
        observer = Observer()
        thread = threading.Thread(target=rclpy.spin, args=(observer,), daemon=True)
        thread.start()
        wait_until(lambda: observer.last_gt is not None, 5)
        runner_cmd = [sys.executable, str(ROOT/'tools/run_provincial_low_speed_validation.py'),
                      '--route', 'start', '--name', TRIAL,
                      '--yaw-mode', 'hold-start', '--terminal-validation',
                      '--timeout', '90', '--output-dir', str(OUT)]
        if raw_every_message:
            runner_cmd.append('--raw-every-message')
        runner_log = (OUT/f'{TRIAL}.runner.log').open('x')
        runner = subprocess.Popen(runner_cmd, stdout=runner_log,
                                  stderr=subprocess.STDOUT)
        accepted = wait_until(lambda: next((event for event in observer.events
                    if event['kind'] == 'rosout' and
                    'RMUC route diagnostic' in event['message']), None),
                    10, runner)
        if accepted is None:
            report['status'] = 'GOAL_NOT_ACCEPTED'
            return 2
        report['accepted_event'] = accepted
        gt = observer.last_gt
        p = gt.pose.pose.position
        gap = polygon_distance(rectangle(p.x, p.y,
                    yaw_of(gt.pose.pose.orientation), 0.52, 0.42), obstacle)
        report['robot_obstacle_gap_before_insert_m'] = gap
        if gap < 0.15:
            report['status'] = 'ABORT_UNSAFE_OBSTACLE_INSERTION'
            return 2
        create = service('create', 'ignition.msgs.EntityFactory',
                         f'sdf_filename: "{OUT/"obstacle.sdf"}"')
        report['create_reply'] = create
        if not create['pass']:
            report['status'] = 'OBSTACLE_CREATE_FAILED'
            return 2
        obstacle_created = True
        report['obstacle_created'] = True
        report['obstacle_insert_wall_s'] = time.monotonic()
        report['obstacle_insert_gt_sim_s'] = observer.latest_sim
        hold = wait_until(lambda: next((event for event in observer.events
                  if event['kind'] == 'diagnostic' and
                  event.get('navigation_state') == 'waiting_for_clear_waypoint'
                  and event.get('pending_waypoints', 0) > 0), None), 12, runner)
        if hold is None:
            report['status'] = 'NOT_EXERCISED'
            return 2
        report['hold_state_exercised'] = True
        report['hold_event'] = hold
        held_until = hold['latest_gt_sim_s'] + 3.0
        while observer.latest_sim is None or observer.latest_sim < held_until:
            if runner.poll() is not None:
                report['status'] = 'RUNNER_EXITED_DURING_HOLD'
                return 2
            gt = observer.last_gt
            p = gt.pose.pose.position
            current_gap = polygon_distance(rectangle(p.x, p.y,
                yaw_of(gt.pose.pose.orientation), 0.52, 0.42), obstacle)
            if current_gap < 0.02:
                report['status'] = 'UNSAFE_OBSTACLE_PROXIMITY'
                report['minimum_obstacle_gap_before_abort_m'] = current_gap
                return 2
            new_final = [event for event in observer.events
                         if event['kind'] == 'final_cmd' and
                         event['wall_monotonic_s'] >= hold['wall_monotonic_s']+0.10]
            if new_final and math.hypot(new_final[-1]['vx'],
                                        new_final[-1]['vy']) > 0.02:
                report['status'] = 'UNSAFE_NONZERO_FINAL_OUTPUT'
                return 2
            time.sleep(0.04)
        report['hold_end_gt_sim_s'] = observer.latest_sim
        remove = service('remove', 'ignition.msgs.Entity',
                         f'name: "{MODEL}" type: MODEL')
        report['remove_reply'] = remove
        if not remove['pass']:
            report['status'] = 'OBSTACLE_REMOVE_FAILED'
            return 2
        obstacle_created = False
        report['obstacle_removed'] = True
        report['obstacle_remove_wall_s'] = time.monotonic()
        report['obstacle_remove_gt_sim_s'] = observer.latest_sim
        try:
            report['runner_returncode'] = runner.wait(timeout=95)
        except subprocess.TimeoutExpired:
            report['status'] = 'RUNNER_TIMEOUT'
            return 2
        report['status'] = 'RECOVERY_COMPLETED' if runner.returncode == 0 else 'RUNNER_FAILED'
        report['pass'] = (runner.returncode == 0 and
                          report['hold_state_exercised'] and
                          report['obstacle_removed'])
        return 0 if report['pass'] else 2
    finally:
        if runner is not None and runner.poll() is None:
            runner.send_signal(signal.SIGINT)
            try:
                runner.wait(timeout=5)
            except subprocess.TimeoutExpired:
                runner.terminate()
                runner.wait(timeout=5)
        if obstacle_created:
            report['cleanup_remove_reply'] = service(
                'remove', 'ignition.msgs.Entity', f'name: "{MODEL}" type: MODEL')
        if observer is not None:
            with (OUT/'observer_events.jsonl').open('x') as stream:
                for event in observer.events:
                    stream.write(json.dumps(event)+'\n')
            observer.destroy_node()
            rclpy.shutdown()
            thread.join(timeout=3)
        try:
            os.killpg(launch.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            launch.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGTERM)
            launch.wait(timeout=8)
        log.close()
        if runner is not None:
            runner_log.close()
        report['gazebo_server_survived_cleanup'] = bool(existing_servers())
        (OUT/'hold_trial_summary.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
