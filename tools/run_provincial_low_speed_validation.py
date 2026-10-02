#!/usr/bin/env python3
"""Run one authorized low-speed provincial benchmark and record Gazebo truth.

Only /goal_pose is published. Truth, odometry, setpoints and ROS logs are
subscribed for evaluation; no evaluation signal feeds back into navigation.
"""
import argparse
import csv
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_check_compatible, QoSCompatibility
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/uav_planning/scripts"))
from grid_route import GridRoute
from ego_trajectory_executor import Spline
from provincial_terminal_gate import TerminalGate, terminal_evidence_pass
from provincial_native_trace import NativeTrace

MAP = ROOT / "src/uav_bringup/maps/provincial_2025_provisional.pgm"
FIELD = ROOT / "src/uav_planning/config/provincial_2025_provisional.json"
OUT = ROOT / "tools/results/provincial_low_speed"
ROUTES = {
    "start": ([(4.7, 0.5), (4.7, 1.15)], (4.7, 0.5), (4.7, 1.15)),
    "turn": ([(4.7, 1.15), (4.7, 1.8), (5.8, 1.8)], (4.7, 1.15), (5.8, 1.8)),
    "shoot": ([(5.8, 1.8), (8.7, 1.8), (8.7, 4.25)], (5.8, 1.8), (8.7, 4.25)),
    "full": ([(4.7, 0.5), (4.7, 1.8), (8.7, 1.8), (8.7, 4.25)], (4.7, 0.5), (8.7, 4.25)),
    "return": ([(8.7, 4.25), (8.7, 1.8), (4.7, 1.8), (4.7, 0.5)], (8.7, 4.25), (4.7, 0.5)),
}


def stamp(st):
    return st.sec + st.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def polygon(cx, cy, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * x - s * y, cy + s * x + c * y)
            for x, y in ((-length/2, -width/2), (length/2, -width/2),
                         (length/2, width/2), (-length/2, width/2))]


def wall_polygon(segment, thickness=0.055):
    x0, y0, x1, y1 = segment
    return polygon((x0+x1)/2, (y0+y1)/2,
                   math.atan2(y1-y0, x1-x0), math.hypot(x1-x0, y1-y0), thickness)


def point_segment_distance(p, a, b):
    vx, vy = b[0]-a[0], b[1]-a[1]
    t = max(0.0, min(1.0, ((p[0]-a[0])*vx + (p[1]-a[1])*vy)/(vx*vx+vy*vy)))
    return math.hypot(p[0]-a[0]-t*vx, p[1]-a[1]-t*vy)


def polygon_distance(a, b):
    # Separating-axis test detects overlap (including rotated rectangles).
    for poly in (a, b):
        for p, q in zip(poly, poly[1:] + poly[:1]):
            axis = (-(q[1]-p[1]), q[0]-p[0])
            pa = [x*axis[0]+y*axis[1] for x, y in a]
            pb = [x*axis[0]+y*axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                break
        else:
            continue
        break
    else:
        return 0.0
    return min(point_segment_distance(p, u, v)
               for first, second in ((a, b), (b, a))
               for p in first for u, v in zip(second, second[1:] + second[:1]))


def centerline_error(x, y, points):
    best = None
    for a, b in zip(points, points[1:]):
        vx, vy = b[0]-a[0], b[1]-a[1]
        t = max(0.0, min(1.0, ((x-a[0])*vx + (y-a[1])*vy)/(vx*vx+vy*vy)))
        error = math.hypot(x-a[0]-t*vx, y-a[1]-t*vy)
        if best is None or error < best[0]:
            best = error, math.atan2(vy, vx)
    return best


class Runner(Node):
    def __init__(self, route, name, timeout, yaw_mode='zero', output_dir=OUT,
                 terminal_validation=False, start_override=None, goal_override=None,
                 raw_every_message=False):
        super().__init__("provincial_low_speed_validation")
        self.route, self.name, self.timeout = route, name, timeout
        self.yaw_mode, self.output_dir = yaw_mode, Path(output_dir)
        self.native_trace = None
        if raw_every_message:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.native_trace = NativeTrace(self.output_dir / (name + '.native.jsonl'))
        self.terminal_validation = terminal_validation
        self.polyline, self.nominal_start, self.goal = ROUTES[route]
        if start_override is not None or goal_override is not None:
            self.nominal_start = tuple(start_override or self.nominal_start)
            self.goal = tuple(goal_override or self.goal)
            self.polyline = [self.nominal_start, self.goal]
        self.walls = [wall_polygon(s) for s in json.loads(FIELD.read_text())["collision_segments"]]
        self.grid = GridRoute(MAP, clearance=0.40)
        self.gt = self.odom = self.setpoint = self.command = None
        self.setpoint_stamp = -math.inf
        self.local_plan_endpoint = None
        self.local_plan_last_control_point = None
        self.active_goal = None
        self.last_actuation = None
        self.last_actuation_received_steady = None
        self.actuation_trace = []
        self.final_command = None
        self.final_command_received_steady = None
        self.final_command_trace = []
        self.terminal_gate = TerminalGate()
        self.actuation_counts = {}
        self.actuation_source_counts = {}
        self.initial_gt_pose = None
        self.final_waypoint_id = None
        self.completion = None
        self.false_success = False
        self.post_completion_departure = False
        self.post_completion_max_error = 0.0
        self.post_completion_max_speed = 0.0
        self.arrival = None
        self.stop_start_sim = self.observe_start_sim = None
        self.stop_anchor = None
        self.gt_received_steady = self.odom_received_steady = None
        self.cmd_received_steady = None
        self.last_checked_gt_sim = None
        self.gt_sequence = 0
        self.last_checked_gt_sequence = -1
        self.terminal_invalid_reasons = []
        self.sent_goal_yaw = None
        self.samples, self.events = [], []
        self.plans = []
        self.last_sample = -math.inf
        self.active = self.accepted = False
        self.receiver_stable_since = None
        self.receiver_signature = None
        self.seen_route_ids = set()
        self.acceptance_timeout_s = 5.0
        self.goal_delivery = {'publish_count': 0, 'state': 'not_sent',
                              'readiness': None, 'acceptance': None,
                              'rejection': None, 'ignored_events': []}
        self.start_wall = self.start_sim = None
        self.retry = self.stuck = self.ego_warnings = self.overlap = 0
        self.last_progress_error = math.inf
        self.last_progress_sim = None
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        def subscribe(message_type, topic, callback, qos):
            def measured(message):
                if self.native_trace:
                    with self.native_trace.observe('callback:' + topic):
                        callback(message)
                else:
                    callback(message)
            return self.create_subscription(message_type, topic, measured, qos)
        if self.native_trace:
            subscribe(Clock, "/clock", self.clock_cb, 100)
            subscribe(PoseStamped, "/goal_pose", self.external_goal_cb, 100)
        subscribe(Odometry, "/gazebo/odometry", self.gt_cb, 100)
        subscribe(Odometry, "/ground/odometry", self.odom_cb, 100)
        subscribe(PoseStamped, "/ground/planning/commanded_setpoint", self.setpoint_cb, 100)
        subscribe(Twist, "/cmd_vel", self.cmd_cb, 100)
        subscribe(Twist, "/model/omni_robot/cmd_vel",
                                 self.final_cmd_cb, 100)
        subscribe(Bspline, "/planning/bspline", self.bspline_cb, 100)
        subscribe(Marker, "/ground/planning/planned_trajectory",
                                 self.planned_cb, 100)
        subscribe(PoseStamped, "/ground/planning/goal", self.goal_cb, 100)
        subscribe(String, "/ground/planning/actuation_diagnostics",
                                 self.actuation_cb, 100)
        subscribe(Log, "/rosout", self.log_cb, 100)

    def observed_node_names(self):
        if self.native_trace:
            with self.native_trace.observe('ros_graph_query'):
                return self.get_node_names()
        return self.get_node_names()

    def trace(self, topic, msg):
        if self.native_trace:
            self.native_trace.record(topic, msg)

    def clock_cb(self, msg):
        self.trace('/clock', msg)

    def external_goal_cb(self, msg):
        self.trace('/goal_pose', msg)

    def odom_cb(self, msg):
        self.trace('/ground/odometry', msg)
        self.odom = msg
        self.odom_received_steady = time.monotonic()

    def setpoint_cb(self, msg):
        self.trace('/ground/planning/commanded_setpoint', msg)
        self.setpoint = msg
        self.setpoint_stamp = stamp(msg.header.stamp)

    def cmd_cb(self, msg):
        self.trace('/cmd_vel', msg)
        self.command = msg
        self.cmd_received_steady = time.monotonic()

    def final_cmd_cb(self, msg):
        self.trace('/model/omni_robot/cmd_vel', msg)
        self.final_command = msg
        self.final_command_received_steady = time.monotonic()
        if self.active:
            self.final_command_trace.append({
                'wall_s': self.final_command_received_steady-self.start_wall,
                'gt_stamp_s': stamp(self.gt.header.stamp) if self.gt else None,
                'vx': msg.linear.x, 'vy': msg.linear.y, 'wz': msg.angular.z,
            })

    def bspline_cb(self, msg):
        self.trace('/planning/bspline', msg)
        if msg.pos_pts:
            last = msg.pos_pts[-1]
            self.local_plan_last_control_point = (last.x, last.y)
            try:
                spline = Spline([(p.x, p.y) for p in msg.pos_pts],
                                int(msg.order), list(msg.knots))
                self.local_plan_endpoint = spline.evaluate(spline.end)
            except (ValueError, IndexError, TypeError):
                self.local_plan_endpoint = None

    def planned_cb(self, msg):
        self.trace('/ground/planning/planned_trajectory', msg)
        if not self.active or not msg.points:
            return
        pose = self.gt.pose.pose if self.gt else None
        yaw = yaw_of(pose.orientation) if pose else 0.0
        self.plans.append({
            "wall_s": round(time.monotonic()-self.start_wall, 3),
            "marker_stamp_s": stamp(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "gt_pose": [pose.position.x, pose.position.y, yaw] if pose else None,
            "active_goal": self.active_goal,
            "sample_interval_s": 0.05,
            "points": [[point.x, point.y] for point in msg.points],
        })

    def actuation_cb(self, msg):
        self.trace('/ground/planning/actuation_diagnostics', msg)
        if not self.active:
            return
        try:
            diagnostic = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.last_actuation = diagnostic
        self.last_actuation_received_steady = time.monotonic()
        self.actuation_trace.append({
            'wall_s': self.last_actuation_received_steady-self.start_wall,
            'gt_stamp_s': stamp(self.gt.header.stamp) if self.gt else None,
            **diagnostic,
        })
        state = diagnostic.get('action', 'unknown')
        self.actuation_counts[state] = self.actuation_counts.get(state, 0) + 1
        source = diagnostic.get('source', 'unknown')
        self.actuation_source_counts[source] = self.actuation_source_counts.get(source, 0) + 1
        if state not in ('allowed', 'disabled', 'stale_stop'):
            self.events.append({'wall_s': round(time.monotonic()-self.start_wall,3),
                                'message':'ACTUATION SAFETY '+state,
                                'actuation':diagnostic})

    def goal_cb(self, msg):
        self.trace('/ground/planning/goal', msg)
        self.active_goal = (msg.pose.position.x, msg.pose.position.y,
                            yaw_of(msg.pose.orientation))

    def log_cb(self, msg):
        content = msg.msg
        if not self.active:
            route = re.search(r'RMUC route diagnostic route=(R\d+)', content)
            if route and msg.name == 'gazebo_navigation_interface':
                self.seen_route_ids.add(route.group(1))
            return
        keys = ("RMUC route diagnostic", "RMUC waypoint diagnostic completed",
                "Navigation stage:", "Retrying RMUC stage",
                "GOAL REJECTED", "Goal requires Gazebo odometry",
                "terminal point of the current trajectory is in obstacle",
                "First 3 control points in obstacles", "A* error", "Emergency stop",
                "EGO_DIAG", "stage made no progress", "stage exhausted its retries")
        if not any(key in content for key in keys):
            return
        self.trace('/rosout/relevant', msg)
        pose = self.gt.pose.pose if self.gt else None
        self.events.append({"wall_s": round(time.monotonic()-self.start_wall, 3),
                            "logger": msg.name, "severity": msg.level, "message": content,
                            "gt_pose": [pose.position.x,pose.position.y,yaw_of(pose.orientation)] if pose else None,
                            "active_goal": self.active_goal,
                            "published_bspline_endpoint": self.local_plan_endpoint,
                            "bspline_last_control_point": self.local_plan_last_control_point})
        if "RMUC route diagnostic" in content:
            self.accept_route_event(msg)
        if (not self.accepted and
                ('GOAL REJECTED' in content or 'Goal requires Gazebo odometry' in content)
                and self.fresh_navigation_event(msg)):
            self.goal_delivery['rejection'] = content
            self.goal_delivery['state'] = 'rejected'
        if "RMUC waypoint diagnostic completed" in content:
            match = re.search(r'waypoint=(R\d+:W\d+)', content)
            if match and match.group(1) == self.final_waypoint_id:
                now = time.monotonic()
                gt_fresh = (self.gt is not None and self.gt_received_steady is not None
                            and now-self.gt_received_steady <= 0.2)
                pose = self.gt.pose.pose.position if gt_fresh else None
                error = math.hypot(pose.x-self.goal[0], pose.y-self.goal[1]) if pose else None
                self.completion = {
                    'waypoint_id': match.group(1), 'message': content,
                    'wall_elapsed_s': now-self.start_wall,
                    'gt_stamp_s': stamp(self.gt.header.stamp) if gt_fresh else None,
                    'gt_goal_error_m': error,
                    'gt_fresh': gt_fresh,
                }
                if error is not None and error > 0.10:
                    self.false_success = True
        if "Retrying RMUC stage" in content:
            self.retry += 1
        if any(key in content for key in ("obstacle", "A* error", "Emergency stop", "EGO_DIAG")):
            self.ego_warnings += 1

    def fresh_navigation_event(self, msg):
        sent_ns = self.goal_delivery.get('publish_wall_ns')
        event_ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        # rcutils /rosout stamps use system time in this ROS Humble setup.
        # An unrecognised time basis is incomplete evidence, never an ACK.
        return (msg.name == 'gazebo_navigation_interface' and sent_ns is not None
                and sent_ns <= event_ns <= time.time_ns() + 100_000_000)

    def accept_route_event(self, msg):
        match = re.fullmatch(r'RMUC route diagnostic route=(R\d+) planned=(.+)', msg.msg)
        reason = None
        if not self.fresh_navigation_event(msg):
            reason = 'wrong_logger_or_stale_or_invalid_log_time'
        elif not self.accepted and time.monotonic()-self.start_wall > self.acceptance_timeout_s:
            reason = 'route_acceptance_after_deadline'
        elif not match:
            reason = 'malformed_route_acceptance'
        else:
            route, payload = match.groups()
            items = re.findall(r'(R\d+:W\d+)=\((-?\d+\.\d+),(-?\d+\.\d+)\)', payload)
            if (not items or ';'.join(f'{key}=({x},{y})' for key,x,y in items) != payload
                    or any(key != f'{route}:W{index:02d}'
                           for index,(key,_,_) in enumerate(items))):
                reason = 'malformed_or_cross_route_waypoint_list'
            elif route in self.seen_route_ids:
                reason = 'route_seen_before_this_goal'
            elif math.dist((float(items[-1][1]), float(items[-1][2])), self.goal) > .002:
                reason = 'route_final_goal_mismatch'
            elif self.goal_delivery['acceptance'] is not None:
                if self.goal_delivery['acceptance']['message'] == msg.msg:
                    return  # identical delivery does not become another task
                reason = 'multiple_route_acceptances'
            if reason is None:
                self.accepted = True
                self.final_waypoint_id = items[-1][0]
                self.goal_delivery['state'] = 'accepted'
                self.goal_delivery['acceptance'] = {
                    'route_id': route, 'final_waypoint_id': items[-1][0],
                    'message': msg.msg,
                    'receive_monotonic_ns': time.monotonic_ns(),
                    'log_stamp_ns': msg.stamp.sec*1_000_000_000+msg.stamp.nanosec,
                    'wall_elapsed_s': time.monotonic()-self.start_wall,
                }
        if reason:
            self.goal_delivery['ignored_events'].append({'reason': reason,
                                                         'message': msg.msg})

    def goal_receiver_ready(self, now):
        endpoints = self.get_subscriptions_info_by_topic('/goal_pose')
        publishers = [item for item in self.get_publishers_info_by_topic('/goal_pose')
                      if item.node_name == self.get_name()
                      and item.node_namespace == self.get_namespace()]
        evidence = []
        named = []
        for endpoint in endpoints:
            # Compare resolved DDS profiles from both graph endpoints. The
            # requested publisher profile contains unresolved system defaults.
            compatibility, reason = (qos_check_compatible(
                publishers[0].qos_profile, endpoint.qos_profile)
                if len(publishers) == 1 else (QoSCompatibility.ERROR,
                                             'local publisher endpoint unavailable/ambiguous'))
            item = {'node_name': endpoint.node_name,
                    'node_namespace': endpoint.node_namespace,
                    'topic_type': endpoint.topic_type,
                    'gid': list(endpoint.endpoint_gid),
                    'qos_compatible': compatibility == QoSCompatibility.OK,
                    'qos_reason': reason}
            evidence.append(item)
            if (endpoint.node_name == 'gazebo_navigation_interface'
                    and endpoint.node_namespace == '/'
                    and endpoint.topic_type == 'geometry_msgs/msg/PoseStamped'
                    and item['qos_compatible']):
                named.append(item)
        matched = self.goal_pub.get_subscription_count()
        compatible = sum(item['qos_compatible'] for item in evidence)
        # Include our observer in the count, but require the named receiver.
        # Stable graph + total matches is readiness evidence, not delivery ACK.
        ready = len(named) == 1 and matched >= compatible and compatible > 0
        signature = tuple(sorted(tuple(item['gid']) for item in evidence))
        prior_signature = self.receiver_signature
        prior_since = self.receiver_stable_since
        if not ready or signature != self.receiver_signature or self.receiver_stable_since is None:
            self.receiver_stable_since = now if ready else None
            self.receiver_signature = signature
        stable = (ready and self.receiver_stable_since is not None and
                  now-self.receiver_stable_since >= .5)
        self.goal_delivery['readiness'] = {
            'endpoints': evidence, 'publisher_matched_count': matched,
            'publisher_endpoints': [list(item.endpoint_gid) for item in publishers],
            'named_navigation_receivers': len(named),
            'stable_for_s': (now-self.receiver_stable_since
                             if self.receiver_stable_since is not None else 0),
            'ready': stable, 'monotonic_ns': time.monotonic_ns()}
        if hasattr(self, 'native_trace'):
            transition = {'now_monotonic_s': now, 'sample_monotonic_ns': time.monotonic_ns(),
                          'raw_ready': ready, 'compatible_count': compatible,
                          'matched_count': matched, 'named_count': len(named),
                          'signature_changed': signature != prior_signature,
                          'prior_stable_since_s': prior_since,
                          'stable_since_s': self.receiver_stable_since,
                          'stable': stable, 'signature': signature}
            history = self.goal_delivery.setdefault('readiness_samples', [])
            if len(history) < 512:
                history.append(transition)
        return stable

    def gt_cb(self, msg):
        self.trace('/gazebo/odometry', msg)
        self.gt = msg
        self.gt_received_steady = time.monotonic()
        self.gt_sequence += 1
        if not self.active:
            return
        sim_t = stamp(msg.header.stamp)
        if sim_t-self.last_sample < 0.05:
            return
        self.last_sample = sim_t
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        body = polygon(p.x, p.y, yaw, 0.52, 0.42)
        margin = min(polygon_distance(body, wall) for wall in self.walls)
        if margin <= 1e-6:
            self.overlap += 1
        cte, heading = centerline_error(p.x, p.y, self.polyline)
        yaw_rel = abs(wrap(yaw-heading))
        if yaw_rel > math.pi/2:
            yaw_rel = math.pi-yaw_rel
        effective_width = 0.42*abs(math.cos(yaw_rel))+0.52*abs(math.sin(yaw_rel))
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        goal_error = math.hypot(p.x-self.goal[0], p.y-self.goal[1])
        setpoint_error = math.nan
        spx = spy = spyaw = math.nan
        if self.setpoint is not None and abs(sim_t-self.setpoint_stamp) <= 0.2:
            sp = self.setpoint.pose.position
            spx, spy = sp.x, sp.y
            spyaw = yaw_of(self.setpoint.pose.orientation)
            setpoint_error = math.hypot(p.x-sp.x, p.y-sp.y)
        op = self.odom.pose.pose.position if self.odom else None
        cmd = self.command
        applied = self.last_actuation
        published = self.final_command
        self.samples.append({
            "sim_t": sim_t, "wall_s": time.monotonic()-self.start_wall,
            "gt_x": p.x, "gt_y": p.y, "gt_yaw": yaw, "gt_speed": speed,
            "gt_yaw_speed": abs(msg.twist.twist.angular.z),
            "odom_x": op.x if op else math.nan, "odom_y": op.y if op else math.nan,
            "gt_goal_error": goal_error,
            "odom_goal_error": math.hypot(op.x-self.goal[0], op.y-self.goal[1]) if op else math.nan,
            "centerline_cte": cte, "setpoint_error": setpoint_error,
            "setpoint_x": spx, "setpoint_y": spy, "setpoint_yaw": spyaw,
            "bspline_endpoint_x": self.local_plan_endpoint[0] if self.local_plan_endpoint else math.nan,
            "bspline_endpoint_y": self.local_plan_endpoint[1] if self.local_plan_endpoint else math.nan,
            "wall_clearance": margin, "yaw_relative_rad": yaw_rel,
            "effective_width": effective_width,
            "cmd_vx": cmd.linear.x if cmd else math.nan,
            "cmd_vy": cmd.linear.y if cmd else math.nan,
            "cmd_wz": cmd.angular.z if cmd else math.nan,
            "actuated_vx": applied.get('vx', math.nan) if applied else math.nan,
            "actuated_vy": applied.get('vy', math.nan) if applied else math.nan,
            "actuated_wz": applied.get('wz', math.nan) if applied else math.nan,
            "published_vx": published.linear.x if published else math.nan,
            "published_vy": published.linear.y if published else math.nan,
            "published_wz": published.angular.z if published else math.nan,
            "yaw_error_to_goal_rad": abs(wrap(yaw-self.sent_goal_yaw)),
        })
        if goal_error < self.last_progress_error-0.05:
            self.last_progress_error = goal_error
            self.last_progress_sim = sim_t

    def wait_ready(self, timeout_s=45):
        end = time.monotonic()+timeout_s
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if self.gt is None or self.odom is None or self.final_command is None:
                continue
            gt_pose = self.gt.pose.pose.position
            odom_pose = self.odom.pose.pose.position
            final = self.final_command
            readings = (gt_pose.x, gt_pose.y, odom_pose.x, odom_pose.y,
                        final.linear.x, final.linear.y, final.angular.z)
            if (self.gt_received_steady is not None and
                    now-self.gt_received_steady <= 0.2 and
                    self.odom_received_steady is not None and
                    now-self.odom_received_steady <= 0.5 and
                    self.final_command_received_steady is not None and
                    now-self.final_command_received_steady <= 0.2 and
                    all(math.isfinite(value) for value in readings) and
                    (not self.native_trace or
                     self.native_trace.counts['/clock'] > 0) and
                    self.goal_receiver_ready(now) and
                    "ego_planner_node" in self.observed_node_names()):
                return True
        return False

    def run(self):
        if not self.wait_ready():
            return self.save("preflight_unavailable")
        p = self.gt.pose.pose.position
        if math.hypot(p.x-self.nominal_start[0], p.y-self.nominal_start[1]) > 0.20:
            return self.save("wrong_start")
        try:
            self.grid.route((p.x,p.y), self.goal)
        except ValueError as error:
            self.events.append({"message": f"Goal Guard preflight: {error}"})
            return self.save("statically_unreachable")
        if not self.goal_receiver_ready(time.monotonic()):
            return self.save('goal_receiver_unavailable')
        self.start_wall, self.start_sim = time.monotonic(), stamp(self.gt.header.stamp)
        self.initial_gt_pose = (p.x, p.y, yaw_of(self.gt.pose.pose.orientation))
        self.active = True
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = self.goal
        self.sent_goal_yaw = (self.initial_gt_pose[2]
                              if self.yaw_mode == 'hold-start' else 0.0)
        goal.pose.orientation.z = math.sin(self.sent_goal_yaw/2)
        goal.pose.orientation.w = math.cos(self.sent_goal_yaw/2)
        self.goal_delivery.update({
            'state': 'sent_awaiting_route', 'publish_count': 1,
            'publish_monotonic_ns': time.monotonic_ns(),
            'publish_wall_ns': time.time_ns(),
            'goal_xy': list(self.goal), 'goal_yaw_rad': self.sent_goal_yaw,
            'acknowledgement_timeout_s': self.acceptance_timeout_s})
        self.goal_pub.publish(goal)
        result = "timeout"
        reached_since = None
        while rclpy.ok() and time.monotonic()-self.start_wall < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.accepted:
                if self.goal_delivery['rejection']:
                    result = 'goal_rejected'
                    break
                if time.monotonic()-self.start_wall >= self.acceptance_timeout_s:
                    self.goal_delivery['state'] = 'acceptance_timeout'
                    result = 'goal_acceptance_timeout'
                    break
            if self.terminal_validation:
                self.terminal_gate.check_gap(time.monotonic())
            if self.overlap:
                result = "geometric_overlap"
                break
            if self.gt is None:
                continue
            p = self.gt.pose.pose.position
            speed = math.hypot(self.gt.twist.twist.linear.x, self.gt.twist.twist.linear.y)
            goal_error = math.hypot(p.x-self.goal[0], p.y-self.goal[1])
            if self.completion is not None:
                self.post_completion_max_error = max(self.post_completion_max_error,
                                                     goal_error)
                self.post_completion_max_speed = max(self.post_completion_max_speed,
                                                     speed)
                if goal_error > 0.10:
                    self.post_completion_departure = True
            if goal_error <= 0.10 and speed <= 0.10:
                reached_since = reached_since or time.monotonic()
                if time.monotonic()-reached_since >= 1.5:
                    if self.arrival is None:
                        self.arrival = {'wall_elapsed_s': time.monotonic()-self.start_wall,
                                        'gt_stamp_s': stamp(self.gt.header.stamp),
                                        'gt_goal_error_m': goal_error}
                    if not self.terminal_validation:
                        result = "success"
                        break
            else:
                reached_since = None
            if self.terminal_validation and self.gt_sequence != self.last_checked_gt_sequence:
                self.last_checked_gt_sequence = self.gt_sequence
                sim_t = stamp(self.gt.header.stamp)
                now = time.monotonic()
                published = self.final_command
                command = ((published.linear.x, published.linear.y,
                            published.angular.z) if published else
                           (math.nan, math.nan, math.nan))
                ready = self.terminal_gate.advance(
                    wall_now=now, sim_t=sim_t,
                    gt_received_wall=self.gt_received_steady,
                    xy=(p.x,p.y), goal_error=goal_error, speed=speed,
                    yaw_speed=abs(self.gt.twist.twist.angular.z),
                    odom_received_wall=self.odom_received_steady,
                    odom_stamp=stamp(self.odom.header.stamp)
                    if self.odom else math.nan,
                    final_command_received_wall=self.final_command_received_steady,
                    final_command=command, completed=self.completion is not None)
                self.stop_start_sim = self.terminal_gate.stop_start_sim
                self.observe_start_sim = self.terminal_gate.observation_start_sim
                self.terminal_invalid_reasons = self.terminal_gate.invalid_reasons
                if ready:
                    result = ('post_completion_departure' if
                              self.post_completion_departure else
                              ('evidence_gap' if self.terminal_invalid_reasons
                               else 'terminal_pass'))
                    break
            if self.accepted and self.last_progress_sim is not None and goal_error > 0.20:
                if stamp(self.gt.header.stamp)-self.last_progress_sim > 20:
                    self.stuck += 1
                    result = "stuck"
                    break
            if "ego_planner_node" not in self.observed_node_names():
                result = "ego_planner_died"
                break
        self.active = False
        return self.save(result)

    def save(self, result):
        native_health = self.native_trace.close() if self.native_trace else None
        if native_health is not None and not native_health['pass']:
            result = 'evidence_gap'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        prefix = self.output_dir / self.name
        if prefix.with_suffix('.summary.json').exists():
            raise FileExistsError(f'refusing to overwrite prior trial: {prefix}')
        if self.samples:
            with prefix.with_suffix(".csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.samples[0]))
                writer.writeheader()
                writer.writerows(self.samples)
        with (self.output_dir / (self.name+".events.jsonl")).open("w", encoding="utf-8") as f:
            for event in self.events:
                f.write(json.dumps(event, ensure_ascii=False)+"\n")
        with (self.output_dir / (self.name+".plans.jsonl")).open("w", encoding="utf-8") as f:
            for plan in self.plans:
                f.write(json.dumps(plan, ensure_ascii=False)+"\n")
        with (self.output_dir / (self.name+".actuation.jsonl")).open("w", encoding="utf-8") as f:
            for diagnostic in self.actuation_trace:
                f.write(json.dumps(diagnostic, ensure_ascii=False)+"\n")
        with (self.output_dir / (self.name+".published_commands.jsonl")).open(
                "w", encoding="utf-8") as f:
            for command in self.final_command_trace:
                f.write(json.dumps(command, ensure_ascii=False)+"\n")
        p = self.gt.pose.pose.position if self.gt else None
        op = self.odom.pose.pose.position if self.odom else None
        def values(key):
            return [row[key] for row in self.samples if math.isfinite(row[key])]
        def stat(key):
            v = values(key)
            return {"mean": float(np.mean(v)), "p95": float(np.percentile(v,95)),
                    "max": float(np.max(v))} if v else None
        margins = values("wall_clearance")
        worst = max(self.samples, key=lambda row: row["yaw_relative_rad"]) if self.samples else None
        widest = max(self.samples, key=lambda row: row["effective_width"]) if self.samples else None
        closest = min(self.samples, key=lambda row: row["wall_clearance"]) if self.samples else None
        now = time.monotonic()
        gt_pose = self.gt.pose.pose if self.gt else None
        odom_pose = self.odom.pose.pose if self.odom else None
        terminal_snapshot = {
            'wall_elapsed_s': now-self.start_wall if self.start_wall else None,
            'gt_stamp_s': stamp(self.gt.header.stamp) if self.gt else None,
            'gt_received_age_s': now-self.gt_received_steady if self.gt_received_steady else None,
            'gt_xy': [gt_pose.position.x, gt_pose.position.y] if gt_pose else None,
            'gt_yaw_rad': yaw_of(gt_pose.orientation) if gt_pose else None,
            'gt_speed_mps': math.hypot(self.gt.twist.twist.linear.x,
                                      self.gt.twist.twist.linear.y) if self.gt else None,
            'gt_yaw_speed_rad_s': abs(self.gt.twist.twist.angular.z) if self.gt else None,
            'odom_stamp_s': stamp(self.odom.header.stamp) if self.odom else None,
            'odom_received_age_s': now-self.odom_received_steady if self.odom_received_steady else None,
            'odom_xy': [odom_pose.position.x, odom_pose.position.y] if odom_pose else None,
            'command': {'vx': self.command.linear.x, 'vy': self.command.linear.y,
                        'wz': self.command.angular.z} if self.command else None,
            'command_received_age_s': now-self.cmd_received_steady if self.cmd_received_steady else None,
            'applied_command': {key: self.last_actuation.get(key)
                                for key in ('vx','vy','wz','source','action')}
                               if self.last_actuation else None,
            'applied_command_received_age_s':
                now-self.last_actuation_received_steady
                if self.last_actuation_received_steady else None,
            'final_command': {
                'vx': self.final_command.linear.x,
                'vy': self.final_command.linear.y,
                'wz': self.final_command.angular.z,
            } if self.final_command else None,
            'final_command_received_age_s':
                now-self.final_command_received_steady
                if self.final_command_received_steady else None,
        }
        summary = {
            "name": self.name, "route": self.route, "result": result,
            "native_trace_health": native_health,
            "terminal_validation": self.terminal_validation,
            "goal_xy": self.goal,
            "goal_accepted": self.accepted,
            "goal_delivery": self.goal_delivery,
            "final_waypoint_id": self.final_waypoint_id,
            "navigation_completion": self.completion,
            "completion_signal": "matched /rosout final waypoint diagnostic" if self.completion else "unavailable",
            "false_success": self.false_success,
            "post_completion_departure": self.post_completion_departure,
            "post_completion_max_error_m": self.post_completion_max_error,
            "post_completion_max_speed_mps": self.post_completion_max_speed,
            "legacy_arrival": self.arrival,
            "stable_stop_2s": self.observe_start_sim is not None,
            "post_stop_observation_5s": result in ('terminal_pass',
                                                   'post_completion_departure'),
            "stable_stop_start_sim_s": self.stop_start_sim,
            "post_stop_observation_start_sim_s": self.observe_start_sim,
            "terminal_invalid_reasons": sorted(set(self.terminal_invalid_reasons)),
            "terminal_snapshot": terminal_snapshot,
            "max_yaw_error_to_goal_rad": max(values("yaw_error_to_goal_rad"),
                                              default=None),
            "yaw_mode": self.yaw_mode,
            "initial_gt_pose": self.initial_gt_pose,
            "sent_goal_yaw_rad": self.sent_goal_yaw,
            "final_gt_yaw_rad": yaw_of(self.gt.pose.pose.orientation) if self.gt else None,
            "final_yaw_required": False,
            "actuation_counts": self.actuation_counts,
            "actuation_source_counts": self.actuation_source_counts,
            "source": "PROVISIONAL TRAINING ASSUMPTION; final_arena_verified=false",
            "speed_limits": {"vx_mps":0.25,"vy_mps":0.25,"wz_rad_s":0.30,"planner_acc_mps2":0.35},
            "sample_count": len(self.samples),
            "published_plan_count": len(self.plans),
            "duration_wall_s": time.monotonic()-self.start_wall if self.start_wall else None,
            "duration_sim_s": stamp(self.gt.header.stamp)-self.start_sim if self.gt and self.start_sim else None,
            "final_gt_xy": [p.x,p.y] if p else None,
            "final_odom_xy": [op.x,op.y] if op else None,
            "gt_goal_error_m": math.hypot(p.x-self.goal[0],p.y-self.goal[1]) if p else None,
            "odom_goal_error_m": math.hypot(op.x-self.goal[0],op.y-self.goal[1]) if op else None,
            "centerline_cte_m": stat("centerline_cte"),
            "trajectory_tracking_error_m": stat("setpoint_error"),
            "minimum_physical_wall_clearance_m": min(margins) if margins else None,
            "minimum_wall_clearance_pose": [closest["gt_x"],closest["gt_y"],closest["gt_yaw"]] if closest else None,
            "maximum_relative_yaw_deg": math.degrees(worst["yaw_relative_rad"]) if worst else None,
            "effective_width_at_max_yaw_m": worst["effective_width"] if worst else None,
            "maximum_effective_width_m": widest["effective_width"] if widest else None,
            "relative_yaw_at_max_width_deg": math.degrees(widest["yaw_relative_rad"]) if widest else None,
            "geometric_overlap_samples": self.overlap,
            "contact_data": "CONTACT DATA UNAVAILABLE",
            "retry_count": self.retry, "stuck_count": self.stuck,
            "ego_obstacle_related_warnings": self.ego_warnings,
            "max_observed_cmd": {key: max((abs(v) for v in values(key)), default=None)
                                 for key in ("cmd_vx","cmd_vy","cmd_wz")},
            "measurement_basis": {"pose":"MEASURED Gazebo odometry",
                                  "wall_clearance":"INFERRED from measured pose and SDF collision boxes",
                                  "contact":"UNAVAILABLE without contact sensor"},
        }
        summary['terminal_evidence_pass'] = terminal_evidence_pass(
            result=result, accepted=self.accepted,
            final_waypoint_id=self.final_waypoint_id,
            completion=self.completion, false_success=self.false_success,
            departure=self.post_completion_departure,
            stop_start_sim=self.stop_start_sim,
            observation_start_sim=self.observe_start_sim,
            invalid_reasons=self.terminal_invalid_reasons,
            snapshot={**terminal_snapshot,
                      'gt_goal_error_m': summary['gt_goal_error_m']},
            goal=self.goal,
            minimum_wall_gap=min(margins) if margins else math.nan,
            overlap_count=self.overlap)
        self.terminal_evidence_pass = summary['terminal_evidence_pass']
        if (result == "success" or summary['terminal_evidence_pass']) and margins and self.overlap == 0:
            lateral = summary["centerline_cte_m"]
            summary["assessment"] = ("MARGINAL" if min(margins)<0.08 or self.retry>1 or
                                     (lateral and lateral["p95"]>0.03) else "PASS")
        else:
            summary["assessment"] = "FAIL"
        (self.output_dir / (self.name+".summary.json")).write_text(
            json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True, choices=ROUTES)
    parser.add_argument("--name", required=True)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--yaw-mode", choices=("zero","hold-start"), default="zero")
    parser.add_argument("--output-dir", default=str(OUT))
    parser.add_argument("--terminal-validation", action="store_true")
    parser.add_argument("--start", nargs=2, type=float)
    parser.add_argument("--goal", nargs=2, type=float)
    parser.add_argument("--raw-every-message", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    runner = Runner(args.route,args.name,args.timeout,args.yaw_mode,args.output_dir,
                    args.terminal_validation,args.start,args.goal,args.raw_every_message)
    try:
        result = runner.run()
    finally:
        if runner.native_trace:
            runner.native_trace.close()
        runner.destroy_node()
        rclpy.shutdown()
    return 0 if result == "success" or runner.terminal_evidence_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
