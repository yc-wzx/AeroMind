#!/usr/bin/env python3
"""Adapt Gazebo odometry and lidar to AeroMind's planar ROS interfaces.

The inherited class only supplies field/robot markers and prior-map sampling.
Motion is computed exclusively by Gazebo; update() never integrates a pose.
"""
import copy
from functools import wraps
import json
import math
import random
import struct
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from planar_navigation_simulator import PlanarNavigationSimulator, quaternion_from_yaw, wrap_angle
from scan_matcher import WallScanMatcher
from grid_route import GridRoute
from provincial_safety_geometry import (
    body_wall_gap, predict_command_gap, reference_stages, wall_box)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def timed_callback(method):
    """Read-only slow-callback evidence; no changes to callback scheduling."""
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        started = time.monotonic()
        try:
            return method(self, *args, **kwargs)
        finally:
            elapsed = time.monotonic()-started
            if elapsed > 0.05:
                self.get_logger().warn(
                    f'RMUC callback timing: method={method.__name__}, '
                    f'duration_s={elapsed:.6f}, '
                    f'sim_s={self.get_clock().now().nanoseconds*1e-9:.6f}')
    return wrapped


class GazeboNavigationInterface(PlanarNavigationSimulator):
    def __init__(self):
        super().__init__()
        self.waypoints = []
        self.route_diag_sequence = 0
        self.route_diag_id = None
        self.waypoint_diag_ids = {}
        self.active_waypoint_diag_id = None
        self.active_waypoint = None
        self.stage_sent_at = None
        self.stage_retry_count = 0
        self.stage_best_distance = None
        self.stage_last_progress_at = None
        self.final_approach_active = False
        self.dynamic_obstacle_points = []
        self.dynamic_scan_received_at = None
        self.dynamic_goal_clearance = float(
            self.declare_parameter('dynamic_goal_clearance', 0.55).value)
        self.last_blocked_goal_warning = 0.0
        self.last_blocked_goal_check = 0.0
        self.last_final_approach_diagnostic = 0.0
        self.route_pub = self.create_publisher(PoseStamped, '/navigation/segment_goal', 5)
        self.create_subscription(PoseStamped, '/goal_pose', self.route_goal, 5)
        self.latest_odom = None
        self.odom_history = deque(maxlen=150)
        self.prior_points = list(self.field_points)
        map_pgm = str(self.declare_parameter('occupancy_map_pgm', '').value)
        grid_clearance = float(self.declare_parameter('grid_route_clearance', 0.55).value)
        if grid_clearance <= 0:
            raise ValueError('grid_route_clearance must be positive')
        self.grid_route = (GridRoute(map_pgm, clearance=grid_clearance)
                           if map_pgm else None)
        self.provincial_reference_route_mode = bool(self.declare_parameter(
            'provincial_reference_route_mode', False).value)
        self.provincial_rect_guard = bool(self.declare_parameter(
            'provincial_rect_guard', False).value)
        self.provincial_min_body_gap = float(self.declare_parameter(
            'provincial_min_body_gap_m', 0.08).value)
        self.provincial_wall_thickness = float(self.declare_parameter(
            'provincial_wall_thickness_m', 0.055).value)
        if (self.provincial_min_body_gap <= 0 or self.provincial_wall_thickness <= 0):
            raise ValueError('provincial safety geometry parameters must be positive')
        self.provincial_walls = [wall_box(segment, self.provincial_wall_thickness)
                                 for segment in self.field['collision_segments']]
        self.planned_path_safe = None
        self.planned_path_min_gap = None
        self.planned_path_received_at = None
        self.planned_path_valid_until = None
        self.last_actuation_diag = 0.0
        self.last_actuation_state = None
        self.actuation_diag_pub = self.create_publisher(
            String, '/ground/planning/actuation_diagnostics', 10)
        self.create_subscription(
            Marker, '/ground/planning/planned_trajectory',
            self.planned_trajectory_callback, 10)
        if self.grid_route:
            self.prior_points = self.grid_route.boundary_points()
            self.field_points = list(self.prior_points)
        self.scan_received = False
        self.ready_since = None
        self.last_odom_stamp = None
        self.last_odom_received = None
        self.scan_received_at = None
        # Integrate noisy motion increments instead of publishing absolute truth.
        # Gazebo truth remains available on /gazebo/odometry for evaluation only.
        self.imperfect_sensors = bool(self.declare_parameter('imperfect_sensors', True).value)
        self.odom_scale_x = float(self.declare_parameter('odom_scale_x', 1.005).value)
        self.odom_scale_y = float(self.declare_parameter('odom_scale_y', 0.995).value)
        self.odom_yaw_bias = float(self.declare_parameter('odom_yaw_bias_rad_s', 0.0005).value)
        self.odom_walk = float(self.declare_parameter('odom_walk_m_sqrt_s', 0.002).value)
        self.odom_yaw_walk = float(self.declare_parameter('odom_yaw_walk_rad_sqrt_s', 0.0003).value)
        self.drive_scale = float(self.declare_parameter('drive_speed_scale', 0.97).value)
        self.drive_tau = float(self.declare_parameter('drive_response_sec', 0.12).value)
        self.random = random.Random(int(self.declare_parameter('noise_seed', 2025).value))
        self.estimate = None
        self.estimated_velocity = (0.0, 0.0, 0.0)
        self.last_estimate_stamp = None
        self.last_truth_pose = None
        # The supplied RMUC occupancy map and visual STL differ locally by
        # decimetres. Matching their edges corrupts pose more than dead
        # reckoning; keep live lidar for obstacle avoidance and noisy odometry
        # for pose until a calibrated map is available.
        self.scan_matcher = (None if self.grid_route else
                             WallScanMatcher(self.field['collision_segments']))
        self.scan_match_count = 0
        self.last_drive_clock = None
        self.actuated = Twist()
        self.auto_goal = bool(self.declare_parameter('auto_goal', True).value)
        self.drive_pub = self.create_publisher(
            Twist, '/model/omni_robot/cmd_vel', 10)
        self.scan_cloud_pub = self.create_publisher(
            PointCloud2, '/gazebo/registered_scan', 5)
        self.create_subscription(
            Odometry, '/gazebo/odometry', self.gazebo_odom, 30)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Gazebo backend: noisy motion-increment odometry, '
            f'lidar matching={self.scan_matcher is not None}, '
            f'imperfect_sensors={self.imperfect_sensors}; '
            'known field boundaries are used as the prior navigation map.')

    def route_projection(self, x, y):
        best = None
        length = 0.0
        for a,b in zip(self.field['reference_route'], self.field['reference_route'][1:]):
            dx,dy = b[0]-a[0],b[1]-a[1]
            segment = math.hypot(dx,dy)
            ratio = max(0.0,min(1.0,((x-a[0])*dx+(y-a[1])*dy)/(segment*segment)))
            px,py = a[0]+ratio*dx,a[1]+ratio*dy
            candidate = (math.hypot(x-px,y-py),length+ratio*segment)
            if best is None or candidate[0] < best[0]:
                best = candidate
            length += segment
        return best

    def route_goal(self, goal):
        if self.latest_odom is None or goal.header.frame_id not in ('map','odom',''):
            self.get_logger().warn('Goal requires Gazebo odometry and map/odom frame')
            return
        x,y = goal.pose.position.x, goal.pose.position.y
        q=goal.pose.orientation
        if not all(math.isfinite(v) for v in (x,y,q.x,q.y,q.z,q.w)):
            return
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0) > 0.02:
            return
        if self.grid_route:
            try:
                route = self.grid_route.route((self.x, self.y), (x, y))
                if self.provincial_reference_route_mode:
                    route = reference_stages(
                        (self.x, self.y), (x, y), self.field['reference_route'])
                    previous = (self.x, self.y)
                    for stage in route:
                        if not self.grid_route.line_safe(
                                previous, stage,
                                clearance=self.grid_route.clearance_required):
                            raise ValueError(
                                f'configured reference leg {previous} -> {stage} '
                                'is not statically safe')
                        previous = stage
            except ValueError as error:
                self.get_logger().warn(f'GOAL REJECTED: STATICALLY UNREACHABLE: {error}')
                return
            self.route_diag_sequence += 1
            self.route_diag_id = f"R{self.route_diag_sequence:04d}"
            self.waypoint_diag_ids = {
                (round(float(wx), 5), round(float(wy), 5)):
                f"{self.route_diag_id}:W{index:02d}"
                for index, (wx, wy) in enumerate(route)
            }
            route_summary = ";".join(
                f"{self.waypoint_diag_ids[(round(float(wx), 5), round(float(wy), 5))]}"
                f"=({float(wx):.3f},{float(wy):.3f})"
                for wx, wy in route)
            self.get_logger().info(
                f"RMUC route diagnostic route={self.route_diag_id} planned={route_summary}")
            self.waypoints=[]
            for cx,cy in route[:-1]:
                pose=PoseStamped()
                pose.header.frame_id='odom'
                pose.pose.position.x=float(cx)
                pose.pose.position.y=float(cy)
                pose.pose.orientation=quaternion_from_yaw(self.yaw)
                self.waypoints.append(pose)
            final=copy.deepcopy(goal)
            final.header.frame_id='odom'
            self.waypoints.append(final)
            self.get_logger().info(f'RMUC mission accepted: {len(self.waypoints)} route stages')
            self.send_next_waypoint()
            return
        for x1,y1,x2,y2 in self.field['collision_segments']:
            dx,dy=x2-x1,y2-y1
            t=max(0.0,min(1.0,((x-x1)*dx+(y-y1)*dy)/(dx*dx+dy*dy)))
            if math.hypot(x-x1-t*dx,y-y1-t*dy) < 0.36:
                self.get_logger().warn('Goal too close to field boundary; choose lane centre')
                return
        distance, end = self.route_projection(x,y)
        if distance > 0.55:
            self.get_logger().warn('Goal outside automatic course; choose a point on the lane')
            return
        _, start = self.route_projection(self.x,self.y)
        route=self.field['reference_route']
        distance=0.0
        corners=[]
        for a,b in zip(route,route[1:]):
            distance += math.hypot(b[0]-a[0],b[1]-a[1])
            if min(start,end)+0.12 < distance < max(start,end)-0.12:
                corners.append((b[0],b[1]))
        if end < start:
            corners.reverse()
        self.waypoints=[]
        for cx,cy in corners:
            pose=PoseStamped()
            pose.header.frame_id='odom'
            pose.pose.position.x=float(cx)
            pose.pose.position.y=float(cy)
            pose.pose.orientation=quaternion_from_yaw(self.yaw)
            self.waypoints.append(pose)
        final=copy.deepcopy(goal)
        final.header.frame_id='odom'
        self.waypoints.append(final)
        self.get_logger().info(f'Mission accepted: {len(self.waypoints)} route stages')
        self.send_next_waypoint()

    def diag_waypoint_id(self, waypoint):
        p = waypoint.pose.position
        return self.waypoint_diag_ids.get(
            (round(float(p.x), 5), round(float(p.y), 5)), "unknown")

    def send_next_waypoint(self):
        self.planned_path_safe = None
        self.planned_path_min_gap = None
        self.planned_path_received_at = None
        self.planned_path_valid_until = None
        if not self.waypoints:
            self.active_waypoint=None
            self.active_waypoint_diag_id=None
            self.stage_sent_at=None
            self.final_approach_active=False
            return
        candidate_index = next((index for index, candidate in enumerate(self.waypoints)
                                if not self.dynamic_waypoint_blocked(candidate)), None)
        if candidate_index is None:
            self.active_waypoint = None
            self.active_waypoint_diag_id = None
            self.stage_sent_at = None
            self.final_approach_active = False
            now = time.monotonic()
            if now-self.last_blocked_goal_warning > 1.0:
                self.get_logger().warn(
                    'All remaining RMUC route stages are occupied in the fresh lidar scan; '
                    'holding without resending an occupied endpoint')
                self.last_blocked_goal_warning = now
            return
        if candidate_index:
            skipped = self.waypoints[:candidate_index]
            skipped_ids = [self.diag_waypoint_id(item) for item in skipped]
            self.get_logger().info(
                f"RMUC dynamic skip diagnostic route={self.route_diag_id} "
                f"waypoints={skipped_ids}")
            del self.waypoints[:candidate_index]
            self.get_logger().warn(
                f'Skipping {len(skipped)} dynamically occupied RMUC route stage(s); '
                'selecting the next free static-path waypoint')
        self.active_waypoint=self.waypoints.pop(0)
        self.active_waypoint_diag_id = self.diag_waypoint_id(self.active_waypoint)
        self.stage_sent_at=time.monotonic()
        self.stage_retry_count=0
        target=self.active_waypoint.pose.position
        self.stage_best_distance=math.hypot(target.x-self.x,target.y-self.y)
        self.stage_last_progress_at=self.stage_sent_at
        self.final_approach_active=False
        self.active_waypoint.header.stamp=self.get_clock().now().to_msg()
        self.route_pub.publish(self.active_waypoint)
        p=self.active_waypoint.pose.position
        self.get_logger().info(f'Navigation stage: ({p.x:.2f}, {p.y:.2f})')
        self.get_logger().info(
            f"RMUC waypoint diagnostic sent route={self.route_diag_id} "
            f"waypoint={self.active_waypoint_diag_id} target=({p.x:.3f},{p.y:.3f})")

    @timed_callback
    def planned_trajectory_callback(self, marker):
        if not self.provincial_rect_guard or not marker.points or self.active_waypoint is None:
            return
        if marker.header.frame_id not in ('odom', 'map'):
            return
        if stamp_seconds(marker.header.stamp) <= stamp_seconds(
                self.active_waypoint.header.stamp):
            return
        goal_q = self.active_waypoint.pose.orientation
        goal_yaw = math.atan2(2.0*(goal_q.w*goal_q.z+goal_q.x*goal_q.y),
                              1.0-2.0*(goal_q.y*goal_q.y+goal_q.z*goal_q.z))
        yaw_error = wrap_angle(goal_yaw-self.yaw)
        gaps = []
        for index, point in enumerate(marker.points):
            # The marker uses the executor's 0.05 s path sample interval.
            turn = max(-0.30*index*0.05,
                       min(0.30*index*0.05, yaw_error))
            gaps.append(body_wall_gap(point.x, point.y, self.yaw+turn,
                                      self.provincial_walls))
        self.planned_path_min_gap = min(gaps)
        self.planned_path_safe = self.planned_path_min_gap >= self.provincial_min_body_gap
        self.planned_path_received_at = time.monotonic()
        self.planned_path_valid_until = (stamp_seconds(marker.header.stamp) +
                                         max(0.2, (len(marker.points)-1)*0.05))

    def publish_actuation_diagnostic(self, source, action, command, predicted_gap):
        now = time.monotonic()
        state = (source, action)
        if state == self.last_actuation_state and now-self.last_actuation_diag < 0.10:
            return
        self.last_actuation_state = state
        self.last_actuation_diag = now
        message = String()
        navigation_state = ('active' if self.active_waypoint is not None else
                            'waiting_for_clear_waypoint' if self.waypoints else
                            'route_complete' if self.route_diag_id is not None else
                            'idle')
        message.data = json.dumps({
            'source': source, 'action': action,
            'navigation_state': navigation_state,
            'pending_waypoints': len(self.waypoints),
            'waypoint': self.active_waypoint_diag_id,
            'vx': command.linear.x, 'vy': command.linear.y,
            'wz': command.angular.z,
            'predicted_gap_m': predicted_gap,
            'planned_path_min_gap_m': self.planned_path_min_gap,
            'planned_path_safe': self.planned_path_safe})
        self.actuation_diag_pub.publish(message)

    def dynamic_waypoint_blocked(self, waypoint):
        # Whether a fresh, non-static lidar return occupies this stage goal.
        if (self.dynamic_scan_received_at is None or
                time.monotonic()-self.dynamic_scan_received_at > 0.75):
            return False
        goal = waypoint.pose.position
        radius2 = self.dynamic_goal_clearance*self.dynamic_goal_clearance
        return any((point[0]-goal.x)**2+(point[1]-goal.y)**2 <= radius2
                   for point in self.dynamic_obstacle_points)

    def gazebo_odom(self, message):
        self.latest_odom = message
        self.last_odom_received = time.monotonic()
        stamp = stamp_seconds(message.header.stamp)
        q = message.pose.pose.orientation
        true_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        true_pose = (message.pose.pose.position.x,
                     message.pose.pose.position.y, true_yaw)
        vx, vy = message.twist.twist.linear.x, message.twist.twist.linear.y
        wz = message.twist.twist.angular.z
        if self.estimate is None or not self.imperfect_sensors:
            self.estimate = true_pose
            c, s = math.cos(true_yaw), math.sin(true_yaw)
            self.estimated_velocity = (c*vx-s*vy, s*vx+c*vy, wz)
        elif self.last_estimate_stamp is not None and self.last_truth_pose is not None:
            dt = stamp - self.last_estimate_stamp
            if 0.0 < dt < 0.5:
                x, y, yaw = self.estimate
                tx, ty, tyaw = self.last_truth_pose
                # Position increments describe actual motion even after contact;
                # Gazebo's instantaneous twist alone can report motion while
                # the collision solver keeps the chassis stationary.
                dx, dy = true_pose[0]-tx, true_pose[1]-ty
                dc, ds = math.cos(tyaw), math.sin(tyaw)
                body_dx, body_dy = dc*dx+ds*dy, -ds*dx+dc*dy
                body_dx *= self.odom_scale_x
                body_dy *= self.odom_scale_y
                dyaw = wrap_angle(true_yaw-tyaw) + self.odom_yaw_bias*dt + self.random.gauss(
                    0.0, self.odom_yaw_walk*math.sqrt(dt))
                c, s = math.cos(yaw + dyaw/2), math.sin(yaw + dyaw/2)
                world_dx, world_dy = c*body_dx-s*body_dy, s*body_dx+c*body_dy
                walk = self.odom_walk*math.sqrt(dt)
                self.estimate = (x + world_dx + self.random.gauss(0.0, walk),
                                 y + world_dy + self.random.gauss(0.0, walk),
                                 wrap_angle(yaw+dyaw))
        if self.imperfect_sensors:
            # Differentiating noisy position increments would magnify random
            # walk by dt; use a noisy instantaneous velocity instead.
            estimate_yaw = self.estimate[2]
            c, s = math.cos(estimate_yaw), math.sin(estimate_yaw)
            self.estimated_velocity = (
                c*vx-s*vy+self.random.gauss(0.0, 0.015),
                s*vx+c*vy+self.random.gauss(0.0, 0.015),
                wz+self.random.gauss(0.0, 0.01))
        self.last_estimate_stamp = stamp
        self.last_truth_pose = true_pose
        self.odom_history.append((stamp, *self.estimate))

    @timed_callback
    def scan_callback(self, scan):
        if not self.odom_history:
            return
        stamp = stamp_seconds(scan.header.stamp)
        t, x, y, yaw = min(self.odom_history, key=lambda p: abs(p[0]-stamp))
        if abs(t-stamp) > 0.1:
            return
        if self.imperfect_sensors and self.scan_matcher is not None:
            correction = self.scan_matcher.match((x, y, yaw), scan)
            if correction is not None:
                dx, dy, dyaw = (0.7*value for value in correction)
                self.estimate = (self.estimate[0]+dx,
                                 self.estimate[1]+dy,
                                 wrap_angle(self.estimate[2]+dyaw))
                self.odom_history = deque(((ts, px+dx, py+dy,
                    wrap_angle(pyaw+dyaw)) for ts, px, py, pyaw in self.odom_history),
                    maxlen=150)
                x, y, yaw = x+dx, y+dy, wrap_angle(yaw+dyaw)
                self.scan_match_count += 1
        points = []
        ray_angles = []
        ray_distances = []
        for index, distance in enumerate(scan.ranges):
            if math.isfinite(distance) and scan.range_min < distance < scan.range_max:
                angle = yaw + scan.angle_min + index*scan.angle_increment
                points.append((x + distance*math.cos(angle),
                               y + distance*math.sin(angle), 0.0))
                ray_angles.append(angle)
                ray_distances.append(distance)
        cloud = PointCloud2()
        cloud.header.stamp = scan.header.stamp
        cloud.header.frame_id = 'odom'
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=n, offset=i*4,
                        datatype=PointField.FLOAT32, count=1)
                        for i,n in enumerate(('x','y','z'))]
        cloud.point_step, cloud.row_step = 12, 12*len(points)
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
        self.scan_cloud_pub.publish(cloud)
        if self.grid_route:
            # Preserve the proven EGO scan filter. Use same-ray map checks only
            # for stage occupancy, so a nearby mapped wall cannot hide a new
            # obstacle without adding every map mismatch to EGO's local grid.
            planner_returns = [point for point in points if
                               not self.grid_route.near_static_obstacle(
                                   point[0], point[1], tolerance=0.60)]
            dynamic_returns = []
            for index, point in enumerate(points):
                if not self.grid_route.near_static_obstacle(
                        point[0], point[1], tolerance=0.60):
                    continue
                if self.grid_route.ray_has_unmapped_return(
                        (x, y), ray_angles[index], ray_distances[index], margin=0.60):
                    dynamic_returns.append(point)
            self.dynamic_obstacle_points = dynamic_returns
            self.dynamic_scan_received_at = time.monotonic()
        else:
            planner_returns = points
        self.field_points = self.prior_points + planner_returns
        self.scan_received = bool(points)
        self.scan_received_at = time.monotonic()

    def observe_guard_resume(self, action, now_steady):
        """Give a resumed executable stage its existing no-progress window.

        Persistent guard rejection still uses the normal retry timer. Only a
        blocked-to-allowed transition of the same active waypoint restarts it.
        """
        waypoint = self.active_waypoint_diag_id if self.active_waypoint else None
        previous = getattr(self, 'progress_guard_state', None)
        blocked = action in ('unsafe_plan', 'unsafe_final_command',
                             'missing_plan', 'stale_stop')
        if (waypoint is not None and previous is not None and
                previous[0] == waypoint and previous[1] and action == 'allowed'):
            self.stage_last_progress_at = now_steady
            self.get_logger().info(
                f'RMUC progress observation resumed: waypoint={waypoint}, '
                f'sim_s={self.get_clock().now().nanoseconds*1e-9:.6f}')
        self.progress_guard_state = (waypoint, blocked)

    @timed_callback
    def update(self):
        now_steady = time.monotonic()
        desired_x = self.command.linear.x
        desired_y = self.command.linear.y
        desired_yaw = self.command.angular.z
        command_source = 'ego_executor'
        if self.provincial_reference_route_mode and self.active_waypoint is None:
            # An old EGO spline can remain fresh while there is no active
            # waypoint, including while all pending stages are occupied.
            desired_x = desired_y = desired_yaw = 0.0
            command_source = ('initial_idle_stop' if self.route_diag_id is None
                              else 'blocked_waypoint_hold' if self.waypoints
                              else 'route_complete_stop')
        if (self.grid_route and self.active_waypoint is not None and
                self.stage_sent_at is not None and
                now_steady-self.stage_sent_at > 3.0):
            p = self.active_waypoint.pose.position
            dx,dy = p.x-self.x,p.y-self.y
            remaining = math.hypot(dx,dy)
            direct_clear = self.grid_route.line_safe(
                (self.x,self.y),(p.x,p.y),
                clearance=min(0.50, self.grid_route.clearance_required))
            if (not self.final_approach_active and 0.10 < remaining < 0.40
                    and direct_clear):
                self.final_approach_active = True
                self.get_logger().info('RMUC low-speed waypoint approach engaged')
            if self.final_approach_active and direct_clear and remaining < 0.50:
                c,s = math.cos(self.yaw),math.sin(self.yaw)
                desired_x = max(-0.25,min(0.25,c*dx+s*dy))
                desired_y = max(-0.25,min(0.25,-s*dx+c*dy))
                desired_yaw = 0.0
                command_source = 'final_approach'
            elif self.final_approach_active:
                self.final_approach_active = False
        # Wall-time watchdog also handles stale sensor data independently of /clock.
        command_fresh = (self.last_command_time is not None and
                         now_steady-self.last_command_time < self.command_timeout)
        # The low-speed final approach is generated here, so it must not be
        # disabled merely because EGO has completed and stopped its trajectory.
        fresh = ((command_fresh or self.final_approach_active) and
                 self.last_odom_received is not None and
                 now_steady-self.last_odom_received < 0.5 and
                 self.scan_received_at is not None and
                 now_steady-self.scan_received_at < 1.0)
        if (self.final_approach_active and
                now_steady-self.last_final_approach_diagnostic > 1.0):
            odom_age = (float('inf') if self.last_odom_received is None else
                        now_steady-self.last_odom_received)
            scan_age = (float('inf') if self.scan_received_at is None else
                        now_steady-self.scan_received_at)
            active_goal = (None if self.active_waypoint is None else
                           (self.active_waypoint.pose.position.x,
                            self.active_waypoint.pose.position.y))
            goal_error = (None if self.active_waypoint is None else
                          math.hypot(active_goal[0]-self.x,
                                     active_goal[1]-self.y))
            near_goal_returns = (0 if active_goal is None else
                sum((point[0]-active_goal[0])**2+(point[1]-active_goal[1])**2 <=
                    self.dynamic_goal_clearance*self.dynamic_goal_clearance
                    for point in self.dynamic_obstacle_points))
            self.get_logger().info(
                f'RMUC final approach status: fresh={fresh}, '
                f'command_fresh={command_fresh}, odom_age={odom_age:.2f}s, '
                f'scan_age={scan_age:.2f}s, pose=({self.x:.3f},{self.y:.3f}), '
                f'goal={active_goal}, remaining={goal_error}, '
                f'near_lidar_returns={near_goal_returns}, '
                f'cmd=({desired_x:.2f},{desired_y:.2f})')
            self.last_final_approach_diagnostic = now_steady
        if fresh:
            sim_now = self.get_clock().now().nanoseconds
            dt = 0.0 if self.last_drive_clock is None else max(
                0.0, min(0.2, (sim_now-self.last_drive_clock)*1e-9))
            self.last_drive_clock = sim_now
            alpha = 1.0 if not self.imperfect_sensors or self.drive_tau <= 0 else (
                1.0-math.exp(-dt/self.drive_tau))
            scale = self.drive_scale if self.imperfect_sensors else 1.0
            self.actuated.linear.x += alpha*(scale*desired_x-self.actuated.linear.x)
            self.actuated.linear.y += alpha*(scale*desired_y-self.actuated.linear.y)
            self.actuated.angular.z += alpha*(desired_yaw-self.actuated.angular.z)
        else:
            # Sensor/command watchdog is an immediate stop, not a delayed command.
            self.actuated = Twist()
            self.last_drive_clock = None
            command_source = 'stale_stop'
        safety_action = 'disabled'
        predicted_gap = None
        if self.provincial_rect_guard:
            safety_action = 'allowed'
            if fresh and self.active_waypoint is not None:
                marker_fresh = (self.planned_path_valid_until is not None and
                                self.get_clock().now().nanoseconds*1e-9 <
                                self.planned_path_valid_until)
                require_plan = command_source == 'ego_executor'
                if require_plan and (not marker_fresh or self.planned_path_safe is not True):
                    safety_action = ('missing_plan' if not marker_fresh
                                     else 'unsafe_plan')
                    self.actuated = Twist()
                else:
                    predicted_gap = predict_command_gap(
                        self.x, self.y, self.yaw,
                        self.actuated.linear.x, self.actuated.linear.y,
                        self.actuated.angular.z, self.provincial_walls)
                    if predicted_gap < self.provincial_min_body_gap:
                        safety_action = 'unsafe_final_command'
                        self.actuated = Twist()
            elif not fresh:
                safety_action = 'stale_stop'
        if self.provincial_reference_route_mode and self.active_waypoint is None:
            # Guarantee zero at the final output even if upstream continues
            # publishing or drive dynamics retain a previous nonzero value.
            self.actuated = Twist()
        self.publish_actuation_diagnostic(
            command_source, safety_action, self.actuated, predicted_gap)
        self.drive_pub.publish(self.actuated)
        if self.latest_odom is None:
            return
        source = self.latest_odom
        if source.header.stamp == self.last_odom_stamp:
            return
        self.last_odom_stamp = copy.deepcopy(source.header.stamp)
        message = copy.deepcopy(source)
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        self.x, self.y, self.yaw = self.estimate
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        self.vx_world, self.vy_world, self.yaw_rate = self.estimated_velocity
        message.twist.twist.linear.x = self.vx_world
        message.twist.twist.linear.y = self.vy_world
        message.twist.twist.angular.z = self.yaw_rate
        if self.imperfect_sensors:
            message.pose.covariance[0] = message.pose.covariance[7] = 0.03**2
            message.pose.covariance[35] = math.radians(1.0)**2
        self.odom_pub.publish(message)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        if now_steady-self.last_path_sample >= 0.08:
            pose = PoseStamped()
            pose.header, pose.pose = message.header, message.pose.pose
            self.path.header = message.header
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-4000:]
            self.path_pub.publish(self.path)
            self.last_path_sample = now_steady
        if self.grid_route and self.active_waypoint is None and self.waypoints:
            if now_steady-self.last_blocked_goal_check > 0.5:
                self.last_blocked_goal_check = now_steady
                self.send_next_waypoint()
        if (self.grid_route and self.active_waypoint is not None and
                self.dynamic_waypoint_blocked(self.active_waypoint)):
            p = self.active_waypoint.pose.position
            self.get_logger().warn(
                f"RMUC dynamic requeue diagnostic route={self.route_diag_id} "
                f"waypoint={self.active_waypoint_diag_id} pose=({self.x:.3f},{self.y:.3f}) "
                f"target=({p.x:.3f},{p.y:.3f})")
            # The endpoint may become occupied after it was sent. Reconsider it
            # against remaining static-path stages and hold if all are occupied.
            self.waypoints.insert(0, self.active_waypoint)
            self.active_waypoint = None
            self.stage_sent_at = None
            self.send_next_waypoint()
        self.observe_guard_resume(safety_action, now_steady)
        if self.active_waypoint is not None:
            p=self.active_waypoint.pose.position
            distance=math.hypot(self.x-p.x,self.y-p.y)
            speed=math.hypot(self.vx_world,self.vy_world)
            if (self.provincial_reference_route_mode and
                    (self.stage_best_distance is None or
                     distance < self.stage_best_distance-0.03)):
                self.stage_best_distance = distance
                self.stage_last_progress_at = now_steady
            retry_no_progress = (
                not self.provincial_reference_route_mode or
                (self.stage_last_progress_at is not None and
                 now_steady-self.stage_last_progress_at > 2.0))
            can_cut_corner = (not self.provincial_reference_route_mode and
                              self.grid_route is not None and bool(self.waypoints)
                              and distance < 0.25 and
                              self.grid_route.line_safe(
                                  (self.x,self.y),
                                  (self.waypoints[0].pose.position.x,
                                   self.waypoints[0].pose.position.y),
                                  clearance=min(0.50, self.grid_route.clearance_required)))
            waypoint_tolerance = (
                (0.06 if self.waypoints else 0.05)
                if self.provincial_reference_route_mode else
                (0.20 if self.waypoints else 0.10))
            # The final route-complete event must describe a nearly parked
            # robot; otherwise stopping the superseded EGO spline is too early.
            handoff_speed = (0.02 if self.provincial_reference_route_mode else 0.10)
            if speed < handoff_speed and (distance < waypoint_tolerance or can_cut_corner):
                reason = "corner_cut" if can_cut_corner and distance >= waypoint_tolerance else "tolerance"
                self.get_logger().info(
                    f"RMUC waypoint diagnostic completed route={self.route_diag_id} "
                    f"waypoint={self.active_waypoint_diag_id} reason={reason} "
                    f"pose=({self.x:.3f},{self.y:.3f}) distance={distance:.3f} speed={speed:.3f}")
                self.send_next_waypoint()
            elif (self.grid_route and self.stage_sent_at is not None and
                  now_steady-self.stage_sent_at > 4.0 and speed < 0.08 and
                  retry_no_progress and
                  self.stage_retry_count < 4):
                # EGO can finish a short spline while still outside the
                # waypoint tolerance. A fresh goal resumes its FSM.
                self.stage_retry_count += 1
                self.stage_sent_at = now_steady
                self.active_waypoint.header.stamp=self.get_clock().now().to_msg()
                self.route_pub.publish(self.active_waypoint)
                self.get_logger().warn(
                    f'Retrying RMUC stage {self.stage_retry_count}: '
                    f'{distance:.2f} m from waypoint')
                self.get_logger().info(
                    f"RMUC retry diagnostic route={self.route_diag_id} "
                    f"waypoint={self.active_waypoint_diag_id} pose=({self.x:.3f},{self.y:.3f}) "
                    f"target=({p.x:.3f},{p.y:.3f}) distance={distance:.3f} speed={speed:.3f}")
            elif (self.grid_route and self.stage_sent_at is not None and
                  now_steady-self.stage_sent_at > 4.0 and speed < 0.08 and
                  retry_no_progress and
                  self.stage_retry_count >= 4):
                safe_index = next((index for index, candidate in enumerate(self.waypoints)
                                   if self.grid_route.line_safe(
                                       (self.x, self.y),
                                       (candidate.pose.position.x,
                                        candidate.pose.position.y),
                                       clearance=min(0.50, self.grid_route.clearance_required))
                                   and not self.dynamic_waypoint_blocked(candidate)),
                                  None)
                if safe_index is None:
                    self.stage_sent_at = None
                    self.get_logger().warn(
                        'RMUC stage exhausted its retries; no clear downstream '
                        'static-path stage is available, holding without success')
                else:
                    blocked_ids = [self.diag_waypoint_id(item) for item in self.waypoints[:safe_index]]
                    new_waypoint_id = self.diag_waypoint_id(self.waypoints[safe_index])
                    del self.waypoints[:safe_index]
                    self.active_waypoint = None
                    self.stage_sent_at = None
                    self.final_approach_active = False
                    self.get_logger().warn(
                        f"RMUC recovery diagnostic route={self.route_diag_id} "
                        f"old_waypoint={self.active_waypoint_diag_id} "
                        f"new_waypoint={new_waypoint_id} "
                        f"blocked_downstream={blocked_ids} pose=({self.x:.3f},{self.y:.3f})")
                    self.get_logger().warn(
                        f'RMUC stage made no progress after {self.stage_retry_count} '
                        f'retries; skipping it and {safe_index} blocked downstream '
                        'stage(s) for the next clear static-path waypoint')
                    self.send_next_waypoint()
        if self.scan_received and self.ready_since is None:
            self.ready_since = self.get_clock().now().nanoseconds
        if (self.auto_goal and not self.goal_sent and self.ready_since is not None
                and (self.get_clock().now().nanoseconds-self.ready_since)*1e-9 >= self.goal_delay):
            goal = PoseStamped()
            goal.header = message.header
            goal.pose.position.x, goal.pose.position.y = self.goal_x, self.goal_y
            goal.pose.orientation = quaternion_from_yaw(self.goal_yaw)
            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.get_logger().info('Gazebo sensors ready; published shooting-zone goal')

    def destroy_node(self):
        if rclpy.ok():
            self.drive_pub.publish(Twist())
        super().destroy_node()


def main():
    rclpy.init()
    node = GazeboNavigationInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
