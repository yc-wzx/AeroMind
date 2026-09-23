#!/usr/bin/env python3
"""Adapt Gazebo odometry and lidar to AeroMind's planar ROS interfaces.

The inherited class only supplies field/robot markers and prior-map sampling.
Motion is computed exclusively by Gazebo; update() never integrates a pose.
"""
import copy
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

from planar_navigation_simulator import PlanarNavigationSimulator, quaternion_from_yaw, wrap_angle
from scan_matcher import WallScanMatcher


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class GazeboNavigationInterface(PlanarNavigationSimulator):
    def __init__(self):
        super().__init__()
        self.waypoints = []
        self.active_waypoint = None
        self.route_pub = self.create_publisher(PoseStamped, '/navigation/segment_goal', 5)
        self.create_subscription(PoseStamped, '/goal_pose', self.route_goal, 5)
        self.latest_odom = None
        self.odom_history = deque(maxlen=150)
        self.prior_points = list(self.field_points)
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
        self.scan_matcher = WallScanMatcher(self.field['collision_segments'])
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
            f'Gazebo backend: noisy motion-increment odometry + lidar wall matching, '
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

    def send_next_waypoint(self):
        if not self.waypoints:
            self.active_waypoint=None
            return
        self.active_waypoint=self.waypoints.pop(0)
        self.active_waypoint.header.stamp=self.get_clock().now().to_msg()
        self.route_pub.publish(self.active_waypoint)
        p=self.active_waypoint.pose.position
        self.get_logger().info(f'Navigation stage: ({p.x:.2f}, {p.y:.2f})')

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
                self.estimated_velocity = (world_dx/dt, world_dy/dt,
                                           dyaw/dt)
        self.last_estimate_stamp = stamp
        self.last_truth_pose = true_pose
        self.odom_history.append((stamp, *self.estimate))

    def scan_callback(self, scan):
        if not self.odom_history:
            return
        stamp = stamp_seconds(scan.header.stamp)
        t, x, y, yaw = min(self.odom_history, key=lambda p: abs(p[0]-stamp))
        if abs(t-stamp) > 0.1:
            return
        if self.imperfect_sensors:
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
        for index, distance in enumerate(scan.ranges):
            if math.isfinite(distance) and scan.range_min < distance < scan.range_max:
                angle = yaw + scan.angle_min + index*scan.angle_increment
                points.append((x + distance*math.cos(angle),
                               y + distance*math.sin(angle), 0.0))
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
        self.field_points = self.prior_points + points
        self.scan_received = bool(points)
        self.scan_received_at = time.monotonic()

    def update(self):
        now_steady = time.monotonic()
        # Wall-time watchdog also handles stale sensor data independently of /clock.
        fresh = (self.last_command_time is not None and
                 now_steady-self.last_command_time < self.command_timeout and
                 self.last_odom_received is not None and
                 now_steady-self.last_odom_received < 0.5 and
                 self.scan_received_at is not None and
                 now_steady-self.scan_received_at < 1.0)
        if fresh:
            sim_now = self.get_clock().now().nanoseconds
            dt = 0.0 if self.last_drive_clock is None else max(
                0.0, min(0.2, (sim_now-self.last_drive_clock)*1e-9))
            self.last_drive_clock = sim_now
            alpha = 1.0 if not self.imperfect_sensors or self.drive_tau <= 0 else (
                1.0-math.exp(-dt/self.drive_tau))
            scale = self.drive_scale if self.imperfect_sensors else 1.0
            self.actuated.linear.x += alpha*(scale*self.command.linear.x-self.actuated.linear.x)
            self.actuated.linear.y += alpha*(scale*self.command.linear.y-self.actuated.linear.y)
            self.actuated.angular.z += alpha*(self.command.angular.z-self.actuated.angular.z)
        else:
            # Sensor/command watchdog is an immediate stop, not a delayed command.
            self.actuated = Twist()
            self.last_drive_clock = None
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
        if self.active_waypoint is not None:
            p=self.active_waypoint.pose.position
            if (math.hypot(self.x-p.x,self.y-p.y) < 0.10 and
                    math.hypot(self.vx_world,self.vy_world) < 0.10):
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
