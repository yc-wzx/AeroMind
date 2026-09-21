#!/usr/bin/env python3
"""Lightweight closed-loop simulator for AeroMind's holonomic planar stack."""

import math
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


class PlanarNavigationSimulator(Node):
    def __init__(self):
        super().__init__('planar_navigation_simulator')
        self.goal_x = float(self.declare_parameter('goal_x', 5.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)
        self.goal_yaw = float(self.declare_parameter('goal_yaw', 0.0).value)
        self.goal_delay = float(self.declare_parameter('goal_delay_sec', 2.0).value)
        self.max_acceleration = float(
            self.declare_parameter('max_acceleration_mps2', 2.5).value)
        self.max_yaw_acceleration = float(
            self.declare_parameter('max_yaw_acceleration_rad_s2', 4.0).value)
        self.command_timeout = float(
            self.declare_parameter('command_timeout_sec', 0.5).value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx_world = 0.0
        self.vy_world = 0.0
        self.yaw_rate = 0.0
        self.command = Twist()
        self.last_command_time = None
        self.started = time.monotonic()
        self.last_update = self.started
        self.goal_sent = False
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_path_sample = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/ground/odometry', 20)
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_2d', 5)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 5)
        self.path_pub = self.create_publisher(Path, '/path', 5)
        self.obstacle_pub = self.create_publisher(
            Marker, '/ground/simulation/obstacle', 1)
        self.robot_pub = self.create_publisher(Marker, '/ground/simulation/robot', 1)
        self.evidence_pub = self.create_publisher(
            Marker, '/ground/simulation/evidence', 1)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 20)

        self.create_timer(0.02, self.update)
        self.create_timer(0.10, self.publish_obstacles)
        self.create_timer(0.10, self.publish_markers)
        self.get_logger().info(
            'Planar holonomic simulation ready: automatic goal '
            f'({self.goal_x:.1f}, {self.goal_y:.1f}, yaw={self.goal_yaw:.2f}); '
            'use RViz 2D Goal Pose for another target')

    def command_callback(self, message):
        self.command = message
        self.last_command_time = time.monotonic()

    @staticmethod
    def obstacle_points():
        # A solid rectangle blocks the straight path. The planner must pass above
        # or below it in XY; there is no z direction available.
        points = []
        for x_index in range(7):
            x = 2.2 + 0.1 * x_index
            for y_index in range(17):
                y = -0.8 + 0.1 * y_index
                points.append((x, y, 0.0))

        # Add landmarks away from the route so the occupancy view is easier to read.
        for angle_index in range(48):
            angle = 2.0 * math.pi * angle_index / 48.0
            points.append((-2.5 + 0.45 * math.cos(angle),
                           2.5 + 0.45 * math.sin(angle), 0.0))
        return points

    def update(self):
        now_steady = time.monotonic()
        dt = clamp(now_steady - self.last_update, 0.0, 0.05)
        self.last_update = now_steady

        command_fresh = (
            self.last_command_time is not None and
            now_steady - self.last_command_time <= self.command_timeout)
        vx_body = self.command.linear.x if command_fresh else 0.0
        vy_body = self.command.linear.y if command_fresh else 0.0
        target_yaw_rate = self.command.angular.z if command_fresh else 0.0
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        target_vx_world = cosine * vx_body - sine * vy_body
        target_vy_world = sine * vx_body + cosine * vy_body

        max_dv = self.max_acceleration * dt
        self.vx_world += clamp(target_vx_world - self.vx_world, -max_dv, max_dv)
        self.vy_world += clamp(target_vy_world - self.vy_world, -max_dv, max_dv)
        max_dw = self.max_yaw_acceleration * dt
        self.yaw_rate += clamp(target_yaw_rate - self.yaw_rate, -max_dw, max_dw)
        self.x += self.vx_world * dt
        self.y += self.vy_world * dt
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * dt)

        stamp = self.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        # EGO consumes map-frame planar velocities, matching thin_odom_adapter.
        odometry.twist.twist.linear.x = self.vx_world
        odometry.twist.twist.linear.y = self.vy_world
        odometry.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odometry)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        if now_steady - self.last_path_sample >= 0.08:
            pose = PoseStamped()
            pose.header = odometry.header
            pose.pose = odometry.pose.pose
            self.path.header.stamp = stamp
            self.path.poses.append(pose)
            if len(self.path.poses) > 2000:
                self.path.poses = self.path.poses[-2000:]
            self.path_pub.publish(self.path)
            self.last_path_sample = now_steady

        if not self.goal_sent and now_steady - self.started >= self.goal_delay:
            goal = PoseStamped()
            goal.header.stamp = stamp
            goal.header.frame_id = 'odom'
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            goal.pose.orientation = quaternion_from_yaw(self.goal_yaw)
            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.get_logger().info('Published automatic planar goal')

    def publish_obstacles(self):
        points = self.obstacle_points()
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b''.join(struct.pack('<fff', *point) for point in points)
        message.is_dense = True
        self.cloud_pub.publish(message)

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        obstacle = Marker()
        obstacle.header.stamp = stamp
        obstacle.header.frame_id = 'odom'
        obstacle.ns = 'planar_simulation_obstacle'
        obstacle.id = 0
        obstacle.type = Marker.CUBE
        obstacle.action = Marker.ADD
        obstacle.pose.position.x = 2.5
        obstacle.pose.position.z = 0.04
        obstacle.pose.orientation.w = 1.0
        obstacle.scale.x = 0.7
        obstacle.scale.y = 1.7
        obstacle.scale.z = 0.08
        obstacle.color.r = 0.95
        obstacle.color.g = 0.18
        obstacle.color.b = 0.08
        obstacle.color.a = 0.75
        self.obstacle_pub.publish(obstacle)

        robot = Marker()
        robot.header = obstacle.header
        robot.ns = 'planar_simulation_robot'
        robot.id = 0
        robot.type = Marker.CUBE
        robot.action = Marker.ADD
        robot.pose.position.x = self.x
        robot.pose.position.y = self.y
        robot.pose.position.z = 0.08
        robot.pose.orientation = quaternion_from_yaw(self.yaw)
        robot.scale.x = 0.55
        robot.scale.y = 0.42
        robot.scale.z = 0.16
        robot.color.r = 0.12
        robot.color.g = 0.85
        robot.color.b = 0.28
        robot.color.a = 1.0
        self.robot_pub.publish(robot)

        evidence = Marker()
        evidence.header = obstacle.header
        evidence.ns = 'planar_simulation_status'
        evidence.id = 0
        evidence.type = Marker.TEXT_VIEW_FACING
        evidence.action = Marker.ADD
        evidence.pose.position.x = self.x
        evidence.pose.position.y = self.y - 0.7
        evidence.pose.position.z = 0.05
        evidence.pose.orientation.w = 1.0
        evidence.scale.z = 0.28
        evidence.color.r = 1.0
        evidence.color.g = 1.0
        evidence.color.b = 1.0
        evidence.color.a = 1.0
        evidence.text = (
            f'holonomic sim  x={self.x:.2f}  y={self.y:.2f}  yaw={self.yaw:.2f}\n'
            f'world velocity=({self.vx_world:.2f}, {self.vy_world:.2f})')
        self.evidence_pub.publish(evidence)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarNavigationSimulator()
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
