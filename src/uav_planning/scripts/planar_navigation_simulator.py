#!/usr/bin/env python3
"""Closed-loop planar simulator with a figure-based competition field."""

import json
import math
import struct
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


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


def point(x, y, z=0.0):
    result = Point()
    result.x = float(x)
    result.y = float(y)
    result.z = float(z)
    return result


class PlanarNavigationSimulator(Node):
    def __init__(self):
        super().__init__('planar_navigation_simulator')

        field_config = self.declare_parameter('field_config', '').value
        if not field_config:
            raise RuntimeError('field_config parameter is required')
        with open(field_config, 'r', encoding='utf-8') as stream:
            self.field = json.load(stream)

        start = self.field['start']
        shooting_zone = self.field['shooting_zone']
        self.goal_x = float(
            self.declare_parameter('goal_x', shooting_zone['x']).value)
        self.goal_y = float(
            self.declare_parameter('goal_y', shooting_zone['y']).value)
        self.goal_yaw = float(
            self.declare_parameter('goal_yaw', shooting_zone['yaw']).value)
        self.goal_delay = float(self.declare_parameter('goal_delay_sec', 3.0).value)
        self.max_acceleration = float(
            self.declare_parameter('max_acceleration_mps2', 2.5).value)
        self.max_yaw_acceleration = float(
            self.declare_parameter('max_yaw_acceleration_rad_s2', 4.0).value)
        self.command_timeout = float(
            self.declare_parameter('command_timeout_sec', 0.5).value)

        self.x = float(self.declare_parameter('start_x', start['x']).value)
        self.y = float(self.declare_parameter('start_y', start['y']).value)
        self.yaw = float(self.declare_parameter('start_yaw', start['yaw']).value)
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
        self.field_points = self.make_field_points()

        self.odom_pub = self.create_publisher(Odometry, '/ground/odometry', 20)
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_2d', 5)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 5)
        self.path_pub = self.create_publisher(Path, '/path', 5)
        self.field_pub = self.create_publisher(
            MarkerArray, '/ground/simulation/field', 5)
        self.robot_pub = self.create_publisher(Marker, '/ground/simulation/robot', 1)
        self.evidence_pub = self.create_publisher(
            Marker, '/ground/simulation/evidence', 1)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 20)

        self.create_timer(0.02, self.update)
        self.create_timer(0.10, self.publish_obstacles)
        self.create_timer(0.25, self.publish_field_markers)
        self.create_timer(0.10, self.publish_robot_markers)
        self.get_logger().info(
            f"Loaded {self.field['name']}: start "
            f"({self.x:.2f}, {self.y:.2f}), automatic goal "
            f"({self.goal_x:.2f}, {self.goal_y:.2f})")

    def make_field_points(self):
        result = []
        spacing = 0.05
        for x1, y1, x2, y2 in self.field['collision_segments']:
            length = math.hypot(x2 - x1, y2 - y1)
            samples = max(1, int(math.ceil(length / spacing)))
            for index in range(samples + 1):
                ratio = index / samples
                result.append((
                    x1 + (x2 - x1) * ratio,
                    y1 + (y2 - y1) * ratio,
                    0.0,
                ))
        return result

    def command_callback(self, message):
        self.command = message
        self.last_command_time = time.monotonic()

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
            self.get_logger().info('Published competition shooting-zone goal')

    def publish_obstacles(self):
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.height = 1
        message.width = len(self.field_points)
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b''.join(
            struct.pack('<fff', *field_point) for field_point in self.field_points)
        message.is_dense = True
        self.cloud_pub.publish(message)

    def base_marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_rectangle(self, namespace, marker_id, rectangle, color, z, alpha):
        marker = self.base_marker(namespace, marker_id, Marker.CUBE)
        marker.pose.position.x = float(rectangle['x'])
        marker.pose.position.y = float(rectangle['y'])
        marker.pose.position.z = z
        marker.scale.x = float(rectangle['width'])
        marker.scale.y = float(rectangle['height'])
        marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha
        return marker

    def publish_field_markers(self):
        markers = []
        arena = self.field['arena']
        floor = {
            'x': arena['width'] / 2.0,
            'y': arena['height'] / 2.0,
            'width': arena['width'],
            'height': arena['height'],
        }
        markers.append(self.publish_rectangle(
            'competition_floor', 0, floor,
            (0.20, 0.12, 0.28), -0.045, 0.88))

        for marker_id, surface in enumerate(self.field['course_surfaces']):
            markers.append(self.publish_rectangle(
                'competition_course', marker_id, surface,
                (0.48, 0.50, 0.54), -0.025, 0.82))

        markers.append(self.publish_rectangle(
            'competition_zones', 0, self.field['start'],
            (0.90, 0.08, 0.08), -0.005, 0.92))
        markers.append(self.publish_rectangle(
            'competition_zones', 1, self.field['shooting_zone'],
            (0.05, 0.30, 0.95), -0.005, 0.92))
        markers.append(self.publish_rectangle(
            'competition_zones', 2, self.field['target_zone'],
            (0.05, 0.72, 0.24), -0.005, 0.72))

        walls = self.base_marker('competition_collision_boundary', 0, Marker.LINE_LIST)
        walls.scale.x = 0.045
        walls.color.r = 0.93
        walls.color.g = 0.93
        walls.color.b = 0.95
        walls.color.a = 0.95
        for x1, y1, x2, y2 in self.field['collision_segments']:
            walls.points.extend((point(x1, y1, 0.04), point(x2, y2, 0.04)))
        markers.append(walls)

        route = self.base_marker('competition_reference_route', 0, Marker.LINE_STRIP)
        route.scale.x = 0.12
        route.color.r = 0.10
        route.color.g = 1.0
        route.color.b = 0.22
        route.color.a = 0.82
        route.points = [point(x, y, 0.06) for x, y in self.field['reference_route']]
        markers.append(route)

        for marker_id, (label, zone) in enumerate((
                ('START', self.field['start']),
                ('SHOOTING', self.field['shooting_zone']),
                ('TARGET ROBOT', self.field['target_zone']))):
            text_marker = self.base_marker('competition_labels', marker_id,
                                           Marker.TEXT_VIEW_FACING)
            text_marker.pose.position.x = float(zone['x'])
            text_marker.pose.position.y = float(zone['y'])
            text_marker.pose.position.z = 0.10
            text_marker.scale.z = 0.22
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = label
            markers.append(text_marker)

        self.field_pub.publish(MarkerArray(markers=markers))

    def publish_robot_markers(self):
        robot = self.base_marker('planar_simulation_robot', 0, Marker.CUBE)
        robot.pose.position.x = self.x
        robot.pose.position.y = self.y
        robot.pose.position.z = 0.10
        robot.pose.orientation = quaternion_from_yaw(self.yaw)
        robot.scale.x = 0.52
        robot.scale.y = 0.42
        robot.scale.z = 0.18
        robot.color.r = 1.0
        robot.color.g = 0.58
        robot.color.b = 0.04
        robot.color.a = 1.0
        self.robot_pub.publish(robot)

        evidence = self.base_marker('planar_simulation_status', 0,
                                    Marker.TEXT_VIEW_FACING)
        evidence.pose.position.x = self.x
        evidence.pose.position.y = self.y - 0.55
        evidence.pose.position.z = 0.12
        evidence.scale.z = 0.18
        evidence.color.r = 1.0
        evidence.color.g = 1.0
        evidence.color.b = 1.0
        evidence.color.a = 1.0
        evidence.text = (
            f'competition sim  x={self.x:.2f}  y={self.y:.2f}  yaw={self.yaw:.2f}\n'
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
