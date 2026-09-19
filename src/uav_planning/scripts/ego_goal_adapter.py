#!/usr/bin/env python3
"""Translate RViz goals to EGO while enforcing an SE(2) navigation state."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def quaternion_from_yaw(yaw):
    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


class EgoGoalAdapter(Node):
    def __init__(self):
        super().__init__('ego_goal_adapter')
        self.input_topic = self.declare_parameter('input_topic', '/goal_pose').value
        self.ego_topic = self.declare_parameter(
            'ego_goal_topic', '/move_base_simple/goal').value
        self.normalized_topic = self.declare_parameter(
            'normalized_goal_topic', '/ground/planning/goal').value
        self.marker_topic = self.declare_parameter(
            'goal_marker_topic', '/ground/planning/goal_marker').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.tf_timeout = float(self.declare_parameter('tf_timeout_sec', 0.2).value)
        if self.tf_timeout <= 0.0:
            raise ValueError('tf_timeout_sec must be positive')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        marker_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ego_pub = self.create_publisher(PoseStamped, self.ego_topic, qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.normalized_topic, qos)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, marker_qos)
        self.create_subscription(PoseStamped, self.input_topic, self.goal_callback, qos)
        self.get_logger().info(
            f'Planar goal adapter: {self.input_topic} -> {self.ego_topic}; '
            f'state=(x,y,yaw), frame={self.frame_id}')

    def goal_callback(self, message):
        values = (message.pose.position.x, message.pose.position.y,
                  message.pose.orientation.x, message.pose.orientation.y,
                  message.pose.orientation.z, message.pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('rejecting non-finite goal')
            return
        norm = math.sqrt(sum(value * value for value in (
            message.pose.orientation.x, message.pose.orientation.y,
            message.pose.orientation.z, message.pose.orientation.w)))
        if norm < 0.99 or norm > 1.01:
            self.get_logger().error(f'rejecting invalid goal quaternion norm {norm:.6f}')
            return

        source_frame = message.header.frame_id or self.frame_id
        transformed = message
        if source_frame != self.frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id, source_frame, Time(),
                    timeout=Duration(seconds=self.tf_timeout))
                transformed = do_transform_pose_stamped(message, transform)
            except TransformException as error:
                self.get_logger().error(
                    f'rejecting goal: cannot transform {source_frame!r} -> '
                    f'{self.frame_id!r}: {error}')
                return

        yaw = yaw_from_quaternion(transformed.pose.orientation)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = transformed.pose.position.x
        goal.pose.position.y = transformed.pose.position.y
        goal.pose.position.z = 0.0  # Message ABI field; not a planning freedom.
        goal.pose.orientation = quaternion_from_yaw(yaw)
        self.goal_pub.publish(goal)
        self.ego_pub.publish(goal)

        marker = Marker()
        marker.header = goal.header
        marker.ns = 'ground_navigation_goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.pose.position.z = 0.03  # Visualization only.
        marker.scale.x = 0.8
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.2, 0.1, 1.0
        self.marker_pub.publish(marker)
        self.get_logger().info(
            f'accepted planar goal x={goal.pose.position.x:.3f}, '
            f'y={goal.pose.position.y:.3f}, yaw={yaw:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = EgoGoalAdapter()
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
