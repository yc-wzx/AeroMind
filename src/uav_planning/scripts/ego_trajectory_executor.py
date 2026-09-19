#!/usr/bin/env python3
"""Track EGO's planar B-spline with holonomic body velocity commands."""

import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


def quaternion_from_yaw(yaw):
    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


class Spline:
    def __init__(self, points, degree, knots):
        self.points = [tuple(point) for point in points]
        self.degree = degree
        self.knots = tuple(knots)
        self.start = self.knots[self.degree]
        self.end = self.knots[len(self.points)]

    def evaluate(self, value):
        value = clamp(value, self.start, self.end)
        index = self.degree
        last_span = len(self.points) - 1
        while index < last_span and self.knots[index + 1] < value:
            index += 1
        work = [list(self.points[index - self.degree + i])
                for i in range(self.degree + 1)]
        for level in range(1, self.degree + 1):
            for i in range(self.degree, level - 1, -1):
                left = self.knots[i + index - self.degree]
                right = self.knots[i + 1 + index - level]
                denominator = right - left
                alpha = 0.0 if abs(denominator) < 1e-12 else (value - left) / denominator
                work[i] = [(1.0 - alpha) * work[i - 1][axis] + alpha * work[i][axis]
                           for axis in range(len(work[i]))]
        return tuple(work[self.degree])

    def derivative(self):
        points = []
        for index in range(len(self.points) - 1):
            denominator = self.knots[index + self.degree + 1] - self.knots[index + 1]
            scale = 0.0 if abs(denominator) < 1e-12 else self.degree / denominator
            points.append(tuple(scale * (self.points[index + 1][axis] - self.points[index][axis])
                                for axis in range(len(self.points[index]))))
        return Spline(points, self.degree - 1, self.knots[1:-1])


class EgoTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('ego_trajectory_executor')
        self.bspline_topic = self.declare_parameter('bspline_topic', '/planning/bspline').value
        self.goal_topic = self.declare_parameter('goal_topic', '/ground/planning/goal').value
        self.odom_topic = self.declare_parameter('odometry_topic', '/ground/odometry').value
        self.cmd_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.rate = float(self.declare_parameter('control_rate_hz', 30.0).value)
        self.enabled = bool(self.declare_parameter('enable_cmd_vel_output', False).value)
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.max_vx = float(self.declare_parameter('max_vel_x_mps', 1.5).value)
        self.max_vy = float(self.declare_parameter('max_vel_y_mps', 1.5).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate_rad_s', 1.5).value)
        self.kp_position = float(self.declare_parameter('position_kp', 1.2).value)
        self.kp_yaw = float(self.declare_parameter('yaw_kp', 2.0).value)
        self.odom_timeout = float(self.declare_parameter('odometry_timeout_sec', 0.3).value)
        self.plan_step = float(self.declare_parameter('planned_path_sample_sec', 0.05).value)
        if min(self.rate, self.max_vx, self.max_vy, self.max_yaw_rate,
               self.kp_position, self.kp_yaw, self.odom_timeout, self.plan_step) <= 0.0:
            raise ValueError('controller rates, limits, gains, and timeout must be positive')

        self.position_spline = None
        self.velocity_spline = None
        self.acceleration_spline = None
        self.received_steady = None
        self.elapsed_at_receive = 0.0
        self.duration = 0.0
        self.final_yaw = None
        self.odom = None
        self.odom_received_steady = None
        self.commanded_path = Path()
        self.commanded_path.header.frame_id = self.frame_id

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ground/planning/commanded_setpoint', 10)
        self.sample_pub = self.create_publisher(Float64MultiArray, '/ground/planning/trajectory_sample', 10)
        self.planned_pub = self.create_publisher(Marker, '/ground/planning/planned_trajectory', 1)
        self.path_pub = self.create_publisher(Path, '/ground/planning/commanded_path', 1)
        self.create_subscription(Bspline, self.bspline_topic, self.bspline_callback, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info(
            f'Holonomic executor: {self.bspline_topic} -> {self.cmd_topic}; '
            f'control=(vx,vy,wz), enabled={self.enabled}')

    def goal_callback(self, message):
        yaw = yaw_from_quaternion(message.pose.orientation)
        if math.isfinite(yaw):
            self.final_yaw = yaw

    def odom_callback(self, message):
        self.odom = message
        self.odom_received_steady = time.monotonic()

    def bspline_callback(self, message):
        try:
            degree = int(message.order)
            if degree < 2 or len(message.pos_pts) <= degree:
                raise ValueError('invalid order/control-point count')
            if len(message.knots) != len(message.pos_pts) + degree + 1:
                raise ValueError('invalid knot count')
            values = list(message.knots)
            for point in message.pos_pts:
                values.extend((point.x, point.y, point.z))
                if abs(point.z) > 1e-4:
                    raise ValueError('non-planar control point received')
            if not all(math.isfinite(value) for value in values):
                raise ValueError('non-finite spline data')
            if any(b < a for a, b in zip(message.knots, message.knots[1:])):
                raise ValueError('knots are not nondecreasing')
            points = [(point.x, point.y) for point in message.pos_pts]
            spline = Spline(points, degree, list(message.knots))
            if spline.end <= spline.start:
                raise ValueError('non-positive trajectory duration')
            velocity = spline.derivative()
            acceleration = velocity.derivative()
        except (ValueError, IndexError, TypeError) as error:
            self.get_logger().error(f'rejecting invalid planar B-spline: {error}')
            self.publish_stop()
            return

        now_ns = self.get_clock().now().nanoseconds
        start_ns = message.start_time.sec * 1_000_000_000 + message.start_time.nanosec
        self.elapsed_at_receive = clamp((now_ns - start_ns) / 1e9, 0.0,
                                        spline.end - spline.start)
        self.received_steady = time.monotonic()
        self.position_spline, self.velocity_spline = spline, velocity
        self.acceleration_spline = acceleration
        self.duration = spline.end - spline.start
        # A new EGO trajectory supersedes the previous one.  Reset the executed
        # trace so RViz cannot make an obsolete plan look active.
        self.commanded_path = Path()
        self.commanded_path.header.frame_id = self.frame_id
        self.publish_planned_marker()
        self.get_logger().info(
            f'accepted planar trajectory id={message.traj_id}, '
            f'duration={self.duration:.3f}s, points={len(points)}')

    def publish_stop(self):
        if self.enabled:
            self.cmd_pub.publish(Twist())

    def timer_callback(self):
        if self.position_spline is None:
            return
        if self.odom is None or time.monotonic() - self.odom_received_steady > self.odom_timeout:
            self.publish_stop()
            return
        elapsed = clamp(self.elapsed_at_receive + time.monotonic() - self.received_steady,
                        0.0, self.duration)
        parameter = self.position_spline.start + elapsed
        position = self.position_spline.evaluate(parameter)
        velocity = self.velocity_spline.evaluate(parameter)
        acceleration = self.acceleration_spline.evaluate(parameter)
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y
        current_yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        target_yaw = self.final_yaw if self.final_yaw is not None else current_yaw

        vx_world = velocity[0] + self.kp_position * (position[0] - current_x)
        vy_world = velocity[1] + self.kp_position * (position[1] - current_y)
        cosine, sine = math.cos(current_yaw), math.sin(current_yaw)
        # Independent body X/Y commands preserve the holonomic motion model.
        vx_body = clamp(cosine * vx_world + sine * vy_world, -self.max_vx, self.max_vx)
        vy_body = clamp(-sine * vx_world + cosine * vy_world, -self.max_vy, self.max_vy)
        yaw_rate = clamp(self.kp_yaw * wrap_angle(target_yaw - current_yaw),
                         -self.max_yaw_rate, self.max_yaw_rate)
        command = Twist()
        command.linear.x, command.linear.y = vx_body, vy_body
        command.angular.z = yaw_rate
        if elapsed >= self.duration and math.hypot(position[0] - current_x,
                                                    position[1] - current_y) < 0.05:
            command = Twist()
        if self.enabled:
            self.cmd_pub.publish(command)

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, self.frame_id
        pose.pose.position.x, pose.pose.position.y = position
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_yaw(target_yaw)
        self.pose_pub.publish(pose)
        sample = Float64MultiArray()
        sample.layout.dim = [MultiArrayDimension(
            label='t,duration,x,y,vx,vy,ax,ay,target_yaw,cmd_vx,cmd_vy,cmd_wz',
            size=12, stride=12)]
        sample.data = [elapsed, self.duration, *position, *velocity, *acceleration,
                       target_yaw, command.linear.x, command.linear.y, command.angular.z]
        self.sample_pub.publish(sample)
        self.commanded_path.header.stamp = stamp
        self.commanded_path.poses.append(pose)
        if len(self.commanded_path.poses) > 2000:
            self.commanded_path.poses = self.commanded_path.poses[-2000:]
        self.path_pub.publish(self.commanded_path)

    def publish_planned_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns, marker.id = 'ego_planar_trajectory', 0
        marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.07
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 1.0, 1.0
        count = max(2, int(math.ceil(self.duration / self.plan_step)) + 1)
        for index in range(count):
            value = self.position_spline.evaluate(
                self.position_spline.start + min(self.duration, index * self.plan_step))
            marker.points.append(Point(x=value[0], y=value[1], z=0.03))
        self.planned_pub.publish(marker)

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EgoTrajectoryExecutor()
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
