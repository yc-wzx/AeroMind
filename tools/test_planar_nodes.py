#!/usr/bin/env python3
"""Hardware-free ROS smoke tests for AeroMind's planar interfaces."""

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from traj_utils.msg import Bspline
from geometry_msgs.msg import Point, Twist


def quaternion_from_rpy(roll, pitch, yaw):
    from geometry_msgs.msg import Quaternion
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy)


class Probe(Node):
    def __init__(self):
        super().__init__('planar_smoke_probe')
        self.cloud = None
        self.odom = None
        self.goal = None
        self.command = None
        self.cloud_pub = self.create_publisher(PointCloud2, '/test/cloud_in', 10)
        self.odom_pub = self.create_publisher(Odometry, '/test/odom_in', 10)
        self.exec_odom_pub = self.create_publisher(Odometry, '/test/executor_odom', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/test/goal_in', 10)
        self.exec_goal_pub = self.create_publisher(PoseStamped, '/test/executor_goal', 10)
        self.bspline_pub = self.create_publisher(Bspline, '/test/bspline', 10)
        self.create_subscription(PointCloud2, '/test/cloud_out', self._cloud, 10)
        self.create_subscription(Odometry, '/test/odom_out', self._odom, 10)
        self.create_subscription(PoseStamped, '/test/goal_out', self._goal, 10)
        self.create_subscription(Twist, '/test/cmd_vel', self._command, 10)

    def _cloud(self, value):
        self.cloud = value

    def _odom(self, value):
        self.odom = value

    def _goal(self, value):
        self.goal = value

    def _command(self, value):
        self.command = value


def wait_for(node, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('timed out waiting for ROS output')


def main():
    commands = [
        ['ros2', 'run', 'uav_planning', 'planar_cloud_projector', '--ros-args',
         '-p', 'input_topic:=/test/cloud_in', '-p', 'output_topic:=/test/cloud_out',
         '-p', 'obstacle_min_z:=0.1', '-p', 'obstacle_max_z:=1.0',
         '-p', 'planar_voxel_size:=0.01'],
        ['ros2', 'run', 'uav_planning', 'thin_odom_adapter', '--ros-args',
         '-p', 'input_topic:=/test/odom_in', '-p', 'output_topic:=/test/odom_out',
         '-p', 'window_size:=2', '-p', 'min_dt_sec:=0.01', '-p', 'max_dt_sec:=0.5'],
        ['ros2', 'run', 'uav_planning', 'ego_goal_adapter.py', '--ros-args',
         '-p', 'input_topic:=/test/goal_in', '-p', 'ego_goal_topic:=/test/goal_out',
         '-p', 'normalized_goal_topic:=/test/normalized_goal', '-p', 'frame_id:=odom'],
        ['ros2', 'run', 'uav_planning', 'ego_trajectory_executor.py', '--ros-args',
         '-p', 'bspline_topic:=/test/bspline', '-p', 'goal_topic:=/test/executor_goal',
         '-p', 'odometry_topic:=/test/executor_odom', '-p', 'cmd_vel_topic:=/test/cmd_vel',
         '-p', 'enable_cmd_vel_output:=true'],
    ]
    processes = [subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.PIPE, text=True)
                 for command in commands]
    rclpy.init()
    probe = Probe()
    try:
        time.sleep(1.0)

        header = Header(frame_id='odom', stamp=probe.get_clock().now().to_msg())
        cloud = point_cloud2.create_cloud_xyz32(
            header, [(1.0, 0.0, 0.05), (2.0, 0.0, 0.50), (3.0, 0.0, 1.50)])
        for _ in range(3):
            probe.cloud_pub.publish(cloud)
            rclpy.spin_once(probe, timeout_sec=0.1)
        wait_for(probe, lambda: probe.cloud is not None)
        points = list(point_cloud2.read_points(
            probe.cloud, field_names=('x', 'y', 'z'), skip_nans=True))
        assert len(points) == 1 and abs(float(points[0][0]) - 2.0) < 0.02
        assert abs(float(points[0][2])) < 1e-6

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.z = 3.0
        odom.pose.pose.orientation = quaternion_from_rpy(0.3, -0.2, 0.7)
        for index in range(4):
            odom.header.stamp = probe.get_clock().now().to_msg()
            odom.pose.pose.position.x = 0.02 * index
            probe.odom_pub.publish(odom)
            time.sleep(0.03)
            rclpy.spin_once(probe, timeout_sec=0.05)
        wait_for(probe, lambda: probe.odom is not None)
        assert abs(probe.odom.pose.pose.position.z) < 1e-9
        assert abs(probe.odom.pose.pose.orientation.x) < 1e-9
        assert abs(probe.odom.pose.pose.orientation.y) < 1e-9
        assert abs(probe.odom.twist.twist.linear.z) < 1e-9
        assert abs(probe.odom.twist.twist.angular.x) < 1e-9
        assert abs(probe.odom.twist.twist.angular.y) < 1e-9

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 1.0, 2.0, 9.0
        goal.pose.orientation = quaternion_from_rpy(0.4, -0.3, 0.6)
        for _ in range(3):
            goal.header.stamp = probe.get_clock().now().to_msg()
            probe.goal_pub.publish(goal)
            rclpy.spin_once(probe, timeout_sec=0.1)
        wait_for(probe, lambda: probe.goal is not None)
        assert abs(probe.goal.pose.position.z) < 1e-9
        assert abs(probe.goal.pose.orientation.x) < 1e-9
        assert abs(probe.goal.pose.orientation.y) < 1e-9

        executor_odom = Odometry()
        executor_odom.header.frame_id = 'odom'
        executor_odom.child_frame_id = 'base_link'
        executor_odom.pose.pose.orientation.w = 1.0
        executor_goal = PoseStamped()
        executor_goal.header.frame_id = 'odom'
        executor_goal.pose.position.y = 1.0
        executor_goal.pose.orientation.w = 1.0
        spline = Bspline()
        spline.order = 3
        spline.traj_id = 1
        spline.start_time = probe.get_clock().now().to_msg()
        spline.pos_pts = [Point(x=0.0, y=y, z=0.0) for y in (0.0, 0.3, 0.7, 1.0)]
        spline.knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
        for _ in range(5):
            executor_odom.header.stamp = probe.get_clock().now().to_msg()
            executor_goal.header.stamp = executor_odom.header.stamp
            probe.exec_odom_pub.publish(executor_odom)
            probe.exec_goal_pub.publish(executor_goal)
            probe.bspline_pub.publish(spline)
            rclpy.spin_once(probe, timeout_sec=0.1)
        wait_for(probe, lambda: probe.command is not None)
        assert probe.command.linear.y > 0.05
        assert abs(probe.command.linear.x) < 0.05
        assert abs(probe.command.angular.z) < 0.05
        print('PASS: height ROI, planar odometry, planar goal, holonomic lateral command')
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
            if process.returncode not in (0, -15):
                error = process.stderr.read() if process.stderr else ''
                if error:
                    print(error)


if __name__ == '__main__':
    main()
