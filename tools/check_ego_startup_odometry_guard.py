#!/usr/bin/env python3
"""Exercise the EGO startup odometry guard without Gazebo or the navigation interface.

Run after sourcing ROS Humble and this workspace's install/setup.bash. The
script uses isolated test odometry and writes its evidence under tools/results.
"""
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tools/results/provincial_stage15_safety_fix'


def send_odom(node, publisher, vx, vy):
    msg = Odometry()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = 4.7
    msg.pose.pose.position.y = 0.5
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = vx
    msg.twist.twist.linear.y = vy
    publisher.publish(msg)


def publish_for(node, publisher, vx, vy, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        send_odom(node, publisher, vx, vy)
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    log_path = OUT / 'ego_startup_guard_injected.log'
    summary_path = OUT / 'ego_startup_guard_injected.json'
    executable = ROOT / 'install/ego_planner/lib/ego_planner/ego_planner_node'
    config = ROOT / 'install/uav_bringup/share/uav_bringup/config/ego_provincial_2025_provisional.yaml'
    rclpy.init()
    node = Node('ego_startup_guard_probe')
    odom_pub = node.create_publisher(Odometry, '/stage15_guard_probe/odom', 10)
    goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
    with log_path.open('w') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '--params-file', str(config),
            '-p', 'use_sim_time:=false',
            '-p', 'fsm/replan_from_odom_if_diverged:=true',
            '-p', 'manager/max_vel:=0.25', '-p', 'optimization/max_vel:=0.25',
            '-p', 'manager/max_acc:=0.35', '-p', 'optimization/max_acc:=0.35',
            '-r', 'odom_world:=/stage15_guard_probe/odom',
            '-r', 'grid_map/odom:=/stage15_guard_probe/odom',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while (odom_pub.get_subscription_count() < 1 or
                   goal_pub.get_subscription_count() < 1):
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('EGO subscriptions did not become ready')
                rclpy.spin_once(node, timeout_sec=0.1)

            publish_for(node, odom_pub, 293.75, 31.25, 0.8)
            goal = PoseStamped()
            goal.header.frame_id = 'odom'
            goal.pose.position.x = 4.7
            goal.pose.position.y = 1.15
            goal.pose.orientation.w = 1.0
            goal_pub.publish(goal)
            publish_for(node, odom_pub, 293.75, 31.25, 0.4)
            publish_for(node, odom_pub, 0.0, 0.0, 1.2)
            if process.poll() is not None:
                raise RuntimeError(f'EGO died during injected guard test: {process.returncode}')
        finally:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()

    content = log_path.read_text(errors='replace')
    result = {
        'injected_twist_mps': [293.75, 31.25],
        'implausible_velocity_logged': 'Ignoring implausible EGO odometry velocity' in content,
        'goal_deferred_until_valid_odometry': 'Deferring EGO goal until first odometry' in content,
        'global_request_uses_zero_velocity': 'vel=(0.000,0.000)' in content,
        'normal_global_duration_logged': 'EGO global duration: 10.400 s' in content,
        'bad_alloc_seen': 'bad_alloc' in content,
    }
    result['pass'] = (all(result[key] for key in (
        'implausible_velocity_logged', 'goal_deferred_until_valid_odometry',
        'global_request_uses_zero_velocity', 'normal_global_duration_logged'))
        and not result['bad_alloc_seen'])
    summary_path.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
