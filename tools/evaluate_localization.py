#!/usr/bin/env python3
"""Record synchronized Gazebo truth, odometry, and trajectory reference to CSV.

Run in a sourced ROS 2 environment while the RMUC Gazebo launch is active.
Gazebo truth is used only for evaluation; it is never published back into navigation.
"""
import argparse
import bisect
import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_entry(stamp, x, y, yaw):
    return (stamp, x, y, yaw)


class LocalizationEvaluator(Node):
    def __init__(self, args):
        super().__init__('evaluate_localization')
        self.args = args
        self.histories = {
            'gt': deque(maxlen=1200),
            'odom': deque(maxlen=1200),
            'planner': deque(maxlen=1200),
        }
        self.start_stamp = None
        self.last_sample_stamp = -math.inf
        self.rows = 0
        self.missing_odom = 0
        self.missing_planner = 0
        self.position_errors = []
        self.yaw_errors = []
        self.planner_errors = []
        self.planner_yaw_errors = []
        self.gt_goal_errors = []
        self.odom_goal_errors = []
        self.output_path = os.path.abspath(args.output)
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        self.file = open(self.output_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=[
            't', 'stamp', 'ground_truth_x', 'ground_truth_y', 'ground_truth_yaw',
            'odom_x', 'odom_y', 'odom_yaw', 'error_x', 'error_y',
            'position_error', 'yaw_error', 'yaw_error_deg', 'odom_sync_delta_sec',
            'planner_setpoint_x', 'planner_setpoint_y', 'planner_setpoint_yaw',
            'planner_error_x', 'planner_error_y', 'planner_error_position',
            'planner_yaw_error', 'planner_sync_delta_sec',
            'ground_truth_goal_error', 'odom_goal_error',
        ])
        self.writer.writeheader()
        self.create_subscription(Odometry, args.gt_topic, self.gt_callback, 100)
        self.create_subscription(Odometry, args.odom_topic,
                                 lambda msg: self.odom_callback('odom', msg), 100)
        self.create_subscription(PoseStamped, args.setpoint_topic,
                                 self.setpoint_callback, 100)
        self.get_logger().info(
            f'CSV={self.output_path}; truth={args.gt_topic}; odom={args.odom_topic}; '
            f'setpoint={args.setpoint_topic}; sample rate={args.hz:g} Hz')

    def odom_callback(self, key, message):
        p = message.pose.pose.position
        self.histories[key].append(pose_entry(
            stamp_seconds(message.header.stamp), p.x, p.y,
            yaw_from_quaternion(message.pose.pose.orientation)))

    def setpoint_callback(self, message):
        p = message.pose.position
        self.histories['planner'].append(pose_entry(
            stamp_seconds(message.header.stamp), p.x, p.y,
            yaw_from_quaternion(message.pose.orientation)))

    def at_stamp(self, key, stamp):
        samples = self.histories[key]
        if not samples:
            return None
        stamps = [sample[0] for sample in samples]
        index = bisect.bisect_left(stamps, stamp)
        if 0 < index < len(samples):
            before, after = samples[index - 1], samples[index]
            span = after[0] - before[0]
            if span <= 1e-9:
                return before[1:], 0.0
            ratio = (stamp - before[0]) / span
            yaw_delta = wrap_angle(after[3] - before[3])
            pose = (before[1] + ratio * (after[1] - before[1]),
                    before[2] + ratio * (after[2] - before[2]),
                    wrap_angle(before[3] + ratio * yaw_delta))
            return pose, 0.0
        nearest = samples[0] if index == 0 else samples[-1]
        delta = stamp - nearest[0]
        if abs(delta) > self.args.max_sync_sec:
            return None
        return nearest[1:], delta

    def gt_callback(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if self.start_stamp is None:
            self.start_stamp = stamp
        period = 1.0 / self.args.hz
        if stamp - self.last_sample_stamp < period:
            return
        truth = message.pose.pose
        gx, gy = truth.position.x, truth.position.y
        gyaw = yaw_from_quaternion(truth.orientation)
        odom_match = self.at_stamp('odom', stamp)
        planner_match = self.at_stamp('planner', stamp)
        if odom_match is None:
            self.missing_odom += 1
            return
        (ox, oy, oyaw), odom_dt = odom_match
        ex, ey = gx - ox, gy - oy
        pos_error = math.hypot(ex, ey)
        yaw_error = wrap_angle(gyaw - oyaw)
        row = {
            't': f'{stamp - self.start_stamp:.3f}',
            'stamp': f'{stamp:.9f}',
            'ground_truth_x': f'{gx:.6f}',
            'ground_truth_y': f'{gy:.6f}',
            'ground_truth_yaw': f'{gyaw:.6f}',
            'odom_x': f'{ox:.6f}',
            'odom_y': f'{oy:.6f}',
            'odom_yaw': f'{oyaw:.6f}',
            'error_x': f'{ex:.6f}',
            'error_y': f'{ey:.6f}',
            'position_error': f'{pos_error:.6f}',
            'yaw_error': f'{yaw_error:.6f}',
            'yaw_error_deg': f'{math.degrees(yaw_error):.3f}',
            'odom_sync_delta_sec': f'{odom_dt:.6f}',
            'planner_setpoint_x': '',
            'planner_setpoint_y': '',
            'planner_setpoint_yaw': '',
            'planner_error_x': '',
            'planner_error_y': '',
            'planner_error_position': '',
            'planner_yaw_error': '',
            'planner_sync_delta_sec': '',
            'ground_truth_goal_error': '',
            'odom_goal_error': '',
        }
        self.position_errors.append(pos_error)
        self.yaw_errors.append(abs(yaw_error))
        if self.args.goal_x is not None:
            gt_goal = math.hypot(gx - self.args.goal_x, gy - self.args.goal_y)
            odom_goal = math.hypot(ox - self.args.goal_x, oy - self.args.goal_y)
            row['ground_truth_goal_error'] = f'{gt_goal:.6f}'
            row['odom_goal_error'] = f'{odom_goal:.6f}'
            self.gt_goal_errors.append(gt_goal)
            self.odom_goal_errors.append(odom_goal)
        if planner_match is None:
            self.missing_planner += 1
        else:
            (px, py, pyaw), planner_dt = planner_match
            pex, pey = gx - px, gy - py
            p_error = math.hypot(pex, pey)
            pyaw_error = wrap_angle(gyaw - pyaw)
            row.update({
                'planner_setpoint_x': f'{px:.6f}',
                'planner_setpoint_y': f'{py:.6f}',
                'planner_setpoint_yaw': f'{pyaw:.6f}',
                'planner_error_x': f'{pex:.6f}',
                'planner_error_y': f'{pey:.6f}',
                'planner_error_position': f'{p_error:.6f}',
                'planner_yaw_error': f'{pyaw_error:.6f}',
                'planner_sync_delta_sec': f'{planner_dt:.6f}',
            })
            self.planner_errors.append(p_error)
            self.planner_yaw_errors.append(abs(pyaw_error))
        self.writer.writerow(row)
        self.file.flush()
        self.rows += 1
        self.last_sample_stamp = stamp

    @staticmethod
    def describe(name, values):
        if not values:
            print(f'{name}: no samples')
            return
        print(f'{name}: mean={sum(values)/len(values):.4f} m, '
              f'max={max(values):.4f} m, final={values[-1]:.4f} m')

    @staticmethod
    def describe_angle(name, values):
        if not values:
            print(f'{name}: no samples')
            return
        print(f'{name}: mean={math.degrees(sum(values)/len(values)):.3f} deg, '
              f'max={math.degrees(max(values)):.3f} deg, '
              f'final={math.degrees(values[-1]):.3f} deg')

    def close(self):
        self.file.flush()
        self.file.close()
        print(f'CSV: {self.output_path}')
        print(f'samples={self.rows}; missing odometry matches={self.missing_odom}; '
              f'missing planner setpoints={self.missing_planner}')
        self.describe('truth-vs-odom position error', self.position_errors)
        self.describe_angle('truth-vs-odom yaw error', self.yaw_errors)
        self.describe('truth-vs-planner setpoint distance',
                      self.planner_errors)
        self.describe_angle('truth-vs-planner setpoint yaw error',
                            self.planner_yaw_errors)
        self.describe('ground-truth distance to goal', self.gt_goal_errors)
        self.describe('odom distance to goal', self.odom_goal_errors)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=os.path.join(
        'tools', 'results', 'localization_' +
        datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'))
    parser.add_argument('--gt-topic', default='/gazebo/odometry')
    parser.add_argument('--odom-topic', default='/ground/odometry')
    parser.add_argument('--setpoint-topic',
                        default='/ground/planning/commanded_setpoint')
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--max-sync-sec', type=float, default=0.10)
    parser.add_argument('--goal-x', type=float)
    parser.add_argument('--goal-y', type=float)
    args = parser.parse_args()
    if args.hz <= 0 or args.max_sync_sec <= 0:
        parser.error('--hz and --max-sync-sec must be positive')
    if (args.goal_x is None) != (args.goal_y is None):
        parser.error('--goal-x and --goal-y must be specified together')
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = LocalizationEvaluator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

