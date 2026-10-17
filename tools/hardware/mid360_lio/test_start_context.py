"""Actual ROS Humble/DDS regression tests for production live_ready().

Synthetic sensor messages only; run in a separate ROS domain. No motor commands.
"""
import math
import os
import struct
import threading
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField

from start import live_ready, POINTS, IMU_SI, ODOM


def stamp(msg, ns):
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)


def cloud():
    ns = time.time_ns()
    msg = PointCloud2()
    stamp(msg, ns)
    msg.header.frame_id = 'livox_frame'
    msg.height, msg.width = 1, 2
    msg.point_step, msg.row_step = 32, 64
    msg.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in
                  [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),
                   ('tag',16,2),('line',17,2),('timestamp',24,8)]]
    data = bytearray(64)
    for i in range(2):
        struct.pack_into('<ffffBB', data, i*32, 2., float(i), 0., 1., 0x10, i)
        struct.pack_into('<d', data, i*32+24, float(ns + i*50_000_000))
    msg.data = bytes(data)
    return msg


class Sensors:
    def __init__(self, *, bad_imu=False, duplicate=False, wrong_source=False):
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.driver = Node('wrong_driver' if wrong_source else 'mid360_real_driver', context=self.context)
        self.adapter = Node('mid360_imu_unit_adapter', context=self.context)
        self.nodes = [self.driver, self.adapter]
        for node in self.nodes:
            self.executor.add_node(node)
        self.points = self.driver.create_publisher(PointCloud2, POINTS, qos_profile_sensor_data)
        self.imu = self.adapter.create_publisher(Imu, IMU_SI, qos_profile_sensor_data)
        self.driver.create_timer(.1, lambda: self.points.publish(cloud()))
        def publish_imu():
            msg = Imu()
            stamp(msg, time.time_ns())
            msg.header.frame_id = 'mid360_imu'
            msg.linear_acceleration.z = math.nan if bad_imu else -9.80665
            self.imu.publish(msg)
        self.adapter.create_timer(.005, publish_imu)
        self.odom = self.driver.create_publisher(Odometry, ODOM, qos_profile_sensor_data) if duplicate else None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.spin)
        self.thread.start()

    def spin(self):
        while not self.stop.is_set():
            self.executor.spin_once(timeout_sec=.01)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.stop.set()
        self.thread.join(timeout=5)
        assert not self.thread.is_alive()
        for node in self.nodes:
            self.executor.remove_node(node)
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown(context=self.context)


class StartContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '217' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Synthetic tests require private ROS_DOMAIN_ID=217 and ROS_LOCALHOST_ONLY=1')

    def test_ready_uses_production_executor_with_default_context_uninitialized(self):
        self.assertFalse(rclpy.ok())
        with Sensors(), patch('rclpy.get_global_executor', side_effect=AssertionError('global executor forbidden')):
            for _ in range(2):
                report = live_ready(timeout_s=5.)
                self.assertTrue(report['pass_'])
                self.assertGreaterEqual(report['counts'][POINTS],5)
                self.assertGreaterEqual(report['counts'][IMU_SI],20)
                self.assertFalse(report['timestamps_modified'])
                self.assertFalse(rclpy.ok())

    def test_missing_inputs_fail_cleanly(self):
        with patch('rclpy.get_global_executor', side_effect=AssertionError('global executor forbidden')):
            with self.assertRaisesRegex(RuntimeError, 'Live raw/SI inputs not ready'):
                live_ready(timeout_s=.2)
        self.assertFalse(rclpy.ok())

    def test_duplicate_lio_publisher_still_rejected(self):
        with Sensors(duplicate=True):
            with self.assertRaisesRegex(RuntimeError, 'publisher already exists'):
                live_ready(timeout_s=3.)
        self.assertFalse(rclpy.ok())

    def test_bad_imu_still_rejected(self):
        with Sensors(bad_imu=True):
            with self.assertRaisesRegex(RuntimeError, 'nonfinite IMU'):
                live_ready(timeout_s=1.5)
        self.assertFalse(rclpy.ok())

    def test_wrong_publisher_still_rejected(self):
        with Sensors(wrong_source=True):
            with self.assertRaisesRegex(RuntimeError, 'Live raw/SI inputs not ready'):
                live_ready(timeout_s=1.5)
        self.assertFalse(rclpy.ok())


if __name__ == '__main__':
    unittest.main(verbosity=2)
