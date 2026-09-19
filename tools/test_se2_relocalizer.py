#!/usr/bin/env python3
"""Verify that relocalization estimates tx, ty and yaw with zero 3D freedom."""

import math
import pathlib
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger


SOURCE = [(0.2 * i, 0.3 * math.sin(0.7 * i) + 0.02 * i * i, 0.0)
          for i in range(30)]
EXPECTED = (0.6, -0.35, 0.22)


def transform(point, pose):
    x, y, _ = point
    tx, ty, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y + tx, s * x + c * y + ty, 0.0


def write_pcd(path, points):
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
        'TYPE F F F\nCOUNT 1 1 1\nWIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        'POINTS {0}\nDATA ascii\n').format(len(points))
    path.write_text(header + ''.join(f'{x} {y} {z}\n' for x, y, z in points))


class Probe(Node):
    def __init__(self):
        super().__init__('se2_relocalizer_probe')
        self.publisher = self.create_publisher(
            PointCloud2, '/test/relocalization_cloud', 10)
        self.client = self.create_client(Trigger, '/relocalize')


def main():
    map_path = pathlib.Path('/tmp/aeromind_se2_map.pcd')
    write_pcd(map_path, [transform(point, EXPECTED) for point in SOURCE])
    command = [
        'ros2', 'run', 'uav_planning', 'kiss_relocalizer', '--ros-args',
        '-p', f'map_path:={map_path}', '-p', 'cloud_topic:=/test/relocalization_cloud',
        '-p', 'min_inliers:=10', '-p', 'voxel_size_m:=0.02',
        '-p', 'max_correspondence_m:=1.0', '-p', 'max_rmse_m:=0.08',
        '-p', 'initial_x:=0.5', '-p', 'initial_y:=-0.3', '-p', 'initial_yaw:=0.18']
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE, text=True)
    rclpy.init()
    probe = Probe()
    try:
        assert probe.client.wait_for_service(timeout_sec=5.0)
        message = point_cloud2.create_cloud_xyz32(
            Header(frame_id='odom', stamp=probe.get_clock().now().to_msg()), SOURCE)
        for _ in range(5):
            probe.publisher.publish(message)
            rclpy.spin_once(probe, timeout_sec=0.1)
        future = probe.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(probe, future, timeout_sec=8.0)
        response = future.result()
        assert response is not None and response.success, response.message if response else 'no response'
        values = dict((key, float(value)) for key, value in
                      re.findall(r'(x|y|yaw)=(-?[0-9.]+)', response.message))
        assert abs(values['x'] - EXPECTED[0]) < 0.04
        assert abs(values['y'] - EXPECTED[1]) < 0.04
        assert abs(values['yaw'] - EXPECTED[2]) < 0.03
        assert ' z=' not in response.message
        print(f'PASS: constrained SE2 relocalization {response.message}')
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
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
