#!/usr/bin/env python3
"""Test-only loss relay. Forward the received object unchanged; never replay."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ScanRelay(Node):
    def __init__(self):
        super().__init__('stage2_scan_relay')
        self.held = False
        self.received = self.forwarded = self.dropped = 0
        self.last_received_stamp = self.last_forwarded_stamp = None
        self.output = self.create_publisher(LaserScan, '/guarded_scan', qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/scan_relay/status', 100)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/scan_relay/hold', self.hold)
        self.create_timer(.1, lambda:self.emit('heartbeat'))

    def scan(self, msg):
        self.received += 1
        self.last_received_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.held:
            self.dropped += 1
            return
        self.output.publish(msg)
        self.forwarded += 1
        self.last_forwarded_stamp = self.last_received_stamp

    def emit(self, event):
        data = dict(event=event, held=self.held, received=self.received,
                    forwarded=self.forwarded, dropped=self.dropped,
                    last_received_stamp_s=self.last_received_stamp,
                    last_forwarded_stamp_s=self.last_forwarded_stamp,
                    sim_s=self.get_clock().now().nanoseconds*1e-9,
                    monotonic_ns=time.monotonic_ns(),
                    reason='controlled scan interruption experiment')
        self.status.publish(String(data=json.dumps(data, allow_nan=False)))

    def hold(self, request, response):
        self.held = bool(request.data)
        self.emit('hold' if self.held else 'resume')
        response.success = True
        response.message = 'discard incoming scans' if self.held else 'forward only newly received scans'
        return response


if __name__ == '__main__':
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
