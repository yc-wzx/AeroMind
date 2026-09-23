"""Check that a live Gazebo laser scan corrects a deliberately offset pose."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/uav_planning/scripts'))
from scan_matcher import WallScanMatcher

field = json.loads((Path(__file__).resolve().parents[1]
                    / 'src/uav_planning/config/competition_field_2025.json').read_text())
matcher = WallScanMatcher(field['collision_segments'])
rclpy.init()
node = rclpy.create_node('test_scan_matcher')
state = {}
node.create_subscription(Odometry, '/gazebo/odometry',
                         lambda m: state.update(odom=m), 10)
node.create_subscription(LaserScan, '/scan',
                         lambda m: state.update(scan=m), qos_profile_sensor_data)
deadline = time.monotonic()+10
while len(state) < 2 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
assert len(state) == 2, 'live odometry and scan required'
p = state['odom'].pose.pose.position
q = state['odom'].pose.pose.orientation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
offset = (p.x+0.055, p.y-0.045, yaw+0.012)
correction = matcher.match(offset, state['scan'])
assert correction is not None, 'matcher rejected a valid scan'
before = math.hypot(offset[0]-p.x, offset[1]-p.y)
after = math.hypot(offset[0]+correction[0]-p.x,
                   offset[1]+correction[1]-p.y)
print(json.dumps({'before_m': before, 'after_m': after,
                  'correction': correction}), flush=True)
node.destroy_node()
rclpy.shutdown()
assert after < before*0.6
