"""Exercise repeated Gazebo goals and score against independent truth odometry."""
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[1]
FIELD = json.loads((ROOT/'src/uav_planning/config/competition_field_2025.json').read_text())
GOALS = [
    ('middle', 6.7, 1.8),
    ('start_1', 4.7, 0.5),
    ('shoot_1', 8.7, 4.25),
    ('start_2', 4.7, 0.5),
    ('shoot_2', 8.7, 4.25),
]


def clearance(x, y, yaw):
    hx = abs(math.cos(yaw))*.26+abs(math.sin(yaw))*.21
    hy = abs(math.sin(yaw))*.26+abs(math.cos(yaw))*.21
    best = float('inf')
    for x1, y1, x2, y2 in FIELD['collision_segments']:
        if x1 == x2:
            dx = max(abs(x-x1)-hx-.0275, 0)
            dy = max(min(y1,y2)-y-hy, y-max(y1,y2)-hy, 0)
        else:
            dx = max(min(x1,x2)-x-hx, x-max(x1,x2)-hx, 0)
            dy = max(abs(y-y1)-hy-.0275, 0)
        best = min(best, math.hypot(dx, dy))
    return best


rclpy.init()
node = rclpy.create_node('gazebo_stress_validation')
latest = {'truth': None, 'estimate': None}
samples = 0
overlaps = 0
min_clearance = float('inf')
localization_errors = []


def estimate(message):
    latest['estimate'] = message


def truth(message):
    global samples, overlaps, min_clearance
    latest['truth'] = message
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    gap = clearance(p.x, p.y, yaw)
    min_clearance = min(min_clearance, gap)
    overlaps += gap == 0
    samples += 1
    if latest['estimate']:
        ep = latest['estimate'].pose.pose.position
        localization_errors.append(math.hypot(p.x-ep.x, p.y-ep.y))


node.create_subscription(Odometry, '/gazebo/odometry', truth, 30)
node.create_subscription(Odometry, '/ground/odometry', estimate, 30)
publisher = node.create_publisher(PoseStamped, '/goal_pose', 5)
deadline = time.monotonic()+15
while (latest['truth'] is None or latest['estimate'] is None
       or publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
assert latest['truth'] and latest['estimate'] and publisher.get_subscription_count(), \
    'Start the Gazebo simulation before running this test'

legs = []
for label, x, y in GOALS:
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x, goal.pose.position.y = x, y
    goal.pose.orientation.z = goal.pose.orientation.w = math.sqrt(.5)
    publisher.publish(goal)
    print('GOAL', label, flush=True)
    started = time.monotonic()
    settled = None
    success = False
    while time.monotonic()-started < 60:
        rclpy.spin_once(node, timeout_sec=.1)
        message = latest['truth']
        p, v = message.pose.pose.position, message.twist.twist.linear
        error = math.hypot(p.x-x, p.y-y)
        speed = math.hypot(v.x, v.y)
        if error < .1 and speed < .03:
            settled = time.monotonic() if settled is None else settled
            if time.monotonic()-settled > 1.0:
                success = True
                break
        else:
            settled = None
    leg = {'goal': label, 'success': success, 'error_m': error,
           'speed_mps': speed, 'wall_seconds': time.monotonic()-started,
           'true_position': [p.x, p.y]}
    legs.append(leg)
    print(json.dumps(leg), flush=True)
    if not success:
        break

ordered = sorted(localization_errors)
report = {'legs': legs, 'samples': samples, 'overlap_samples': overlaps,
          'minimum_body_clearance_m': min_clearance,
          'localization_error_m': {
              'median': statistics.median(ordered),
              'p95': ordered[int(.95*(len(ordered)-1))],
              'maximum': ordered[-1]}}
print(json.dumps(report, indent=2), flush=True)
(ROOT/'docs/gazebo_stress_validation.json').write_text(json.dumps(report, indent=2)+'\n')
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if len(legs) == len(GOALS)
                 and all(leg['success'] for leg in legs) and overlaps == 0
                 and report['localization_error_m']['p95'] < .10 else 1)
