"""Insert a temporary box in the lane and verify lidar-guided navigation."""
import json
import math
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[1]
NAME = 'stress_obstacle'
X, Y = 6.4, 1.48
WIDTH = 0.20


def service(name, request_type, request):
    result = subprocess.run([
        'ign', 'service', '-s', '/world/competition/'+name,
        '--reqtype', request_type, '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '5000', '--req', request],
        capture_output=True, text=True, timeout=8, check=True)
    if 'true' not in result.stdout:
        raise RuntimeError(result.stdout+' '+result.stderr)


model = f'''<?xml version="1.0"?>
<sdf version="1.8"><model name="{NAME}"><static>true</static>
<pose>{X} {Y} 0.225 0 0 0</pose><link name="body">
<collision name="body"><geometry><box><size>{WIDTH} {WIDTH} 0.45</size></box></geometry></collision>
<visual name="body"><geometry><box><size>{WIDTH} {WIDTH} 0.45</size></box></geometry>
<material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse></material></visual>
</link></model></sdf>'''

with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as file:
    file.write(model)
    model_path = Path(file.name)

rclpy.init()
node = rclpy.create_node('gazebo_obstacle_validation')
state = {'odom': None}
node.create_subscription(Odometry, '/gazebo/odometry',
                         lambda m: state.update(odom=m), 30)
publisher = node.create_publisher(PoseStamped, '/goal_pose', 5)
deadline = time.monotonic()+10
while (state['odom'] is None or publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
assert state['odom'] and publisher.get_subscription_count(), 'Gazebo simulation required'

minimum_gap = float('inf')
success = False
created = False
try:
    service('create', 'ignition.msgs.EntityFactory',
            f'sdf_filename: "{model_path}"')
    created = True
    print('OBSTACLE_CREATED', flush=True)
    # Let lidar and the occupancy grid observe the new obstacle before moving.
    ready = time.monotonic()+2
    while time.monotonic() < ready:
        rclpy.spin_once(node, timeout_sec=.1)
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x, goal.pose.position.y = 4.7, 0.5
    goal.pose.orientation.z = goal.pose.orientation.w = math.sqrt(.5)
    publisher.publish(goal)
    start = time.monotonic()
    settled = None
    while time.monotonic()-start < 55:
        rclpy.spin_once(node, timeout_sec=.1)
        msg = state['odom']
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        hx = abs(math.cos(yaw))*.26+abs(math.sin(yaw))*.21
        hy = abs(math.sin(yaw))*.26+abs(math.cos(yaw))*.21
        gap = math.hypot(max(abs(p.x-X)-hx-WIDTH/2, 0),
                         max(abs(p.y-Y)-hy-WIDTH/2, 0))
        minimum_gap = min(minimum_gap, gap)
        error = math.hypot(p.x-4.7, p.y-.5)
        speed = math.hypot(v.x, v.y)
        if error < .1 and speed < .03:
            settled = time.monotonic() if settled is None else settled
            if time.monotonic()-settled > 1:
                success = True
                break
        else:
            settled = None
    report = {'success': success, 'true_goal_error_m': error,
              'minimum_obstacle_clearance_m': minimum_gap,
              'wall_seconds': time.monotonic()-start}
    print(json.dumps(report), flush=True)
    (ROOT/'docs/gazebo_obstacle_validation.json').write_text(json.dumps(report, indent=2)+'\n')
finally:
    if created:
        service('remove', 'ignition.msgs.Entity', f'name: "{NAME}" type: MODEL')
    model_path.unlink(missing_ok=True)
    node.destroy_node()
    rclpy.shutdown()

raise SystemExit(0 if success and minimum_gap > .02 else 1)
