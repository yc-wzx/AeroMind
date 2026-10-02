"""Isolated real ROS-message callbacks; publishers are in-memory only."""
import copy
from collections import deque
import json
import math
from pathlib import Path
import struct
import sys
import time
import types
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from stage2_localized_interface import LocalizedInterface
from stage2_navigation_interface import Stage2NavigationInterface
from gazebo_navigation_interface import GazeboNavigationInterface
from stage2_wall_localization import Correction, FaceMatcher

FRAME='omni_robot/base_link/lidar'
FIELD=json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())


def recorded_scan():
    path=ROOT/'tools/results/stage2_localization_20261001/zero/stage2_localized_zero_01.native.jsonl'
    with path.open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['topic']=='/scan':break
    data=row['data'];s=LaserScan();s.header.stamp.sec=1;s.header.frame_id=FRAME
    for k,v in data.items():
        if k!='header':setattr(s,k,v)
    return s


def node():
    n=LocalizedInterface.__new__(LocalizedInterface);now=time.monotonic();clouds=[];diags=[];outputs=[]
    pose=(4.7,.5,math.pi/2)
    n.__dict__.update(raw_history=deque([(1.,*pose)],maxlen=150),correction=Correction(),
        last_match_sim=.9,last_match_steady=now,last_scan_stamp=.9,matcher=FaceMatcher(FIELD['collision_segments']),
        gain=.5,match_timeout=.5,scan_frame=FRAME,scan_input_fault=False,
        localization_diag=types.SimpleNamespace(publish=lambda m:diags.append(json.loads(m.data))),
        odom_history=deque([(1.,*pose)],maxlen=150),imperfect_sensors=False,grid_route=None,
        scan_matcher=None,prior_points=[],field_points=[],scan_received=False,scan_received_at=None,
        dynamic_obstacle_points=[],dynamic_scan_received_at=None,
        scan_cloud_pub=types.SimpleNamespace(publish=lambda m:clouds.append(copy.deepcopy(m))),
        command=Twist(),provincial_reference_route_mode=True,route_diag_id='R0001',
        active_waypoint=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=4.7,y=1.15))),
        waypoints=[object()],stage_sent_at=now,final_approach_active=False,
        last_command_time=now,command_timeout=.5,last_odom_received=now,
        last_drive_clock=None,drive_tau=0.,drive_scale=1.,actuated=Twist(),provincial_rect_guard=True,
        planned_path_valid_until=2.,planned_path_safe=True,provincial_min_body_gap=.08,provincial_walls=[],
        x=4.7,y=.5,yaw=math.pi/2,latest_odom=None,
        publish_actuation_diagnostic=lambda *args:None,
        drive_pub=types.SimpleNamespace(publish=lambda m:outputs.append((m.linear.x,m.linear.y,m.angular.z))))
    n.command.linear.x=.1
    return n,clouds,diags,outputs


def callback_case(mutate=lambda s:None):
    n,clouds,diags,outputs=node();s=recorded_scan();mutate(s)
    clock=types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000))
    with patch.object(LocalizedInterface,'get_clock',lambda _:clock),patch.object(LocalizedInterface,'get_logger',lambda _:types.SimpleNamespace(warn=lambda s:None,error=lambda s:None)):
        n.scan_callback(s)
    bad=sum(not all(math.isfinite(v) for v in struct.unpack_from('<fff',c.data,i)) for c in clouds for i in range(0,len(c.data),12))
    return {'diag':diags[-1], 'published_clouds':len(clouds),'nonfinite_cloud_points':bad,
            'scan_freshness_refreshed':n.scan_received_at is not None,'correction':n.correction.offset,
            'last_match_sim':n.last_match_sim,'scan_input_fault':getattr(n,'scan_input_fault',None)}
