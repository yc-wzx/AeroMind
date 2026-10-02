"""Full production update with ROS objects and memory-only transport.

No rclpy.init(), executor, graph, service or actual publishing is used.
The actual inherited update, task methods and geometry guards remain intact.
"""
import copy
import json
import math
from collections import deque
from contextlib import contextmanager
from types import SimpleNamespace as NS
from unittest.mock import patch
from scan_guard_fixture import node, ROOT, FIELD
from stage2_localized_interface import LocalizedInterface
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from builtin_interfaces.msg import Time
from planar_navigation_simulator import quaternion_from_yaw
from provincial_safety_geometry import wall_box


class MemoryPublisher:
    def __init__(self):self.messages=[]
    def publish(self, message):self.messages.append(copy.deepcopy(message))


class Clock:
    def __init__(self, now=10.):self.t=now
    def stamp(self):
        ns=round(self.t*1e9);return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
    def now(self):return NS(nanoseconds=round(self.t*1e9),to_msg=self.stamp)


def goal(x=4.7,y=1.15):
    p=PoseStamped();p.header.frame_id='odom';p.pose.position.x=x;p.pose.position.y=y
    p.pose.orientation=quaternion_from_yaw(math.pi/2);return p


def full_node(clock):
    n,*_=node();logs=[]
    n.__dict__.update(
       route_diag_id='R0001',active_waypoint_diag_id='R0001:W00',
       active_waypoint=goal(),waypoints=[goal(y=1.8)],
       waypoint_diag_ids={(4.7,1.15):'R0001:W00',(4.7,1.8):'R0001:W01'},
       grid_route=NS(clearance_required=.4,line_safe=lambda *a,**k:True),
       stage_sent_at=1.,stage_last_progress_at=1.,stage_best_distance=.48,stage_retry_count=0,
       estimated_velocity=(0.,0.,0.),estimate=(4.7,.67,math.pi/2),x=4.7,y=.67,yaw=math.pi/2,
       last_odom_stamp=Time(),latest_odom=Odometry(),
       odom_pub=MemoryPublisher(),route_pub=MemoryPublisher(),drive_pub=MemoryPublisher(),
       actuation_diag_pub=MemoryPublisher(),path_pub=MemoryPublisher(),
       tf_broadcaster=NS(sendTransform=lambda m:None),path=Path(),
       last_path_sample=clock.t,ready_since=0.,auto_goal=False,
       scan_received=True,scan_received_at=clock.t,last_command_time=clock.t,last_odom_received=clock.t,
       last_actuation_state=None,last_actuation_diag=-100.,last_blocked_goal_check=0.,
       last_blocked_goal_warning=0.,last_final_approach_diagnostic=clock.t,
       planned_path_valid_until=1000.,planned_path_safe=True,planned_path_min_gap=.19,
       provincial_wall_thickness=.055,provincial_walls=[wall_box(s,.055) for s in FIELD['collision_segments']],
       dynamic_goal_clearance=.4,dynamic_scan_received_at=None,
       localization_diag=MemoryPublisher(),last_match_sim=1.,last_match_steady=1.,scan_input_fault=False,
       scan_frame='omni_robot/base_link/lidar')
    # Retain the real diagnostic publisher method, instead of the short fixture's stub.
    del n.__dict__['publish_actuation_diagnostic']
    n.command.linear.x=.1
    n.latest_odom.header.frame_id='odom';n.latest_odom.child_frame_id='base_link'
    n.latest_odom.pose.pose.position.x=4.7;n.latest_odom.pose.pose.position.y=.67
    n.latest_odom.pose.pose.orientation=quaternion_from_yaw(math.pi/2)
    return n,logs


@contextmanager
def isolated(clock, logs):
    logger=NS(info=lambda m:logs.append(('info',m)),warn=lambda m:logs.append(('warn',m)),
              error=lambda m:logs.append(('error',m)))
    with patch('time.monotonic',lambda:clock.t),patch.object(LocalizedInterface,'get_clock',lambda _:clock),\
         patch.object(LocalizedInterface,'get_logger',lambda _:logger):
        yield


def tick(n,clock,t,fresh=False):
    clock.t=t
    # Fresh raw odometry and upstream nonzero commands, independent of scan status.
    n.last_odom_received=n.last_command_time=t;n.scan_received_at=t
    n.latest_odom.header.stamp=clock.stamp()
    if fresh:
        n.last_match_sim=n.last_match_steady=t;n.scan_input_fault=False
    n.update()


def snapshot(n,logs):
    return dict(route_id=n.route_diag_id,waypoint_id=n.active_waypoint_diag_id,
      pending=len(n.waypoints),retry_count=n.stage_retry_count,
      internal_publications=[dict(x=m.pose.position.x,y=m.pose.position.y,
                                  stamp_s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9) for m in n.route_pub.messages],
      final_output=[dict(vx=m.linear.x,vy=m.linear.y,wz=m.angular.z) for m in n.drive_pub.messages],
      stage_sent_at=n.stage_sent_at,stage_last_progress_at=n.stage_last_progress_at,
      odom_publications=len(n.odom_pub.messages),
      diagnostics=[json.loads(m.data) for m in n.actuation_diag_pub.messages],logs=logs,
      isolation='Actual production methods; memory-only transport and controlled test clock; no ROS executor or live input')


def counterexample():
    clock=Clock();n,logs=full_node(clock)
    with isolated(clock,logs):tick(n,clock,10.)
    return snapshot(n,logs)
