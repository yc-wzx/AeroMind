"""Synthetic geometry and isolated production callbacks, never live actuation."""
import json
import math
from pathlib import Path
import sys
import types
import time
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/uav_planning/scripts'))
from stage2_wall_localization import Correction, FaceMatcher, transform
from stage2_localized_interface import LocalizedInterface
from stage2_navigation_interface import Stage2NavigationInterface
from gazebo_navigation_interface import GazeboNavigationInterface
from geometry_msgs.msg import Twist
from provincial_safety_geometry import wall_box

SEGMENTS=json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())['collision_segments']


def synthetic_scan(pose, segments=SEGMENTS, noise=0.):
    """Independent ray/box intersections; artificial data, not recorded scans."""
    faces=[]
    for s in segments:
        box=wall_box(s,.055);faces.extend(zip(box,box[1:]+box[:1]))
    ranges=[];rng=np.random.default_rng(17)
    for angle in np.linspace(-math.pi, math.pi, 360):
        ray=np.array([math.cos(pose[2]+angle),math.sin(pose[2]+angle)])
        hits=[]
        for a,b in faces:
            v=np.subtract(b,a)
            matrix=np.column_stack((ray,-v))
            if abs(np.linalg.det(matrix))<1e-10:continue
            distance,along=np.linalg.solve(matrix,np.subtract(a,pose[:2]))
            if distance>.08 and 0<=along<=1:hits.append(distance)
        ranges.append(min(hits)+rng.normal(0,noise) if hits else math.inf)
    return types.SimpleNamespace(ranges=ranges,angle_min=-math.pi,angle_max=math.pi,angle_increment=2*math.pi/359,
                                 range_min=.08,range_max=16.,time_increment=0.,scan_time=0.)


class LocalizationTests(unittest.TestCase):
    def test_known_failed_poses_recovered_with_exact_faces(self):
        matcher=FaceMatcher(SEGMENTS)
        for truth,offset in (((8.2827,1.7443,1.5497),(-.011,.066,.0262)),
                             ((5.6332,1.8597,1.5695),(.003,-.0083,.00283)),
                             ((4.7,.5,math.pi/2),(.02,-.03,.015))):
            scan=synthetic_scan(truth,noise=.01)
            pose=tuple(a+b for a,b in zip(truth,offset))
            result=matcher.match(pose,scan)
            self.assertTrue(result['accepted'],result)
            corrected=np.asarray(pose)+result['delta']
            self.assertLess(math.dist(corrected[:2],truth[:2]),.006)
            self.assertLess(abs(corrected[2]-truth[2]),.005)

    def test_empty_or_nonfinite_scan_cannot_match(self):
        s=synthetic_scan((4.7,.5,math.pi/2))
        for value in (math.nan,math.inf):
            s.ranges=[value]*360;self.assertFalse(FaceMatcher(SEGMENTS).match((4.7,.5,0),s)['accepted'])

    def test_nan_metadata_rejected(self):
        s=synthetic_scan((4.7,.5,math.pi/2));s.angle_increment=math.nan
        self.assertFalse(FaceMatcher(SEGMENTS).match((4.7,.5,0),s)['accepted'])

    def test_no_weak_axis_invented_by_prior(self):
        walls=[[-100,-.4275,100,-.4275],[-100,.4275,100,.4275]]
        s=synthetic_scan((0,0,0),walls)
        self.assertFalse(FaceMatcher(walls).match((.02,.02,.01),s)['accepted'])

    def test_correction_transform_preserves_future_body_motion(self):
        c=Correction();c.update((5.,2.,.4),(.02,-.03,.01),1.)
        self.assertLess(math.dist(c.apply((5.,2.,.4)),(5.02,1.97,.41)),1e-10)
        a=c.apply((5.,2.,.4));b=c.apply((5.1,2.,.4))
        self.assertAlmostEqual(math.dist(a[:2],b[:2]),.1)

    def node(self):
        n=LocalizedInterface.__new__(LocalizedInterface)
        n.last_odom_received=17.;n.last_match_sim=None;n.last_match_steady=None;n.match_timeout=.5
        n.scan_input_fault=False
        n.active_waypoint=None;n.route_diag_id=None;n.stage_last_progress_at=None
        n.localization_diag=types.SimpleNamespace(publish=lambda _:None)
        return n

    def test_stale_correction_reuses_actual_production_watchdog(self):
        n=self.node();seen=[]
        with patch.object(LocalizedInterface,'localization_fresh',lambda _:False),patch.object(Stage2NavigationInterface,'update',lambda node:seen.append(node.last_odom_received)):
            n.update()
        self.assertEqual(seen,[None]);self.assertEqual(n.last_odom_received,17.)

    def test_fresh_correction_does_not_disable_original_watchdog(self):
        n=self.node();seen=[]
        with patch.object(LocalizedInterface,'localization_fresh',lambda _:True),patch.object(Stage2NavigationInterface,'update',lambda node:seen.append(node.last_odom_received)):
            n.update()
        self.assertEqual(seen,[17.])

    def test_target_rejected_before_match_and_forwarded_when_fresh(self):
        n=self.node()
        with patch.object(LocalizedInterface,'get_logger',lambda _:types.SimpleNamespace(warn=lambda _:None)),patch.object(Stage2NavigationInterface,'route_goal') as base:
            with patch.object(LocalizedInterface,'localization_fresh',lambda _:False):n.route_goal(object())
            self.assertFalse(base.called)
            with patch.object(LocalizedInterface,'localization_fresh',lambda _:True):n.route_goal(object())
            self.assertEqual(base.call_count,1)

    def test_real_final_output_stops_on_lost_match_and_can_resume(self):
        n=self.node();now=time.monotonic();outputs=[]
        values=dict(command=Twist(),provincial_reference_route_mode=True,route_diag_id='R0001',
                    active_waypoint=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=4.7,y=1.15))),
                    waypoints=[object()],grid_route=object(),stage_sent_at=now,final_approach_active=False,
                    last_command_time=now,command_timeout=.5,last_odom_received=now,scan_received_at=now,
                    last_drive_clock=None,imperfect_sensors=False,drive_tau=0.,drive_scale=1.,actuated=Twist(),
                    provincial_rect_guard=True,planned_path_valid_until=2.,planned_path_safe=True,
                    provincial_min_body_gap=.08,provincial_walls=[],x=4.7,y=.5,yaw=math.pi/2,latest_odom=None,
                    publish_actuation_diagnostic=lambda *args:None,
                    drive_pub=types.SimpleNamespace(publish=lambda m:outputs.append((m.linear.x,m.linear.y,m.angular.z))))
        n.__dict__.update(values);n.command.linear.x=.1
        clock=types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000))
        # Execute the real production method body; isolate only publishers,
        # clock and polygon calculation. No live ROS velocity publication.
        with patch.object(LocalizedInterface,'get_clock',lambda _:clock),patch.object(Stage2NavigationInterface,'update',GazeboNavigationInterface.update.__wrapped__),patch('gazebo_navigation_interface.predict_command_gap',lambda *args:.2):
            n.update();self.assertEqual(outputs[-1],(0.,0.,0.))
            n.last_match_sim=.99;n.last_match_steady=time.monotonic()
            n.update();self.assertEqual(outputs[-1],(.1,0.,0.))
            n.last_match_sim=.4
            n.update();self.assertEqual(outputs[-1],(0.,0.,0.))


if __name__=='__main__':unittest.main()
