import copy
import json
import math
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch
from scan_guard_fixture import ROOT, FRAME, callback_case, node, recorded_scan
from stage2_localized_interface import LocalizedInterface
from stage2_navigation_interface import Stage2NavigationInterface
from gazebo_navigation_interface import GazeboNavigationInterface
from stage2_wall_localization import verify_lidar_contract, validate_scan_metadata


class ScanGuardTests(unittest.TestCase):
    def assert_rejected_without_updates(self, mutate, hard=True):
        result=callback_case(mutate)
        self.assertFalse(result['diag']['accepted'],result)
        self.assertEqual(result['published_clouds'],0)
        self.assertFalse(result['scan_freshness_refreshed'])
        self.assertEqual(result['correction'],(0.,0.,0.))
        self.assertEqual(result['last_match_sim'],.9)
        self.assertEqual(result['scan_input_fault'],hard)

    def test_bad_angles_range_timing_empty_and_frame(self):
        cases=[lambda s:setattr(s.header,'frame_id','camera'),
               lambda s:setattr(s,'angle_increment',math.nan),
               lambda s:setattr(s,'angle_max',1.),
               lambda s:setattr(s,'angle_increment',0.),
               lambda s:setattr(s,'angle_min',math.inf),
               lambda s:setattr(s,'range_max',s.range_min),
               lambda s:setattr(s,'range_min',-.1),
               lambda s:setattr(s,'range_max',math.inf),
               lambda s:setattr(s,'scan_time',-.1),
               lambda s:setattr(s,'time_increment',.1),
               lambda s:setattr(s,'ranges',[]),
               lambda s:setattr(s.header.stamp,'nanosec',1_000_000_000)]
        for mutate in cases:
            with self.subTest(mutate=mutate):self.assert_rejected_without_updates(mutate)

    def test_stale_future_duplicate_or_reordered_do_not_refresh(self):
        def stamp(s,t):s.header.stamp.sec=int(t);s.header.stamp.nanosec=round((t-int(t))*1e9)
        for t in (.7,1.1,.9,.88):
            with self.subTest(t=t):self.assert_rejected_without_updates(lambda s:stamp(s,t),hard=False)

    def test_all_no_returns_rejected_without_refresh_or_cloud(self):
        self.assert_rejected_without_updates(lambda s:setattr(s,'ranges',[math.inf]*360),hard=False)

    def test_individual_nan_or_inf_returns_not_replaced_with_zero(self):
        def mutate(s):s.ranges[0]=math.inf;s.ranges[1]=math.nan
        r=callback_case(mutate)
        self.assertTrue(r['diag']['accepted']);self.assertEqual(r['nonfinite_cloud_points'],0)
        self.assertEqual(r['published_clouds'],1)

    def test_actual_sdf_mount_and_unsupported_changed_mount(self):
        source=ROOT/'src/uav_bringup/worlds/provincial_2025_training.sdf'
        actual=verify_lidar_contract(source)
        self.assertEqual(actual['frame_id'],FRAME)
        with self.assertRaises(ValueError):verify_lidar_contract(source,'camera')
        with tempfile.TemporaryDirectory() as d:
            import xml.etree.ElementTree as ET
            for field,value in (('x','.01'),('yaw','.01'),('relative_to','camera')):
                tree=ET.parse(source);pose=tree.find(".//model[@name='omni_robot']/link/sensor/pose")
                values=pose.text.split()
                if field=='relative_to':pose.set('relative_to',value)
                else:values[0 if field=='x' else 5]=value;pose.text=' '.join(values)
                path=Path(d)/'changed.sdf';tree.write(path)
                with self.assertRaises(ValueError):verify_lidar_contract(path)

    def test_hard_fault_stops_actual_final_output_then_fresh_scan_can_resume(self):
        n,clouds,diagnostics,outputs=node();n.scan_received_at=time.monotonic()
        clock=types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000))
        bad=recorded_scan();bad.header.frame_id='camera'
        with patch.object(LocalizedInterface,'get_clock',lambda _:clock),patch.object(LocalizedInterface,'get_logger',lambda _:types.SimpleNamespace(warn=lambda _:None)),patch.object(Stage2NavigationInterface,'update',GazeboNavigationInterface.update.__wrapped__),patch('gazebo_navigation_interface.predict_command_gap',lambda *args:.2):
            n.scan_callback(bad);n.update()
            self.assertEqual(outputs[-1],(0.,0.,0.));self.assertEqual(clouds,[])
            n.scan_callback(recorded_scan());n.update()
            self.assertEqual(outputs[-1],(.1,0.,0.));self.assertFalse(n.scan_input_fault)

    def test_matching_failure_expires_and_fresh_scan_does_not_bypass_plan(self):
        n,clouds,diagnostics,outputs=node();n.scan_received_at=time.monotonic()
        clock=types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000))
        with patch.object(LocalizedInterface,'get_clock',lambda _:clock),patch.object(LocalizedInterface,'get_logger',lambda _:types.SimpleNamespace(warn=lambda _:None)),patch.object(Stage2NavigationInterface,'update',GazeboNavigationInterface.update.__wrapped__),patch('gazebo_navigation_interface.predict_command_gap',lambda *args:.2):
            no_returns=recorded_scan();no_returns.ranges=[math.inf]*360
            n.scan_callback(no_returns);self.assertEqual(n.last_match_sim,.9)
            clock.now=lambda:types.SimpleNamespace(nanoseconds=1_500_000_000)
            n.update();self.assertEqual(outputs[-1],(0.,0.,0.))
            clock.now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000)
            good=recorded_scan();good.header.stamp.nanosec=20_000_000
            n.raw_history.append((1.02,4.7,.5,math.pi/2));n.scan_callback(good)
            clock.now=lambda:types.SimpleNamespace(nanoseconds=1_020_000_000)
            n.planned_path_safe=False;n.update();self.assertEqual(outputs[-1],(0.,0.,0.))
            n.planned_path_safe=True;n.update();self.assertEqual(outputs[-1],(.1,0.,0.))

    def test_wall_time_expiry_also_stops_when_sim_clock_paused(self):
        n,clouds,diagnostics,outputs=node();n.last_match_steady=time.monotonic()-.6
        n.scan_received_at=time.monotonic()
        clock=types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000))
        with patch.object(LocalizedInterface,'get_clock',lambda _:clock),patch.object(Stage2NavigationInterface,'update',GazeboNavigationInterface.update.__wrapped__):
            n.update();self.assertEqual(outputs[-1],(0.,0.,0.))


if __name__=='__main__':unittest.main()
