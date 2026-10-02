import sys
import math
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src/stage2_lio_sim/scripts'), str(ROOT/'tools/stage2')]
from lio_safe_navigation_interface import LioSafeNavigationInterface, _DriveGate
from lio_safety_geometry import planning_envelope, scan_points, sensor_command_gap
from localization_hold_fixture import Clock, full_node, isolated, MemoryPublisher
from scan_guard_fixture import recorded_scan
from geometry_msgs.msg import Twist


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(10.)
        self.node, self.logs = full_node(self.clock)
        self.node.__class__ = LioSafeNavigationInterface
        self.node.last_lio_received = self.node.last_match_sim = self.node.last_match_steady = 10.
        self.node.last_stage2_stamp = 10.
        self.node.sensor_reserve = .035
        self.node.pose_safety_margin = .04
        self.node.sensor_gap_mode = 'diagnostic_only'
        self.node.last_sensor_diag = -math.inf
        self.node.sensor_safety_pub = MemoryPublisher()
        self.node.safety_scan_fault = False
        self.node.safety_scan = dict(points=np.array([[0., .5]]),
            pose=(4.7, .67, math.pi/2), stamp=10., received=10.)
        self.actual = self.node.drive_pub
        self.node.drive_pub = _DriveGate(self.node, self.actual)
        self.context = isolated(self.clock, self.logs)
        self.context.__enter__()

    def tearDown(self):
        self.context.__exit__(None, None, None)

    def update(self):
        self.node.latest_odom.header.stamp = self.clock.stamp()
        self.node.update()
        m = self.actual.messages[-1]
        return (m.linear.x, m.linear.y, m.angular.z)

    def test_safe_fresh_production_update_moves(self):
        self.node.stage_sent_at = 10.
        self.assertNotEqual(self.update(), (0., 0., 0.))

    def test_shadow_near_return_does_not_claim_distance_stop(self):
        self.node.safety_scan['points'] = np.array([[0., .31]])
        self.node.stage_sent_at = 10.
        self.assertNotEqual(self.update(), (0., 0., 0.))
        import json
        self.assertTrue(json.loads(self.node.sensor_safety_pub.messages[-1].data)['raw_sensor_gap_would_stop'])

    def test_map_reserve_forces_complete_zero(self):
        self.node.estimate = (5.5, 1.84, math.pi/2)
        self.assertEqual(self.update(), (0., 0., 0.))
        self.assertEqual(self.node.last_sensor_action, 'reserved_map_gap_stop')

    def test_latest_pose_age_stop(self):
        self.node.last_stage2_stamp = 9.9
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_nonfinite_latest_pose_stop(self):
        self.node.estimate = (float('nan'), 1.8, math.pi/2)
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_plan_uses_stronger_rectangle_gap(self):
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point
        m = Marker(); m.header.frame_id = 'odom'; m.header.stamp.sec = 10
        m.points = [Point(x=5.5, y=1.83, z=0.)]
        self.node.planned_trajectory_callback(m)
        self.assertFalse(self.node.planned_path_safe)
        m.points = [Point(x=5.5, y=1.8, z=0.)]
        self.node.planned_trajectory_callback(m)
        self.assertTrue(self.node.planned_path_safe)

    def test_route_complete_and_idle_zero(self):
        self.node.active_waypoint = None
        self.node.waypoints = []
        self.assertEqual(self.update(), (0., 0., 0.))
        self.node.route_diag_id = None
        self.clock.t += .02
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_stale_scan_zero(self):
        self.node.safety_scan['stamp'] = 9.
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_scan_fault_zero(self):
        self.node.safety_scan_fault = True
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_restore_sensor_does_not_release_unsafe_plan(self):
        self.node.planned_path_safe = False
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_pending_no_active_keeps_zero(self):
        self.node.active_waypoint = None
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_stale_LIO_zero_despite_valid_scan(self):
        self.node.last_lio_received = 9.
        self.assertEqual(self.update(), (0., 0., 0.))

    def test_nonfinite_command_zero(self):
        c = Twist(); c.linear.x = float('nan')
        self.assertEqual(self.node.check_sensor_command(c).linear.x, 0.)

    def test_swept_future_collision(self):
        gap = sensor_command_gap(np.array([[.4, 0.]]), (0., 0., 0.),
                                 (0., 0., 0.), (.25, 0., 0.))
        self.assertAlmostEqual(gap, .0775)

    def test_global_origin_invariance(self):
        p = np.array([[.4, .5]])
        a = sensor_command_gap(p, (0., 0., 1.), (.01, .02, 1.), (.1, 0., .1))
        b = sensor_command_gap(p, (50., 30., 1.), (50.01, 30.02, 1.), (.1, 0., .1))
        self.assertAlmostEqual(a, b, places=12)

    def test_scan_metadata_and_NaN_rejected(self):
        scan = recorded_scan(); self.assertGreater(len(scan_points(scan)), 24)
        scan.ranges[4] = float('nan')
        with self.assertRaises(ValueError): scan_points(scan)

    def test_padding_preserves_wall_coordinates(self):
        s = [[0., 0., 1., 0.]]
        p = planning_envelope(s)
        self.assertEqual(s, [[0., 0., 1., 0.]])
        self.assertAlmostEqual(max(q[1] for q in p), .1025)
        self.assertAlmostEqual(min(q[0] for q in p), -.075)


if __name__ == '__main__':
    unittest.main(verbosity=2)
