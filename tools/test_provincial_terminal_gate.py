#!/usr/bin/env python3
import unittest

from provincial_terminal_gate import TerminalGate, terminal_evidence_pass


def frame(gate, t, *, command=(0.0, 0.0, 0.0), complete=True,
          wall=None, error=0.02):
    wall = t if wall is None else wall
    return gate.advance(
        wall_now=wall, sim_t=t, gt_received_wall=wall,
        xy=(4.7, 1.13), goal_error=error, speed=0.0, yaw_speed=0.0,
        odom_received_wall=wall, odom_stamp=t,
        final_command_received_wall=wall,
        final_command=command, completed=complete)


class TerminalGateTests(unittest.TestCase):
    def test_complete_and_parked_for_two_plus_five_sim_seconds(self):
        gate = TerminalGate()
        results = [frame(gate, round(i*.05, 3)) for i in range(141)]
        self.assertFalse(any(results[:-1]))
        self.assertTrue(results[-1])
        self.assertEqual(gate.invalid_reasons, [])

    def test_nonzero_final_command_restarts_stop_window(self):
        gate = TerminalGate()
        for i in range(61):
            frame(gate, round(i*.05, 3))
        self.assertFalse(frame(gate, 3.05, command=(0.08, 0.0, 0.0)))
        self.assertIsNone(gate.stop_start_sim)
        self.assertFalse(frame(gate, 7.0))

    def test_gt_gap_does_not_join_two_windows(self):
        gate = TerminalGate()
        for i in range(31):
            frame(gate, round(i*.05, 3))
        gate.check_gap(2.0)
        self.assertIsNone(gate.stop_start_sim)
        self.assertIn('GT reception gap during stop window', gate.invalid_reasons)
        self.assertFalse(frame(gate, 1.55, wall=2.05))

    def test_missing_completion_never_starts_window(self):
        gate = TerminalGate()
        self.assertFalse(any(frame(gate, round(i*.05, 3), complete=False)
                             for i in range(161)))
        self.assertIsNone(gate.stop_start_sim)

    def test_false_success_and_mismatched_snapshot_fail_evidence(self):
        common = dict(
            result='terminal_pass', accepted=True, final_waypoint_id='R0001:W00',
            completion={'waypoint_id':'R0001:W00','gt_fresh':True,
                        'gt_goal_error_m':0.02},
            false_success=False, departure=False, stop_start_sim=1.0,
            observation_start_sim=3.0, invalid_reasons=[],
            snapshot={'gt_xy':[4.7,1.13], 'gt_goal_error_m':0.02,
                      'gt_speed_mps':0.0, 'gt_yaw_speed_rad_s':0.0,
                      'final_command':{'vx':0.0,'vy':0.0,'wz':0.0}},
            goal=(4.7,1.15), minimum_wall_gap=0.14, overlap_count=0)
        self.assertTrue(terminal_evidence_pass(**common))
        self.assertFalse(terminal_evidence_pass(**{**common,'false_success':True}))
        bad = {**common, 'snapshot':{**common['snapshot'],
                                   'gt_goal_error_m':0.01}}
        self.assertFalse(terminal_evidence_pass(**bad))


if __name__ == '__main__':
    unittest.main()
