"""Actual captured stationary rows; isolated copies, no ROS node or publishers."""
import copy
from pathlib import Path
import unittest
from audit_stationary_gravity import audit_rows, extract

ROOT = Path(__file__).resolve().parents[3]


class GravityEvidence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = extract(ROOT/'tools/results/stage2_step4_lio_optimization_20261002/full_instant_cloud_01')

    def mutated(self): return copy.deepcopy(self.rows)

    def test_actual_stationary_rows(self):
        self.assertTrue(audit_rows(*self.rows)['pass'])

    def test_gravity_removed(self):
        rows = self.mutated()
        for r in rows[2]: r[2:] = [0., 0., 0.]
        self.assertFalse(audit_rows(*rows)['pass'])

    def test_single_bad_gravity_not_hidden_in_average(self):
        rows = self.mutated(); rows[2][500][2:] = [0., 0., 0.]
        self.assertFalse(audit_rows(*rows)['pass'])

    def test_nonfinite_measurement(self):
        rows = self.mutated(); rows[2][500][2] = float('nan')
        self.assertFalse(audit_rows(*rows)['pass'])

    def test_missing_IMU_segment(self):
        rows = self.mutated(); del rows[2][500:550]
        self.assertFalse(audit_rows(*rows)['pass'])

    def test_missing_window_boundary(self):
        rows = self.mutated(); del rows[2][-100:]
        self.assertFalse(audit_rows(*rows)['pass'])

    def test_moving_GT_is_not_gravity_window(self):
        rows = self.mutated(); rows[0][100][4] = .1
        self.assertFalse(audit_rows(*rows)['pass'])

    def test_dynamic_physics_is_not_gravity_window(self):
        rows = self.mutated(); rows[1][500][5] = .1
        self.assertFalse(audit_rows(*rows)['pass'])

    def test_goal_inside_window(self):
        rows = list(self.mutated()); rows[3] = 12.
        self.assertFalse(audit_rows(*rows)['pass'])


if __name__ == '__main__': unittest.main()
