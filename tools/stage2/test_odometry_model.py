import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/uav_planning/scripts'))
from stage2_odometry_model import ErrorParameters, OdometryErrorModel, wrap


class ModelTests(unittest.TestCase):
    def test_zero_is_exact_identity_even_turning(self):
        m = OdometryErrorModel(ErrorParameters())
        for i in range(100):
            pose = (4.7+math.sin(i*.04), .5+i*.03, wrap(i*.07))
            velocity = (.2, .1, .3)
            self.assertEqual(m.advance(i*.02, pose, velocity), (pose, velocity))

    def test_scale_accumulates_in_body_axes_at_yaw90(self):
        m = OdometryErrorModel(ErrorParameters(scale_x=1.005, scale_y=.995))
        m.advance(0, (4.7, .5, math.pi/2), (0, 0, 0))
        for i in range(1, 101):
            pose, velocity = m.advance(i*.02, (4.7, .5+i*.01, math.pi/2), (.5, 0, 0))
        self.assertAlmostEqual(pose[1], 1.505)
        self.assertAlmostEqual(velocity[0], .5025)

    def test_stationary_yaw_bias_accumulates(self):
        m = OdometryErrorModel(ErrorParameters(yaw_bias_rad_s=.0005))
        for i in range(101):
            pose, _ = m.advance(i*.02, (4.7, .5, math.pi/2), (0, 0, 0))
        self.assertAlmostEqual(wrap(pose[2]-math.pi/2), .001)

    def test_seed_reproducible_without_reinitializing_drift(self):
        p = ErrorParameters(position_walk_m_sqrt_s=.002, yaw_walk_rad_sqrt_s=.0003)
        a, b = OdometryErrorModel(p), OdometryErrorModel(p)
        outputs = []
        for i in range(101):
            x = a.advance(i*.02, (4.7, .5, math.pi/2), (0, 0, 0))
            self.assertEqual(x, b.advance(i*.02, (4.7, .5, math.pi/2), (0, 0, 0)))
            outputs.append(x[0])
        self.assertNotEqual(outputs[-1], outputs[0])

    def test_duplicate_does_not_mutate_state_or_random_sequence(self):
        m = OdometryErrorModel(ErrorParameters(position_walk_m_sqrt_s=.002))
        m.advance(0, (0, 0, 0), (0, 0, 0))
        before = (m.pose, m.rng.getstate(), m.elapsed_s)
        with self.assertRaises(ValueError): m.advance(0, (1, 1, 0), (0, 0, 0))
        self.assertEqual(before, (m.pose, m.rng.getstate(), m.elapsed_s))

    def test_clock_rollback_latches_no_truth_reset(self):
        m = OdometryErrorModel(ErrorParameters())
        m.advance(1, (4.7, .5, 0), (0, 0, 0))
        with self.assertRaises(ValueError): m.advance(.9, (0, 0, 0), (0, 0, 0))
        with self.assertRaises(ValueError): m.advance(1.02, (4.7, .5, 0), (0, 0, 0))
        self.assertEqual(m.pose, (4.7, .5, 0))

    def test_source_gap_latches(self):
        m = OdometryErrorModel(ErrorParameters())
        m.advance(0, (0, 0, 0), (0, 0, 0))
        with self.assertRaises(ValueError): m.advance(.21, (0, 0, 0), (0, 0, 0))
        self.assertEqual(m.fault, 'source gap')

    def test_nonfinite_is_rejected(self):
        m = OdometryErrorModel(ErrorParameters())
        for pose in ((math.nan, 0, 0), (0, math.inf, 0)):
            with self.assertRaises(ValueError): m.advance(0, pose, (0, 0, 0))
        self.assertIsNone(m.pose)

    def test_invalid_parameters_rejected(self):
        for options in ({'scale_x':0}, {'yaw_bias_rad_s':math.nan},
                        {'position_walk_m_sqrt_s':-.01}, {'max_source_gap_s':0}):
            with self.assertRaises(ValueError): ErrorParameters(**options)


if __name__ == '__main__':
    unittest.main()
