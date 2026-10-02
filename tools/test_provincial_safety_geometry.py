#!/usr/bin/env python3
"""Safety regression cases anchored to the provisional arena geometry."""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import (
    body_wall_gap, predict_command_gap, reference_stages, wall_box)


class ProvincialSafetyGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        field = json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())
        cls.reference = field['reference_route']
        cls.walls = [wall_box(segment,.055) for segment in field['collision_segments']]

    def test_intervening_turns_are_configured_corners(self):
        self.assertEqual(reference_stages((4.7,1.15),(5.8,1.8),self.reference),
                         [(4.7,1.8),(5.8,1.8)])
        self.assertEqual(reference_stages((5.8,1.8),(8.7,4.25),self.reference),
                         [(8.7,1.8),(8.7,4.25)])

    def test_reference_corner_has_margin_but_old_c_cut_does_not(self):
        self.assertGreaterEqual(body_wall_gap(8.7,1.8,0,self.walls),.14-1e-6)
        self.assertLess(body_wall_gap(8.5728058,1.9841582,0,self.walls),.02)

    def test_a_rotation_consumes_margin(self):
        self.assertGreater(body_wall_gap(4.7,.85,math.pi/2,self.walls),.18)
        self.assertLess(body_wall_gap(4.7,.85,.6767963,self.walls),.08)

    def test_final_command_checks_motion_toward_wall(self):
        along_corridor = predict_command_gap(8.7,2.3,0,0,.25,0,self.walls)
        toward_wall = predict_command_gap(8.7,2.3,0,.25,0,0,self.walls)
        self.assertGreaterEqual(along_corridor,.08)
        self.assertLess(toward_wall,.08)


if __name__ == '__main__':
    unittest.main()
