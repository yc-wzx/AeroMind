import json, unittest, math
from pathlib import Path
from provincial_stage15_round_trip import ROOT, MAP, FIELD, SIDES, offline_leg, sdf_walls, GridRoute
from summarize_provincial_full_observed import native_audit
from analyze_provincial_forward_clearance import walls_from_sdf

class RoundTripTests(unittest.TestCase):
    def test_original_full_native_passes_default_direction(self):
        out=ROOT/'tools/results/provincial_stage15_full_capture_20260929';name='trial_full_capture_01'
        summary=json.loads((out/(name+'.summary.json')).read_text())
        walls=walls_from_sdf(out/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf')
        self.assertTrue(native_audit(out,summary,walls,name)['pass'])
        # Original forward evidence must not substitute for a Return trial.
        rejected=native_audit(out,summary,walls,name,expected_goal=(4.7,.5))
        self.assertFalse(rejected['pass']);self.assertFalse(rejected['checks']['one_external_goal_observed'])
    def test_both_directions_keep_existing_clearance(self):
        grid=GridRoute(MAP,clearance=.4);walls=sdf_walls();reference=json.loads(FIELD.read_text())['reference_route']
        for side,(a,b) in SIDES.items():
            with self.subTest(side=side):
                report=offline_leg(a,b,grid,walls,reference)
                self.assertAlmostEqual(report['length_m'],7.75)
                self.assertGreaterEqual(report['min_grid_clearance_m'],.4)
                self.assertGreaterEqual(report['min_sampled_body_wall_gap_m'],.08)
                self.assertEqual(tuple(report['reference_stages'][-1]),b)
        reverse=offline_leg(SIDES['return'][0],SIDES['return'][1],grid,walls,reference)
        self.assertEqual([list(p) for p in reverse['reference_stages']],[[8.7,1.8],[4.7,1.8],[4.7,.5]])

if __name__=='__main__':unittest.main()
