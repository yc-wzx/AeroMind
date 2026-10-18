import copy,json,tempfile,unittest
from pathlib import Path
from tests import fixture,GimbalTests
from diagnose_timing import timeline,diagnose


class TimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.rows,cls.events=fixture()
    def test_aligned_and_complete_partition(self):
        report=timeline(self.rows,self.events,[0,0,-1])
        windows=report['phase_and_gap_windows'];out=next(w for w in windows if w['label']=='turn')
        self.assertAlmostEqual(out['imu_integral']['projected_net_deg'],45,delta=.1)
        self.assertAlmostEqual(out['lio_first_to_last_projected_deg'],45,delta=2)
        self.assertEqual(sum(w['imu_count'] for w in windows),report['partition_counts']['imu'])
        self.assertEqual(sum(w['lio_count'] for w in windows),report['partition_counts']['corrected'])
    def test_movement_before_turn_marker_is_not_hidden(self):
        events=copy.deepcopy(self.events);events[2]['receive_monotonic_ns']+=2_900_000_000
        report=timeline(self.rows,events,[0,0,-1])
        gap=next(w for w in report['phase_and_gap_windows'] if w['label']=='between_baseline_and_turn')
        turn=next(w for w in report['phase_and_gap_windows'] if w['label']=='turn')
        self.assertGreater(gap['imu_integral']['projected_net_deg'],40)
        self.assertLess(turn['imu_integral']['projected_net_deg'],2)
    def test_bad_markers_rejected(self):
        with self.assertRaises(ValueError):timeline(self.rows,self.events[:-1],[0,0,-1])
    def test_short_gap_has_null_not_zero(self):
        report=timeline(self.rows,self.events,[0,0,-1])
        gap=next(w for w in report['phase_and_gap_windows'] if w['label']=='between_baseline_and_turn')
        self.assertIsNone(gap['imu_integral']);self.assertIsNone(gap['lio_first_to_last_projected_deg'])
    def test_existing_acceptance_and_original_hashes_preserved(self):
        helper=GimbalTests();helper.rows=self.rows;helper.events=self.events
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);helper.save(root);before={p.name:p.read_bytes() for p in root.iterdir()}
            result=diagnose(root);self.assertTrue(result['original_motion_check_complete'])
            self.assertEqual(before,{p.name:p.read_bytes() for p in root.iterdir()})
            with (root/'native.jsonl').open('a') as f:f.write('\n')
            with self.assertRaises(ValueError):diagnose(root)

if __name__=='__main__':unittest.main(defaultTest='TimingTests',verbosity=2)
