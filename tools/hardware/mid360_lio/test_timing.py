import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from inspect_timing import analyze_rows, inspect


def sample(index, stamp):
    return dict(sequence=index,stamp_ns=stamp*10**6,
        receive_wall_ns=10000000000+index*10000000,
        receive_monotonic_ns=1000000000+index*10000000,error=None,
        position=[0.,0.,0.],quaternion=[0.,0.,0.,1.],velocity=[0.]*6,
        pose_covariance=[0.]*36,twist_covariance=[0.]*36)


class TimingTests(unittest.TestCase):
    def test_mixed_phases_remain_nonmonotonic(self):
        rows=[sample(i+1,t) for i,t in enumerate([100,105,102,110,115,112,120])]
        result=analyze_rows(rows)
        self.assertEqual(result['adjacent_regressed_stamps'],2)
        self.assertEqual(result['stamps_below_previous_high_watermark'],2)
        self.assertFalse(result['timestamp_strictly_increasing'])
        self.assertAlmostEqual(result['backward_jump_s']['max'],.003)
        self.assertEqual([v['sequence'] for v in result['regression_details']],[3,6])
        self.assertEqual([r['stamp_ns'] for r in rows],[t*10**6 for t in [100,105,102,110,115,112,120]])

    def test_duplicates_remain_invalid(self):
        result=analyze_rows([sample(1,100),sample(2,100)])
        self.assertEqual(result['duplicate_stamps'],1)
        self.assertFalse(result['timestamp_strictly_increasing'])

    def test_missing_nonfinite_and_receive_regressions_rejected(self):
        for case in ['missing','nan','receipt','sequence','quaternion','stamp']:
            rows=[sample(1,100),sample(2,110)]
            if case=='missing':del rows[1]['velocity']
            elif case=='nan':rows[1]['position'][0]=float('nan')
            elif case=='receipt':rows[1]['receive_monotonic_ns']=1
            elif case=='sequence':rows[1]['sequence']=5
            elif case=='quaternion':rows[1]['quaternion']=[0.]*4
            elif case=='stamp':rows[1]['stamp_ns']=None
            with self.subTest(case=case),self.assertRaises(ValueError):analyze_rows(rows)

    def fixture(self,root):
        rows=[sample(1,100),sample(2,110),sample(3,105)]
        raw=root/'odometry.jsonl'
        raw.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        saved=dict(scope='REAL SENSOR-LOCAL STATIC LIO DIAGNOSTIC; NOT NAVIGATION ACCEPTANCE',
            evidence_complete=False,stream=dict(count=3,regressed_stamps=1,duplicate_stamps=0),
            health=dict(received=3,written=3,actual_lines=3,overflow=0,error=None,closed=True),
            data_sha256={'odometry.jsonl':hashlib.sha256(raw.read_bytes()).hexdigest()})
        path=root/'summary.json';path.write_text(json.dumps(saved))
        return path,raw,saved

    def test_inspection_preserves_original_verdict_and_files(self):
        with tempfile.TemporaryDirectory() as folder:
            path,raw,_=self.fixture(Path(folder));before=(path.read_bytes(),raw.read_bytes())
            result=inspect(path)
            self.assertFalse(result['original_evidence_complete'])
            self.assertTrue(result['original_hashes_verified'])
            self.assertEqual(result['status'],'TIMESTAMP ORDERING BLOCKED')
            self.assertEqual(before,(path.read_bytes(),raw.read_bytes()))

    def test_hash_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path,raw,_=self.fixture(Path(folder));raw.write_text(raw.read_text()+'\n')
            with self.assertRaisesRegex(ValueError,'SHA-256 mismatch'):inspect(path)

    def test_inconsistent_summary_and_writer_rejected(self):
        for case in ['summary','health']:
            with self.subTest(case=case),tempfile.TemporaryDirectory() as folder:
                path,_,saved=self.fixture(Path(folder))
                if case=='summary':saved['stream']['regressed_stamps']=0
                else:saved['health']['written']=2
                path.write_text(json.dumps(saved))
                with self.assertRaises(ValueError):inspect(path)

    def test_zero_twist_does_not_become_acceptance(self):
        result=analyze_rows([sample(1,100),sample(2,110)])
        self.assertTrue(result['all_recorded_twist_components_zero'])
        self.assertNotIn('pass',result)
        self.assertNotIn('evidence_complete',result)


if __name__=='__main__':unittest.main(verbosity=2)
