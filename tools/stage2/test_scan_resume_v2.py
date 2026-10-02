"""Real evidence goal-parameter tests, using disposable copies only."""
import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from audit_scan_resume_v2 import native_audit, walls_from_sdf, ROOT, read

class GoalParameterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source=ROOT/'tools/results/stage2_scan_resume_20261001'
        cls.name=read(cls.source/'progress.json')['name']
        cls.summary=read(cls.source/(cls.name+'.summary.json'))
        cls.walls=walls_from_sdf(cls.source/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf')

    def evaluate(self,kind):
        with tempfile.TemporaryDirectory(prefix='scan-resume-goal-evidence-') as temporary:
            out=Path(temporary)
            for p in self.source.iterdir():
                if p.name==self.name+'.native.jsonl':continue
                (out/p.name).symlink_to(p,target_is_directory=p.is_dir())
            rows=[json.loads(l) for l in (self.source/(self.name+'.native.jsonl')).read_text().splitlines()]
            goals=[r for r in rows if r['topic']=='/goal_pose']
            if kind=='wrong':goals[0]['data']['pose']['position']['y']=4.25
            elif kind=='missing':rows=[r for r in rows if r['topic']!='/goal_pose']
            elif kind=='duplicate':rows.append(copy.deepcopy(goals[0]));rows.sort(key=lambda r:r['receive_monotonic_ns'])
            with (out/(self.name+'.native.jsonl')).open('x') as stream:
                for r in rows:stream.write(json.dumps(r)+'\n')
            result=native_audit(out,self.summary,self.walls,self.name,expected_goal=(4.7,1.15))
            return result

    def test_real_short_goal_passes(self):self.assertTrue(self.evaluate('valid')['pass'])
    def test_wrong_goal_rejected(self):self.assertFalse(self.evaluate('wrong')['checks']['one_external_goal_observed'])
    def test_missing_goal_rejected(self):self.assertFalse(self.evaluate('missing')['checks']['one_external_goal_observed'])
    def test_duplicate_goal_rejected(self):self.assertFalse(self.evaluate('duplicate')['checks']['one_external_goal_observed'])

if __name__=='__main__':unittest.main()
