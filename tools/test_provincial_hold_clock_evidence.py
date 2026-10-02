import json,shutil,tempfile,unittest
from pathlib import Path
from provincial_stage15_hold_clock_recheck import audit,sha
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/results/provincial_stage15_completion_20261001/hold_clock_recheck'
class HoldEvidenceTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.out=Path(self.tmp.name)/'evidence';shutil.copytree(BASE,self.out,symlinks=True);self.leg=self.out/'trial_hold_clock_01';self.path=self.leg/'trial_hold_clock_01.native.jsonl'
 def tearDown(self):self.tmp.cleanup()
 def write_rows(self,rows):
  self.path.write_text(''.join(json.dumps(x)+'\n' for x in rows));p=self.leg/'raw_data_sha256.json';h=json.loads(p.read_text());h[self.path.name]=sha(self.path);p.write_text(json.dumps(h))
 def test_original_reaudits(self):self.assertTrue(audit(self.out)['all_pass'])
 def test_hidden_active_state_during_wait_rejected(self):
  rows=[json.loads(l) for l in self.path.read_text().splitlines()];x=next(x for x in rows if x['topic']=='/ground/planning/actuation_diagnostics' and 12.<x['receive_sim_s']<13.);d=json.loads(x['data']['data']);d.update(navigation_state='active',waypoint='R0001:W00',pending_waypoints=0);x['data']['data']=json.dumps(d);self.write_rows(rows);a=audit(self.out);self.assertFalse(a['all_pass']);self.assertFalse(a['checks']['waiting_state_continuous']);self.assertTrue(a['checks']['raw_hashes'])
 def test_nonzero_final_wait_command_rejected(self):
  rows=[json.loads(l) for l in self.path.read_text().splitlines()];x=next(x for x in rows if x['topic']=='/model/omni_robot/cmd_vel' and 12.<x['receive_sim_s']<13.);x['data']['linear']['x']=.1;self.write_rows(rows);a=audit(self.out);self.assertFalse(a['all_pass']);self.assertFalse(a['checks']['waiting_final_exact_zero']);self.assertTrue(a['checks']['raw_hashes'])
 def test_missing_wait_command_interval_rejected(self):
  rows=[json.loads(l) for l in self.path.read_text().splitlines()];self.write_rows([x for x in rows if not(x['topic']=='/model/omni_robot/cmd_vel' and 12.<=x['receive_sim_s']<=12.4)]);a=audit(self.out);self.assertFalse(a['all_pass']);self.assertFalse(a['checks']['native_messages']);self.assertTrue(a['checks']['raw_hashes'])
if __name__=='__main__':unittest.main()
