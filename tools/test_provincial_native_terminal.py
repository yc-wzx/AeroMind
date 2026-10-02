"""Offline counterexamples against the actual Full verify() entry point."""
import contextlib
import hashlib
import io
import json
import math
import shutil
import tempfile
import unittest
from pathlib import Path
from summarize_provincial_full_observed import verify, ROOT

BASE=ROOT/'tools/results/provincial_stage15_full_capture_20260929'
NAME='trial_full_capture_01'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


class NativeTerminalIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        cls.out=Path(cls.tmp.name)/'evidence'
        shutil.copytree(BASE,cls.out)
        cls.restore_names=[NAME+s for s in ('.native.jsonl','.summary.json','.events.jsonl','.native.health.json')]+['raw_data_sha256.json','input_sha256_after.json']
        cls.original={n:(BASE/n).read_bytes() for n in cls.restore_names}
    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()
    def setUp(self):
        for n,data in self.original.items(): (self.out/n).write_bytes(data)
    def rows(self):
        return [json.loads(l) for l in (self.out/(NAME+'.native.jsonl')).open()]
    def save_rows(self,rows):
        # Repair copied counts/hash after intentional edits to isolate predicates.
        counts={}
        for r in rows:
            counts[r['topic']]=counts.get(r['topic'],0)+1
            r['receive_sequence']=counts[r['topic']]
        (self.out/(NAME+'.native.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
        p=self.out/(NAME+'.native.health.json');h=json.loads(p.read_text())
        h['received_counts']=h['written_counts']=counts;p.write_text(json.dumps(h))
    def mutate_gt(self,at,change):
        rows=self.rows();selected=[r for r in rows if r['topic']=='/gazebo/odometry' and abs(r['message_stamp_s']-at)<1e-8]
        self.assertEqual(len(selected),1);change(selected[0]);self.save_rows(rows)
    def mutate_cmd(self,at,change):
        rows=self.rows();r=min((r for r in rows if r['topic']=='/model/omni_robot/cmd_vel' and r['receive_sim_s'] is not None),key=lambda r:abs(r['receive_sim_s']-at))
        self.assertLess(abs(r['receive_sim_s']-at),.03);change(r);self.save_rows(rows)
    def audit(self):
        p=self.out/'raw_data_sha256.json';raw=json.loads(p.read_text())
        for n in raw:
            if (self.out/n).exists():raw[n]=sha(self.out/n)
        p.write_text(json.dumps(raw))
        with contextlib.redirect_stdout(io.StringIO()):
            return verify(self.out,verify_only=True)
    def reject_native(self):
        a=self.audit();self.assertFalse(a['all_pass']);self.assertFalse(a['checks']['native_terminal_2_plus_5_pass'])
        self.assertTrue(a['checks']['raw_files_match_closed_hashes'])
        self.assertTrue(a['checks']['pre_goal_input_snapshots_match_before_and_after'])
        return a
    def test_original_all_native_windows_pass(self):
        a=self.audit();self.assertTrue(a['all_pass'])
        w=a['native_terminal']['full_7s'];self.assertEqual(w['gt_samples'],351)
        self.assertEqual(w['final_command_samples'],350)
        self.assertAlmostEqual(w['max_gt_speed_mps'],.019072062266669257)
    def test_saved_known_counterexample_rejected(self):
        p=ROOT/'tools/results/provincial_stage15_full_capture_review_20260930/counterexample_native_speed'
        with contextlib.redirect_stdout(io.StringIO()):a=verify(p,verify_only=True)
        self.assertFalse(a['all_pass']);self.assertFalse(a['native_terminal']['first_2s']['checks']['gt_stopped'])
        self.assertTrue(a['checks']['raw_files_match_closed_hashes'])
    def test_first_two_seconds_native_gt_speed(self):
        self.mutate_gt(76.28,lambda r:r['data']['twist']['twist']['linear'].update(x=.1,y=0.))
        self.assertFalse(self.reject_native()['native_terminal']['first_2s']['pass'])
    def test_last_five_seconds_native_gt_speed(self):
        self.mutate_gt(79.,lambda r:r['data']['twist']['twist']['linear'].update(x=.1,y=0.))
        self.assertFalse(self.reject_native()['native_terminal']['last_5s']['pass'])
    def test_native_gt_yaw_speed(self):
        self.mutate_gt(79.,lambda r:r['data']['twist']['twist']['angular'].update(z=.1));self.reject_native()
    def test_native_command_linear_speed(self):
        self.mutate_cmd(79.,lambda r:r['data']['linear'].update(x=.1));self.reject_native()
    def test_native_command_yaw_speed(self):
        self.mutate_cmd(79.,lambda r:r['data']['angular'].update(z=.1));self.reject_native()
    def test_native_gt_nan(self):
        self.mutate_gt(79.,lambda r:r['data']['twist']['twist']['linear'].update(x=math.nan));self.reject_native()
    def test_native_gt_missing_velocity_field(self):
        self.mutate_gt(79.,lambda r:r['data']['twist']['twist']['linear'].pop('x'));self.reject_native()
    def test_native_gt_receive_nan(self):
        self.mutate_gt(79.,lambda r:r.update(receive_monotonic_ns=math.nan));self.reject_native()
    def test_native_gt_chunk_missing(self):
        self.save_rows([r for r in self.rows() if not(r['topic']=='/gazebo/odometry' and 79.<=r['message_stamp_s']<=79.4)])
        self.assertFalse(self.reject_native()['native_terminal']['last_5s']['checks']['gt_sim_and_receive_continuity'])
    def test_native_command_chunk_missing(self):
        self.save_rows([r for r in self.rows() if not(r['topic']=='/model/omni_robot/cmd_vel' and r['receive_sim_s'] is not None and 79.<=r['receive_sim_s']<=79.4)])
        self.reject_native()
    def test_native_odom_stale(self):
        self.save_rows([r for r in self.rows() if not(r['topic']=='/ground/odometry' and 79.<=r['message_stamp_s']<=79.7)])
        self.reject_native()
    def test_native_clock_association_not_fabricated(self):
        self.mutate_cmd(79.,lambda r:r.update(receive_sim_s=r['receive_sim_s']+.001));self.reject_native()
    def test_native_clock_chunk_missing(self):
        self.save_rows([r for r in self.rows() if not(r['topic']=='/clock' and 79.<=r['message_stamp_s']<=79.4)])
        self.reject_native()
    def test_native_goal_error(self):
        self.mutate_gt(79.,lambda r:r['data']['pose']['pose']['position'].update(x=8.9));self.reject_native()
    def test_native_drift_even_when_goal_error_valid(self):
        self.mutate_gt(79.,lambda r:r['data']['pose']['pose']['position'].update(y=4.19))
        self.assertFalse(self.reject_native()['native_terminal']['last_5s']['checks']['near_goal_no_drift'])
    def test_snapshot_position_inconsistent(self):
        p=self.out/(NAME+'.summary.json');s=json.loads(p.read_text());s['terminal_snapshot']['gt_xy'][0]+=.1;p.write_text(json.dumps(s));self.reject_native()
    def test_snapshot_nonzero_command_below_stop_limit_still_inconsistent(self):
        p=self.out/(NAME+'.summary.json');s=json.loads(p.read_text());s['terminal_snapshot']['final_command']['vx']=.01;p.write_text(json.dumps(s));self.reject_native()
    def test_native_final_completion_missing(self):
        self.save_rows([r for r in self.rows() if not(r['topic']=='/rosout/relevant' and 'completed' in r['data'].get('msg','') and 'waypoint=R0001:W02 ' in r['data'].get('msg',''))]);self.reject_native()
    def test_native_completion_wrong_route(self):
        rows=self.rows()
        for r in rows:
            if r['topic']=='/rosout/relevant' and 'completed' in r['data'].get('msg','') and 'waypoint=R0001:W02 ' in r['data']['msg']:
                r['data']['msg']=r['data']['msg'].replace('R0001','R0999')
        self.save_rows(rows);self.reject_native()
    def test_native_completion_wrong_logger(self):
        rows=self.rows()
        for r in rows:
            if r['topic']=='/rosout/relevant' and 'completed' in r['data'].get('msg','') and 'waypoint=R0001:W02 ' in r['data']['msg']:
                r['data']['name']='other_node'
        self.save_rows(rows);self.reject_native()
    def test_intermediate_waypoint_event_still_required(self):
        p=self.out/(NAME+'.events.jsonl');events=[json.loads(l) for l in p.open()]
        events=[e for e in events if not('completed' in e.get('message','') and 'waypoint=R0001:W00 ' in e['message'])]
        p.write_text(''.join(json.dumps(e)+'\n' for e in events));a=self.reject_native();self.assertFalse(a['checks']['all_internal_waypoint_events_pass'])
    def test_native_intermediate_completion_missing(self):
        self.save_rows([r for r in self.rows() if not(r['topic']=='/rosout/relevant' and 'completed' in r['data'].get('msg','') and 'waypoint=R0001:W00 ' in r['data'].get('msg',''))])
        self.reject_native()
    def test_native_plan_conflicts_with_events_and_preflight(self):
        rows=self.rows();changed=None
        for r in rows:
            if r['topic']=='/rosout/relevant' and r['data'].get('msg','').startswith('RMUC route diagnostic'):
                r['data']['msg']=r['data']['msg'].replace('(4.700,1.800)','(4.700,1.900)');changed=r['data']['msg']
        self.save_rows(rows)
        p=self.out/(NAME+'.summary.json');s=json.loads(p.read_text());s['goal_delivery']['acceptance']['message']=changed;p.write_text(json.dumps(s))
        self.reject_native()
    def test_writer_health_still_required(self):
        p=self.out/(NAME+'.native.health.json');h=json.loads(p.read_text());h.update(pass_=False);h['pass']=False;h['errors']=['injected writer failure'];p.write_text(json.dumps(h))
        a=self.audit();self.assertFalse(a['all_pass']);self.assertFalse(a['checks']['native_record_pass'])
    def test_missing_health_still_required(self):
        (self.out/(NAME+'.native.health.json')).unlink();a=self.audit();self.assertFalse(a['all_pass'])
    def test_hash_mismatch_still_required(self):
        p=self.out/'input_sha256_after.json';h=json.loads(p.read_text());h[next(iter(h))]['after']='0'*64;p.write_text(json.dumps(h))
        a=self.audit();self.assertFalse(a['all_pass']);self.assertFalse(a['checks']['pre_goal_input_snapshots_match_before_and_after'])
    def test_no_later_window_search(self):
        p=self.out/(NAME+'.summary.json');s=json.loads(p.read_text());s['stable_stop_start_sim_s']+=.02;p.write_text(json.dumps(s));self.reject_native()
    def test_verify_only_preserves_old_report(self):
        p=self.out/'mission_verification_capture_v1.json';before=sha(p);self.audit();self.assertEqual(sha(p),before)

if __name__=='__main__':unittest.main()
