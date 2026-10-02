"""Readonly counterexamples use in-memory copies of sealed native evidence."""
import copy
import json
from pathlib import Path
import unittest
from audit_scan_resume_v3 import clock_associated_loss, task_clock_audit

ROOT=Path(__file__).resolve().parents[2]
DIR=ROOT/'tools/results/stage2_scan_resume_retry_fixed_20261001'
NAME='stage2_scan_resume_retry_fixed_01'


class AuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with (DIR/(NAME+'.native.jsonl')).open() as stream:
            cls.original=[json.loads(x) for x in stream]
        cls.summary=json.loads((DIR/(NAME+'.summary.json')).read_text())
        cls.loss=clock_associated_loss(cls.original)

    def setUp(self):self.rows=copy.deepcopy(self.original)

    def post_clock(self,topic):
        return next(r for r in self.rows if r['topic']==topic and (r['receive_sim_s'] or 0)>18)

    def edit_clock(self,phase,key,value):
        for r in self.rows:
            if r['topic']!='/localization/diagnostics':continue
            d=json.loads(r['data']['data'])
            if d.get('event')=='localization_task_clock' and d.get('phase')==phase:
                d[key]=value;r['data']['data']=json.dumps(d);return
        self.fail('No task-clock event')

    def task(self):return task_clock_audit(self.rows,self.summary,self.loss)

    def test_original_prefix_handled_but_missing_scans_remain_fail(self):
        self.assertEqual(self.loss['startup_clock_association']['omitted_from_loss_window_only'],47)
        self.assertEqual(self.loss['startup_clock_association']['pre_clock_actuation_diagnostics'],8)
        self.assertFalse(self.loss['pass'])
        self.assertFalse(self.loss['checks']['every_accepted_scan_was_forwarded'])

    def test_post_clock_command_missing_time_rejected(self):
        self.post_clock('/model/omni_robot/cmd_vel')['receive_sim_s']=None
        with self.assertRaises(ValueError):clock_associated_loss(self.rows)

    def test_post_clock_diagnostic_missing_time_rejected(self):
        self.post_clock('/ground/planning/actuation_diagnostics')['receive_sim_s']=None
        with self.assertRaises(ValueError):clock_associated_loss(self.rows)

    def test_nonfinite_final_command_rejected(self):
        self.post_clock('/model/omni_robot/cmd_vel')['data']['linear']['x']=float('nan')
        with self.assertRaises(ValueError):clock_associated_loss(self.rows)

    def test_nonfinite_prefix_command_not_exempt(self):
        next(r for r in self.rows if r['topic']=='/model/omni_robot/cmd_vel')['data']['linear']['x']=float('nan')
        with self.assertRaises(ValueError):clock_associated_loss(self.rows)

    def test_missing_clock_rejected(self):
        self.rows=[r for r in self.rows if r['topic']!='/clock']
        with self.assertRaises(ValueError):clock_associated_loss(self.rows)

    def test_nonzero_hold_command_not_exempt(self):
        self.post_clock('/model/omni_robot/cmd_vel')['data']['linear']['x']=.1
        self.assertFalse(clock_associated_loss(self.rows)['checks']['exact_zero_until_new_scan'])

    def test_actual_task_clock_pass(self):self.assertTrue(self.task()['pass_'])

    def test_missing_pause_rejected(self):
        self.rows=[r for r in self.rows if not(r['topic']=='/localization/diagnostics' and
            json.loads(r['data']['data']).get('phase')=='paused')]
        self.assertFalse(self.task()['pass_'])

    def test_wrong_task_identity_rejected(self):
        self.edit_clock('resumed','route','R9999');self.assertFalse(self.task()['pass_'])

    def test_wrong_duration_rejected(self):
        self.edit_clock('resumed','hold_duration_s',.1);self.assertFalse(self.task()['pass_'])

    def test_state_event_cannot_be_scan(self):
        self.edit_clock('resumed','accepted',True);self.assertFalse(self.task()['pass_'])


if __name__=='__main__':unittest.main()
