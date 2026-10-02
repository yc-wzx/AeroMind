#!/usr/bin/env python3
"""Adversarial mutations of copied raw input/output records; originals untouched."""
import copy,json,unittest
from pathlib import Path
from audit_input_history import audit_rows,RAW,EVENT,OUT
ROOT=Path(__file__).resolve().parents[3]
NATIVE=ROOT/'tools/results/stage2_step4_lio_live_noise_model_v2_20261002/full_01/lio_full_01.native.jsonl'

class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=[r for r in (json.loads(l) for l in NATIVE.open()) if r['topic'] in [RAW,EVENT,OUT]]
        cls.initial=next(i for i,r in enumerate(cls.original) if r['topic']==RAW)
        cls.output=next(i for i,r in enumerate(cls.original) if r['topic']==OUT)
        cls.diag=next(i for i,r in enumerate(cls.original) if r['topic']==EVENT)
    def setUp(self):self.rows=list(self.original)
    def changed(self,index):
        self.rows[index]=copy.deepcopy(self.rows[index]);return self.rows[index]
    def rejected(self):self.assertFalse(audit_rows(self.rows)['pass'])
    def test_original_full_initialization_passes(self):
        result=audit_rows(self.rows);self.assertTrue(result['pass']);self.assertEqual(result['initialization_inputs_retained'],66)
    def test_missing_initializing_input_rejected(self):self.rows.pop(self.initial);self.rejected()
    def test_initializing_position_mutation_rejected(self):
        self.changed(self.initial)['data']['pose']['pose']['position']['x']+=.1;self.rejected()
    def test_initializing_nan_rejected(self):
        self.changed(self.initial)['data']['pose']['pose']['position']['x']=float('nan');self.rejected()
    def test_nonzero_output_velocity_mutation_rejected(self):
        self.changed(self.output)['data']['twist']['twist']['linear']['x']+=.1;self.rejected()
    def test_missing_callback_disposition_rejected(self):self.rows.pop(self.diag);self.rejected()
    def test_misordered_callback_stamp_rejected(self):
        row=self.changed(self.diag);e=json.loads(row['data']['data']);e['stamp_s']+=.2;row['data']['data']=json.dumps(e);self.rejected()
    def test_missing_published_output_rejected(self):self.rows.pop(self.output);self.rejected()
    def test_nan_output_stamp_rejected(self):self.changed(self.output)['message_stamp_s']=float('nan');self.rejected()

if __name__=='__main__':unittest.main(verbosity=2)
