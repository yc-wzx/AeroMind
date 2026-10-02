#!/usr/bin/env python3
"""Mutation tests of the actual new audit entry's parameter evidence guard."""
import importlib.util,json,hashlib,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import yaml
ROOT=Path(__file__).resolve().parents[3]
path=ROOT/'tools/stage2/lio_model_validation_v2/audit_model_navigation.py'
spec=importlib.util.spec_from_file_location('model_audit_v2',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        (self.root/'runtime_parameters').mkdir()
        self.params=self.root/'runtime_parameters/lio_mapping.yaml'
        self.params.write_text(yaml.safe_dump({'/lio_mapping':{'ros__parameters':{
            'mapping.acc_cov':1e-6,'mapping.gyr_cov':1e-8,'mapping.b_acc_cov':0.,'mapping.b_gyr_cov':0.}}}))
        self.gate=self.root/'runtime_parameter_gate.json';self.gate.write_text(json.dumps({'pass':True}))
        self.manifest=self.root/'runtime_parameter_sha256.json';self.seal()
        self.patch=patch.object(module,'audit',return_value={'checks':{'original_audit_fixture':True}});self.patch.start()
    def tearDown(self):self.patch.stop();self.temp.cleanup()
    def seal(self):
        self.manifest.write_text(json.dumps({str(p.relative_to(self.root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [self.params,self.gate]}))
    def test_original_evidence_passes(self):self.assertTrue(module.evaluate(self.root)['all_pass'])
    def test_changed_parameter_file_rejected(self):
        self.params.write_text(self.params.read_text()+'# changed after query\n')
        self.assertFalse(module.evaluate(self.root)['all_pass'])
    def test_missing_manifest_rejected(self):
        self.manifest.unlink();self.assertFalse(module.evaluate(self.root)['all_pass'])
    def test_rehashed_failed_gate_still_rejected(self):
        self.gate.write_text(json.dumps({'pass':False}));self.seal();self.assertFalse(module.evaluate(self.root)['all_pass'])

if __name__=='__main__':unittest.main()
