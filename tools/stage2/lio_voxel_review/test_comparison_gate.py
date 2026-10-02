import copy
import unittest
from compare_and_gate import metric_checks
from analyze_voxel import flatten


class Gate(unittest.TestCase):
    def setUp(self):
        self.b={'evidence_pass':True,'replayed':{'max_xy_error_m':.05,'RMS_xy_m':.02,'last':[100.,.04]}}
        self.c={'evidence_pass':True,'replayed':{'max_xy_error_m':.02,'RMS_xy_m':.004,'last':[100.,.005]}}
    def passes(self):return all(metric_checks(self.b,self.c).values())
    def test_valid(self):self.assertTrue(self.passes())
    def test_tiny_improvement_is_not_accepted(self):
        self.c['replayed']['max_xy_error_m']=.049;self.assertFalse(self.passes())
    def test_worse_RMS(self):
        self.c['replayed']['RMS_xy_m']=.03;self.assertFalse(self.passes())
    def test_worse_terminal(self):
        self.c['replayed']['last'][1]=.05;self.assertFalse(self.passes())
    def test_missing_evidence(self):
        self.c['evidence_pass']=False;self.assertFalse(self.passes())
    def test_NaN_rejected(self):
        self.c['replayed']['max_xy_error_m']=float('nan');self.assertFalse(self.passes())
    def test_missing_metric(self):
        del self.c['replayed']['last'];self.assertFalse(self.passes())
    def test_nested_and_flat_parameters_equivalent(self):
        values={'common':{'planar_mode':False},'mapping':{'acc_cov':1e-6,'extrinsic_T':[0.,0.,0.]}}
        expected={'common.planar_mode':False,'mapping.acc_cov':1e-6,'mapping.extrinsic_T':[0.,0.,0.]}
        self.assertEqual(flatten(values),expected);self.assertEqual(flatten(expected),expected)


if __name__=='__main__':unittest.main()
