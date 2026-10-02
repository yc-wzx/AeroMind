"""New result/trial parameterization refuses invalid inputs before any launch."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import run_scan_resume_experiment as entry

class EntryTests(unittest.TestCase):
    def test_existing_explicit_directory_refuses_without_launch(self):
        with tempfile.TemporaryDirectory() as d,patch.object(entry.subprocess,'Popen') as launch:
            with self.assertRaises(FileExistsError):entry.run_profile('zero',output_dir=Path(d),trial_id='short_02')
            launch.assert_not_called()
    def test_invalid_trial_refuses_without_launch(self):
        with patch.object(entry.subprocess,'Popen') as launch:
            for name in ('../overwrite','a/b','', 'a'*65):
                with self.subTest(name=name),self.assertRaises(ValueError):entry.run_profile('zero',trial_id=name)
            launch.assert_not_called()
    def test_nonzero_profile_refuses_without_launch(self):
        with patch.object(entry.subprocess,'Popen') as launch:
            with self.assertRaises(ValueError):entry.run_profile('yaw_bias')
            launch.assert_not_called()
    def test_explicit_parameters_exist_and_v2_audit_selected(self):
        import inspect
        parameters=inspect.signature(entry.run_profile).parameters
        self.assertIn('output_dir',parameters);self.assertIn('trial_id',parameters)
        self.assertEqual(entry.audit.__module__,'audit_scan_resume_v2')

if __name__=='__main__':unittest.main()
