import tempfile,unittest
from pathlib import Path
import run_provincial_full_observed as observed
class InputCaptureTests(unittest.TestCase):
 def test_actual_sourced_elf_closure_and_local_plugins_are_snapshotted(self):
  previous=observed.OUT
  try:
   with tempfile.TemporaryDirectory() as t:
    observed.OUT=Path(t);m=observed.capture_inputs()
    libs={str(p.relative_to(observed.ROOT)) for p in (observed.ROOT/'install').rglob('*.so*') if p.is_file()}
    self.assertGreater(len(libs),0);self.assertTrue(libs<=m['inputs'].keys());self.assertTrue(m['local_linked_elf_closure_captured'])
    for k in libs:self.assertEqual(m['inputs'][k]['sha256'],observed.digest(observed.OUT/'input_snapshot'/k))
    observed.final_hashes(m)
    import json
    hashes=json.loads((observed.OUT/'raw_data_sha256.json').read_text());self.assertIn('input_manifest.json',hashes);self.assertIn('planner_linked_dependencies.json',hashes);self.assertIn('git_diff_before.patch',hashes)
    self.assertTrue(all(observed.digest(observed.OUT/k)==v for k,v in hashes.items()))
  finally:observed.OUT=previous
if __name__=='__main__':unittest.main()
