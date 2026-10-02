import ast,types,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];SOURCE=ROOT/'src/uav_planning/scripts/gazebo_navigation_interface.py'
tree=ast.parse(SOURCE.read_text());method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='observe_guard_resume');ns={};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(SOURCE),'exec'),ns);observe=ns['observe_guard_resume']
def state():return types.SimpleNamespace(active_waypoint=object(),active_waypoint_diag_id='R0002:W01',stage_last_progress_at=114.9,get_logger=lambda:types.SimpleNamespace(info=lambda text:None),get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=116860000000)))
class ResumeClockTests(unittest.TestCase):
 def test_replayed_guard_resume_does_not_spend_fresh_motion_window(self):
  s=state();observe(s,'unsafe_plan',115.852);observe(s,'allowed',116.86)
  self.assertEqual(s.stage_last_progress_at,116.86)
  self.assertFalse(117.76-s.stage_last_progress_at>2)
  self.assertTrue(119.-s.stage_last_progress_at>2)
 def test_continuous_rejection_does_not_suppress_original_retry(self):
  s=state();observe(s,'unsafe_plan',115.852);observe(s,'unsafe_plan',117.76)
  self.assertEqual(s.stage_last_progress_at,114.9);self.assertTrue(117.76-s.stage_last_progress_at>2)
 def test_each_safe_replan_does_not_keep_resetting_timer(self):
  s=state();observe(s,'allowed',116.86);observe(s,'allowed',117.76);self.assertEqual(s.stage_last_progress_at,114.9)
 def test_new_waypoint_does_not_inherit_previous_guard_resume(self):
  s=state();observe(s,'unsafe_plan',115.852);s.active_waypoint_diag_id='R0002:W02';observe(s,'allowed',116.86);self.assertEqual(s.stage_last_progress_at,114.9)
 def test_idle_does_not_reset_progress(self):
  s=state();observe(s,'unsafe_plan',115.852);s.active_waypoint=None;observe(s,'allowed',116.86);self.assertEqual(s.stage_last_progress_at,114.9)
if __name__=='__main__':unittest.main()
