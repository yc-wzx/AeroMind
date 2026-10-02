import importlib.util,math,random,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import body_wall_gap,wall_box
s=importlib.util.spec_from_file_location('geometry_before',ROOT/'tools/results/provincial_stage15_completion_20261001/geometry_before.py');old=importlib.util.module_from_spec(s);s.loader.exec_module(old)
class GeometryAcceleration(unittest.TestCase):
 def test_exact_reference_random_rotated_overlap(self):
  rng=random.Random(61001)
  walls=[wall_box([rng.uniform(-2,10),rng.uniform(-2,8),rng.uniform(-2,10),rng.uniform(-2,8)],.055) for _ in range(11)]
  for _ in range(1200):
   args=(rng.uniform(-3,11),rng.uniform(-3,9),rng.uniform(-math.pi,math.pi),walls)
   self.assertAlmostEqual(body_wall_gap(*args),old.body_wall_gap(*args),places=12)
 def test_missing_walls_retains_original_rejection(self):
  with self.assertRaises(ValueError):body_wall_gap(0,0,0,[])
  with self.assertRaises(ValueError):old.body_wall_gap(0,0,0,[])
 def test_recorded_gt_matches_reference(self):
  import json
  from summarize_provincial_stage15_terminal_v3 import sdf_walls
  p=ROOT/'tools/results/provincial_stage15_completion_20261001/round_trip_04/round_trip_04_forward/round_trip_04_forward.native.jsonl';walls=sdf_walls()
  for line in p.read_text().splitlines():
   row=json.loads(line)
   if row['topic']!='/gazebo/odometry':continue
   pt=row['data']['pose']['pose']['position'];q=row['data']['pose']['pose']['orientation'];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
   args=(pt['x'],pt['y'],yaw,walls)
   self.assertAlmostEqual(body_wall_gap(*args),old.body_wall_gap(*args),places=12)
if __name__=='__main__':unittest.main()
