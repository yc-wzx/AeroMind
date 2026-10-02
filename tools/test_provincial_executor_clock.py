import json, math, sys, types, unittest
from pathlib import Path
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
import ego_trajectory_executor as executor

def message():
    return types.SimpleNamespace(order=3,pos_pts=[types.SimpleNamespace(x=0.,y=float(i),z=0.) for i in range(7)],knots=[-3.,-2.,-1.,0.,1.,2.,3.,4.,5.,6.,7.],start_time=types.SimpleNamespace(sec=1_700_000_000,nanosec=0),traj_id=1)
def state(mode):
    return types.SimpleNamespace(trajectory_start_clock=mode,get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=10_000_000_000)),get_logger=lambda:Mock(),publish_stop=Mock(),publish_planned_marker=Mock(),frame_id='odom',position_spline='old')

class ExecutorClockTests(unittest.TestCase):
    def test_declared_system_clock_preserves_two_second_age(self):
        s=state('system')
        with patch.object(executor.time,'time_ns',return_value=1_700_000_002_000_000_000):executor.EgoTrajectoryExecutor.bspline_callback(s,message())
        self.assertAlmostEqual(s.elapsed_at_receive,2.)
        self.assertEqual(s.received_ros_ns,10_000_000_000)
        self.assertNotEqual(s.position_spline.evaluate(s.position_spline.start),s.position_spline.evaluate(s.position_spline.start+s.elapsed_at_receive))
    def test_ros_mode_uses_same_ros_stamp(self):
        s=state('ros');m=message();m.start_time.sec=9
        executor.EgoTrajectoryExecutor.bspline_callback(s,m)
        self.assertAlmostEqual(s.elapsed_at_receive,1.)
    def test_fresh_system_trajectory_starts_at_zero(self):
        s=state('system')
        with patch.object(executor.time,'time_ns',return_value=1_700_000_000_000_000_000):executor.EgoTrajectoryExecutor.bspline_callback(s,message())
        self.assertEqual(s.elapsed_at_receive,0.)
    def test_expired_system_trajectory_stops_and_cannot_replay_old(self):
        s=state('system')
        with patch.object(executor.time,'time_ns',return_value=1_700_000_020_000_000_000):executor.EgoTrajectoryExecutor.bspline_callback(s,message())
        self.assertIsNone(s.position_spline);s.publish_stop.assert_called_once();s.publish_planned_marker.assert_not_called()
    def test_malformed_spline_still_rejected(self):
        s=state('system');m=message();m.pos_pts[0].x=math.nan
        executor.EgoTrajectoryExecutor.bspline_callback(s,m);s.publish_stop.assert_called_once()
    def test_real_record_separates_wall_step_from_transport_latency(self):
        p=ROOT/'tools/results/provincial_stage15_completion_20261001/clock_handoff_diagnosis_v2.json';d=json.loads(p.read_text())['relevant_delayed_trajectory'][0]
        self.assertLess(abs(d['estimated_transport_or_planning_age_s']),.01)
        self.assertLess(d['system_vs_monotonic_offset_change_s'],-2.)
        self.assertEqual(d['old_elapsed_at_receive_s'],0)
        self.assertGreater(d['start_system_s'],1e9)

if __name__=='__main__':unittest.main()
