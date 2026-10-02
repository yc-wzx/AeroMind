"""Actual LIO interface update and inherited production control, no ROS transport."""
import sys,unittest,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/stage2_lio_sim/scripts'),str(ROOT/'tools/stage2')]
from lio_navigation_interface import LioNavigationInterface
from localization_hold_fixture import Clock,full_node,isolated
from sensor_msgs.msg import PointCloud2

class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.n,self.logs=full_node(self.clock)
        self.n.__class__=LioNavigationInterface;self.n.last_lio_received=1.
        self.context=isolated(self.clock,self.logs);self.context.__enter__()
    def tearDown(self):self.context.__exit__(None,None,None)
    def tick(self,t,cloud=True,odom=True):
        self.clock.t=t;self.n.last_command_time=self.n.last_odom_received=self.n.scan_received_at=t
        if cloud:self.n.last_match_sim=self.n.last_match_steady=t
        if odom:self.n.last_lio_received=t
        self.n.update()
        m=self.n.drive_pub.messages[-1];return m.linear.x,m.linear.y,m.angular.z
    def test_cloud_loss_stops_even_fresh_high_rate_odom(self):
        for t in [10,12,30]:self.assertEqual(self.tick(t,False,True),(0.,0.,0.))
        self.assertEqual(self.n.stage_retry_count,0);self.assertFalse(self.n.route_pub.messages)
    def test_odom_loss_stops_even_fresh_cloud(self):
        self.assertEqual(self.tick(10,True,False),(0.,0.,0.))
    def test_both_fresh_safe_plan_allows_motion(self):
        self.n.stage_sent_at=self.n.stage_last_progress_at=10.
        self.assertNotEqual(self.tick(10),(0.,0.,0.))
    def test_resume_requires_safe_plan(self):
        self.tick(10,False,True);self.n.planned_path_safe=False
        self.assertEqual(self.tick(12),(0.,0.,0.));self.n.planned_path_safe=True
        self.assertNotEqual(self.tick(12.1),(0.,0.,0.))
    def test_expired_command_still_stops(self):
        self.clock.t=10.;self.n.last_match_sim=self.n.last_match_steady=self.n.last_lio_received=10.
        self.n.last_command_time=1.;self.n.update();m=self.n.drive_pub.messages[-1]
        self.assertEqual((m.linear.x,m.linear.y,m.angular.z),(0.,0.,0.))
    def test_bad_correction_frame_does_not_refresh(self):
        m=PointCloud2();m.header.frame_id='wrong';m.width=24;m.height=1;m.header.stamp.sec=10
        self.n.lio_correction(m);self.assertEqual(self.n.last_match_sim,1.)

if __name__=='__main__':unittest.main(verbosity=2)
