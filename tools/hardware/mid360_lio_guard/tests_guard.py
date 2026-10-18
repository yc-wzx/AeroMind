import copy,json,math,os,sys,tempfile,time,unittest
from pathlib import Path
import numpy as np
from input_health import InputHealth,PoseHealth
from input_guard import Guard,inspect_cloud
import rclpy
from sensor_msgs.msg import PointCloud2,PointField,Imu
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile

def cloud(stamp=10**18,span_s=.099):
    m=PointCloud2();m.header.frame_id='livox_frame';m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9)
    names=['x','y','z','intensity','tag','line','timestamp'];offsets=[0,4,8,12,16,17,18];types=[7,7,7,7,2,2,8]
    m.fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in zip(names,offsets,types)]
    dtype=np.dtype(dict(names=names,formats=['<f4']*4+['u1','u1','<f8'],offsets=offsets,itemsize=26))
    a=np.zeros(12,dtype=dtype);a['x']=2.;a['intensity']=20.;a['timestamp']=stamp+np.linspace(0,span_s*1e9,12)
    m.height=1;m.width=len(a);m.point_step=26;m.row_step=26*len(a);m.data=a.tobytes();return m

def imu(stamp=10**18):
    m=Imu();m.header.frame_id='mid360_imu';m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9);m.linear_acceleration.z=-9.80665;return m

class PureTests(unittest.TestCase):
    def armed(self):
        g=InputHealth();g.accept('points',10**18,10**9);g.accept('imu',10**18,10**9);return g
    def test_valid_inputs(self):
        g=InputHealth()
        for i in range(301):
            t=10**9+i*5_000_000;s=10**18+i*5_000_000
            self.assertTrue(g.accept('imu',s,t))
            if i%20==0:self.assertTrue(g.accept('points',s,t))
        self.assertTrue(g.ready(t))
    def test_scan_start_is_not_latest_point_time(self):
        g=InputHealth();self.assertTrue(g.accept('points',10**18,10**9,10**18+100_000_000))
        for i in range(22):self.assertTrue(g.accept('imu',10**18+100_000_000+i*5_000_000,10**9+i*5_000_000))
        self.assertIsNone(g.failure)
    def test_sensor_gap_latches_without_recovery(self):
        g=self.armed();self.assertFalse(g.accept('imu',10**18+1_000_000_000,10**9+5_000_000));self.assertFalse(g.accept('imu',10**18+5_000_000,10**9+6_000_000))
    def test_receive_gap_even_continuous_sensor_time(self):
        g=self.armed();self.assertFalse(g.accept('imu',10**18+5_000_000,10**9+101_000_000))
    def test_cloud_stale_with_imu_continuing(self):
        g=self.armed()
        for i in range(1,41):self.assertTrue(g.accept('imu',10**18+i*5_000_000,10**9+i*5_000_000))
        self.assertFalse(g.tick(10**9+200_000_001))
    def test_duplicate_and_regression(self):
        for delta in [0,-1]:
            self.assertFalse(self.armed().accept('imu',10**18+delta,10**9+5_000_000))
    def test_point_span_counterexample(self):
        self.assertLess(inspect_cloud(cloud())['point_span_s'],.2)
        with self.assertRaises(ValueError):inspect_cloud(cloud(span_s=1827.19912))
    def test_pose_static_and_moving(self):
        h=PoseHealth()
        for i in range(61):self.assertTrue(h.accept(10**18+i*10**8,10**9+i*10**8,[.001*math.sin(i),0,0],[0,0,0,1]))
        self.assertTrue(h.static_report()['pass_'])
        h=PoseHealth()
        for i in range(51):self.assertTrue(h.accept(10**18+i*10**8,10**9+i*10**8,[.01*i,0,0],[0,0,0,1]))
        self.assertFalse(h.static_report()['pass_'])
    def test_historical_raw_runaway_rejected_before_push(self):
        h=PoseHealth();self.assertTrue(h.accept(10**18,10**9,[-84719.67,134601.40,238562.50],[0,0,0,1]))
        self.assertFalse(h.accept(10**18+10**8,10**9+10**8,[-84799.67,134701.40,238862.50],[0,0,0,1]))
    def test_nan_and_time_gap_rejected(self):
        for position in [[float('nan'),0,0],[float('inf'),0,0]]:
            self.assertFalse(PoseHealth().accept(1,1,position,[0,0,0,1]))
        h=PoseHealth();h.accept(10**18,10**9,[0,0,0],[0,0,0,1]);self.assertFalse(h.accept(10**18+300_000_000,10**9+300_000_000,[0,0,0],[0,0,0,1]))

class ProductionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def test_actual_guard_does_not_forward_malformed_cloud_or_resume(self):
        with tempfile.TemporaryDirectory() as d:
            g=Guard(d);g.sources_verified=True
            try:
                g.receive('points',cloud());self.assertEqual(g.forwarded['points'],1)
                g.receive('points',cloud(stamp=10**18+100_000_000,span_s=1827.19912))
                self.assertIsNotNone(g.engine.failure);self.assertEqual(g.forwarded['points'],1)
                g.receive('points',cloud(stamp=10**18+200_000_000));self.assertEqual(g.forwarded['points'],1)
                self.assertEqual(g.status()['state'],'FAULT_LATCHED')
            finally:g.events.close();g.destroy_node()
    def test_interleaved_points_with_valid_span_forward_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            g=Guard(d);g.sources_verified=True;m=cloud();data=bytearray(m.data)
            a,b=3*m.point_step,4*m.point_step
            data[a:a+m.point_step],data[b:b+m.point_step]=data[b:b+m.point_step],data[a:a+m.point_step]
            m.data=bytes(data);before=bytes(serialize_message(m))
            try:
                self.assertGreater(inspect_cloud(m)['within_cloud_regressions'],0)
                g.receive('points',m);self.assertIsNone(g.engine.failure);self.assertEqual(g.forwarded['points'],1)
                self.assertEqual(bytes(serialize_message(m)),before)
            finally:g.events.close();g.destroy_node()
    def test_missing_source_does_not_forward(self):
        with tempfile.TemporaryDirectory() as d:
            g=Guard(d)
            try:g.receive('points',cloud());self.assertEqual(g.forwarded['points'],0)
            finally:g.events.close();g.destroy_node()
    def test_actual_pose_monitor_latches_runaway(self):
        with tempfile.TemporaryDirectory() as d:
            g=Guard(d);m=Odometry();m.header.stamp.sec=100;m.pose.pose.orientation.w=1.
            try:
                g.receive_pose(m);m.header.stamp.nanosec=100_000_000;m.pose.pose.position.x=1000.
                g.receive_pose(m);self.assertIn('3m/s',g.engine.failure)
            finally:g.events.close();g.destroy_node()
    def test_real_dds_source_identity_and_original_payload(self):
        with tempfile.TemporaryDirectory() as d:
            g=Guard(d);radar=Node('mid360_real_driver');adapter=Node('mid360_imu_unit_adapter');observer=Node('guard_test_observer')
            p=radar.create_publisher(PointCloud2,'/hardware/mid360/points',10)
            q=adapter.create_publisher(Imu,'/hardware/mid360/imu_si',qos_profile_sensor_data)
            received=[]
            sub=observer.create_subscription(PointCloud2,'/hardware/mid360/checked/points',lambda m:received.append(bytes(serialize_message(m))),10)
            try:
                deadline=time.monotonic()+5
                while time.monotonic()<deadline and not g.discover():time.sleep(.02)
                self.assertTrue(g.sources_verified);time.sleep(.2)
                original=cloud();p.publish(original)
                deadline=time.monotonic()+2
                while time.monotonic()<deadline and not received:
                    rclpy.spin_once(g,timeout_sec=.01);rclpy.spin_once(observer,timeout_sec=.01)
                self.assertEqual(received,[bytes(serialize_message(original))])
            finally:
                g.events.close();g.destroy_node();radar.destroy_node();adapter.destroy_node();observer.destroy_node()

if __name__=='__main__':unittest.main(verbosity=2)
