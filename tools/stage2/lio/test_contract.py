import sys,unittest,struct,math
from pathlib import Path
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField,Imu
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/stage2_lio_sim/scripts'))
from lio_contract import points,pack_mid360,valid_imu,body_velocity,window_velocity,align_pose,RAW_CLOUD_FRAME,RAW_IMU_FRAME
from lio_sensor_adapter import SensorAdapter

def cloud():
    m=PointCloud2();m.header.frame_id=RAW_CLOUD_FRAME;m.header.stamp.sec=5
    m.width=24;m.height=1;m.point_step=12;m.row_step=288
    m.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')]
    m.data=b''.join(struct.pack('<fff',1.,i*.01,.5) for i in range(24));return m

class Pub:
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(m)

class Fixture:
    check_time=SensorAdapter.check_time;report=SensorAdapter.report
    cloud=SensorAdapter.cloud;imu=SensorAdapter.imu
    def __init__(self):
        self.last={};self.counts={'cloud':0,'imu':0,'rejected':0};self.diag=Pub();self.cloud_pub=Pub();self.imu_pub=Pub()
    def get_clock(self):
        class Clock:
            def now(self):
                class T:nanoseconds=5000000000
                return T()
        return Clock()

class Tests(unittest.TestCase):
    def test_cloud_conversion_actual_callback(self):
        n=Fixture();n.cloud(cloud());self.assertEqual(len(n.cloud_pub.messages),1)
        m=n.cloud_pub.messages[0];self.assertEqual(m.header.stamp.sec,5)
        self.assertEqual(m.header.frame_id,'lio_lidar');ts=[struct.unpack_from('<d',m.data,i*32+24)[0] for i in range(m.width)]
        self.assertEqual(set(ts),{5000000000.});self.assertEqual(m.width,24)
    def test_invalid_frame_no_publish(self):
        n=Fixture();m=cloud();m.header.frame_id='world';n.cloud(m);self.assertFalse(n.cloud_pub.messages)
    def test_malformed_payload(self):
        m=cloud();m.data=m.data[:-1]
        with self.assertRaises(ValueError):points(m)
    def test_no_returns_removed_not_zero(self):
        m=cloud();m.width=25;m.row_step=300;m.data=bytes(m.data)+struct.pack('<fff',math.inf,math.nan,math.inf)
        a,k=points(m);self.assertEqual(k,1);self.assertEqual(len(a),24);self.assertTrue(np.isfinite(a).all())
    def test_all_missing_rejected(self):
        m=cloud();m.data=struct.pack('<fff',math.nan,0.,0.)*24
        with self.assertRaises(ValueError):points(m)
    def test_duplicate_stale_future_no_publish(self):
        for delta in [-1,1]:
            n=Fixture();m=cloud();m.header.stamp.sec+=delta;n.cloud(m);self.assertFalse(n.cloud_pub.messages)
        n=Fixture();n.cloud(cloud());n.cloud(cloud());self.assertEqual(len(n.cloud_pub.messages),1)
    def test_imu_excludes_absolute_orientation(self):
        n=Fixture();m=Imu();m.header.frame_id=RAW_IMU_FRAME;m.header.stamp.sec=5;m.orientation.w=1.;m.linear_acceleration.z=9.81
        n.imu(m);o=n.imu_pub.messages[0];self.assertEqual(o.orientation.w,0.);self.assertEqual(o.orientation_covariance[0],-1.)
        self.assertEqual(o.linear_acceleration.z,9.81);self.assertEqual(o.header.stamp.sec,5)
    def test_bad_imu_rejected(self):
        n=Fixture();m=Imu();m.header.frame_id=RAW_IMU_FRAME;m.header.stamp.sec=5;m.angular_velocity.z=math.nan
        n.imu(m);self.assertFalse(n.imu_pub.messages)
    def test_alignment_yaw_body(self):
        x,y,z,a=align_pose(1.,0.,0.,0.);self.assertAlmostEqual(x,4.7);self.assertAlmostEqual(y,1.5)
        vx,vy,wz=body_velocity((1.,4.7,.5,math.pi/2),(1.1,4.7,.51,math.pi/2))
        self.assertAlmostEqual(vx,.1);self.assertAlmostEqual(vy,0.);self.assertEqual(wz,0.)
    def test_nonmonotonic_velocity(self):
        with self.assertRaises(ValueError):body_velocity((1,0,0,0),(1,0,0,0))
    def test_window_constant_motion(self):
        h=[(i*.004,4.7,.5+i*.004*.2,math.pi/2) for i in range(51)]
        v=window_velocity(h);self.assertAlmostEqual(v[0],.2);self.assertAlmostEqual(v[1],0.);self.assertAlmostEqual(v[2],0.)
    def test_window_missing_or_nan_not_zero(self):
        for h in [[(0,0,0,0)],[(0,0,0,0),(.1,math.nan,0,0),(.2,0,0,0)]]:
            with self.assertRaises(ValueError):window_velocity(h)

if __name__=='__main__':unittest.main(verbosity=2)
