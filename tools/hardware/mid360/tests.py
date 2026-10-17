#!/usr/bin/env python3
"""Synthetic ROS-message/unit/storage tests only; never a real device qualification."""
import copy
import importlib.util
import json
import math
import struct
import tempfile
import time
import unittest
from pathlib import Path
from sensor_msgs.msg import PointCloud2, PointField, Imu
from rclpy.serialization import serialize_message
from core import G,POINTS,IMU_RAW,IMU_SI,imu_to_si,imu_values,inspect_cloud,stream_stats

def cloud(big=False):
    p=PointCloud2();p.header.frame_id='livox_frame';p.header.stamp.sec=100
    p.width=2;p.height=1;p.point_step=26;p.row_step=52;p.is_bigendian=big
    p.fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in
              [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]]
    fmt=('>' if big else '<')+'ffffBBd'
    p.data=struct.pack(fmt,1.,2.,3.,10.,0x10,0,100e9)+struct.pack(fmt,2.,3.,4.,20.,0,3,100e9+50e6)
    return p

def imu():
    p=Imu();p.header.frame_id='livox_frame';p.header.stamp.sec=100
    p.linear_acceleration.z=-1.;p.angular_velocity.z=.1
    p.linear_acceleration_covariance=[.1]*9
    return p

class InputTests(unittest.TestCase):
    def test_native_point_timing_and_endian(self):
        for big in [False,True]:
            v=inspect_cloud(cloud(big));self.assertEqual(v['points'],2);self.assertEqual(v['usable_points'],2);self.assertAlmostEqual(v['point_span_s'],.05)
    def test_empty_cloud(self):
        p=cloud();p.width=0
        with self.assertRaisesRegex(ValueError,'empty'):inspect_cloud(p)
    def test_nonfinite_points_and_time(self):
        for offset in [0,4,8,12,18]:
            p=cloud();b=bytearray(p.data);struct.pack_into('<d' if offset==18 else '<f',b,offset,float('nan'));p.data=bytes(b)
            with self.assertRaisesRegex(ValueError,'nonfinite'):inspect_cloud(p)
    def test_missing_timestamp(self):
        p=cloud();p.fields=p.fields[:-1]
        with self.assertRaisesRegex(ValueError,'timestamp'):inspect_cloud(p)
    def test_bad_field_type(self):
        p=cloud();p.fields[-1].datatype=7
        with self.assertRaises(ValueError):inspect_cloud(p)
    def test_duplicate_field(self):
        p=cloud();p.fields.append(copy.deepcopy(p.fields[0]))
        with self.assertRaisesRegex(ValueError,'duplicate'):inspect_cloud(p)
    def test_bad_byte_count(self):
        p=cloud();p.data=p.data[:-1]
        with self.assertRaisesRegex(ValueError,'byte count'):inspect_cloud(p)
    def test_wrong_time_unit(self):
        p=cloud();b=bytearray(p.data);struct.pack_into('<d',b,18,100.);p.data=bytes(b)
        with self.assertRaisesRegex(ValueError,'nanosecond'):inspect_cloud(p)
    def test_instantaneous_sim_cloud_rejected(self):
        p=cloud();b=bytearray(p.data);struct.pack_into('<d',b,44,100e9);p.data=bytes(b)
        with self.assertRaisesRegex(ValueError,'instantaneous'):inspect_cloud(p)
    def test_bad_line(self):
        p=cloud();b=bytearray(p.data);b[17]=4;p.data=bytes(b)
        with self.assertRaisesRegex(ValueError,'line'):inspect_cloud(p)
    def test_row_padding(self):
        p=cloud();raw=bytes(p.data);p.width=1;p.height=2;p.row_step=30;p.data=raw[:26]+b'1234'+raw[26:]+b'5678'
        self.assertEqual(inspect_cloud(p)['points'],2)
    def test_real_imu_unit_conversion_no_flip(self):
        p=imu();before=copy.deepcopy(p);out=imu_to_si(p)
        self.assertEqual(p,before);self.assertEqual(out.header.stamp,p.header.stamp)
        self.assertEqual(out.header.frame_id,'mid360_imu');self.assertAlmostEqual(out.linear_acceleration.z,-G)
        self.assertEqual(out.angular_velocity,p.angular_velocity)
        self.assertAlmostEqual(out.linear_acceleration_covariance[0],.1*G*G)
        self.assertEqual(out.orientation_covariance[0],-1.)
    def test_nan_imu_rejected(self):
        p=imu();p.angular_velocity.x=float('nan')
        with self.assertRaisesRegex(ValueError,'nonfinite'):imu_to_si(p)
    def test_imu_timestamp_missing(self):
        p=imu();p.header.stamp.sec=0
        with self.assertRaisesRegex(ValueError,'timestamp'):imu_values(p)
    def test_stream_finite_continuity(self):
        rows=[dict(stamp_ns=1_000_000_000+i*100_000_000,receive_monotonic_ns=2_000_000_000+i*100_000_000,error=None) for i in range(111)]
        self.assertTrue(stream_stats(rows)['pass_'])
        for value in [rows[49]['stamp_ns'],rows[49]['stamp_ns']-1]:
            bad=copy.deepcopy(rows);bad[50]['stamp_ns']=value;self.assertFalse(stream_stats(bad)['pass_'])
        bad=copy.deepcopy(rows);bad[50]['error']='nonfinite';self.assertFalse(stream_stats(bad)['pass_'])
        self.assertFalse(stream_stats(rows[:2])['pass_']);self.assertFalse(stream_stats([])['pass_'])
    def test_async_bag_preserves_every_message_and_detects_health_error(self):
        from capture import Recorder,summarize
        from core import stamp_ns,imu_values
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);recorder=Recorder(root);rows={t:[] for t in [POINTS,IMU_RAW,IMU_SI]};expected={}
            for topic,message in [(POINTS,cloud()),(IMU_RAW,imu()),(IMU_SI,imu_to_si(imu()))]:
                row=dict(topic=topic,sequence=1,stamp_ns=stamp_ns(message),receive_monotonic_ns=time.monotonic_ns(),receive_wall_ns=time.time_ns(),error=None)
                row.update(inspect_cloud(message) if topic==POINTS else imu_values(message))
                serialized=serialize_message(message);expected[topic]=serialized
                recorder.put(topic,serialized,row);rows[topic].append(row)
            health=recorder.close();result=summarize(rows,health,root)
            self.assertTrue(result['bag_counts_verified']);self.assertFalse(result['pass_raw']) # insufficient duration
            self.assertEqual(len((root/'receipts.jsonl').read_text().splitlines()),3)
            import rosbag2_py
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'raw_bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            while reader.has_next():
                topic,data,_=reader.read_next();self.assertEqual(data,expected.pop(topic))
            self.assertFalse(expected);del reader
            health['queue_overflow']=1;self.assertFalse(summarize(rows,health,root)['bag_counts_verified'])

if __name__=='__main__':unittest.main(verbosity=2)
