#!/usr/bin/env python3
"""Simulation format boundary after physical sensor inversion + normalization.

Retains existing instantaneous Gazebo -> Livox layout conversion. The virtual
2D safety scan is also obtained from inverted physical rays, not a second
upright sensor. Never subscribes to GT/odom or actuator commands.
"""
import copy
from array import array
import json
import math
import sys
import time
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from inverted_mount_contract import transform_cloud,transform_imu
from rmw_diagnostic_executor import RmwDiagnosticExecutor
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan,PointCloud2,PointField,Imu
from lio_sensor_adapter import SensorAdapter
from lio_contract import RAW_CLOUD_FRAME, RAW_IMU_FRAME, points, pack_mid360


def upright_scan(msg):
    if msg.header.frame_id != 'omni_robot/gimbal_link/lidar_inverted':
        raise ValueError('wrong inverted physical scan frame')
    count=len(msg.ranges)
    if count < 2 or len(msg.intensities) not in (0, count):
        raise ValueError('malformed inverted scan dimensions')
    if not all(math.isfinite(v) for v in [msg.angle_min,msg.angle_max,msg.angle_increment]):
        raise ValueError('nonfinite scan angle metadata')
    if msg.angle_increment <= 0 or abs(msg.angle_min+msg.angle_max) > 1e-5:
        raise ValueError('requires symmetric raw scan angular grid; no resampling')
    if abs(msg.angle_min+(count-1)*msg.angle_increment-msg.angle_max)>1e-4:
        raise ValueError('scan angular grid inconsistent')
    # Raw azimuth theta -> upright azimuth -theta; preserve symmetric grid.
    out=copy.deepcopy(msg)
    out.header.frame_id='omni_robot/gimbal_link/lidar'
    out.ranges=list(reversed(msg.ranges))
    out.intensities=list(reversed(msg.intensities))
    return out


class InvertedSimFormatAdapter(SensorAdapter):
    def __init__(self):
        # Retain actual IMU history across a cloud callback. The bounded
        # executor takes at most eight received messages; original 0.2s age,
        # frame, duplicate and finite-value checks still run for each one.
        self._subscription_batch_limits={'/simulation/imu3d':8}
        super().__init__()
        self._callback_timing={}
        self.create_timer(2.,self.report_timing)
        self.upright_cloud_pub=self.create_publisher(PointCloud2,'/mid360/upright/lidar',10)
        self.upright_imu_pub=self.create_publisher(Imu,'/mid360/upright/imu',qos_profile_sensor_data)
        self.scan_pub=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/simulation/scan_inverted',self.scan,qos_profile_sensor_data)

    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        if topic=='/simulation/imu3d':
            qos_profile=QoSProfile(depth=32,reliability=ReliabilityPolicy.BEST_EFFORT)
        return super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)

    def report_timing(self):
        rows=self._callback_timing
        self._callback_timing={}
        self.report(event='inverted_input_callback_timing',streams=rows,
            imu_history_depth=32,imu_batch_limit=8,
            scope='Actual callback times; original measurement timestamps untouched')

    def _observe_callback(self,stream,started,stamp):
        if not hasattr(self,'_callback_timing'):return
        now=time.monotonic();row=self._callback_timing.setdefault(stream,
            dict(count=0,total_wall_s=0.,max_wall_s=0.,last_input_stamp_s=None,max_input_gap_s=0.))
        row['count']+=1;row['total_wall_s']+=now-started;row['max_wall_s']=max(row['max_wall_s'],now-started)
        previous=row['last_input_stamp_s']
        if previous is not None:row['max_input_gap_s']=max(row['max_input_gap_s'],stamp-previous)
        row['last_input_stamp_s']=stamp

    def cloud(self,msg):
        started=time.monotonic()
        try:
            # Same production rotation contract; one callback avoids another
            # DDS receive queue before the simulator-specific Livox packing.
            upright,_=transform_cloud(msg,'omni_robot/gimbal_link/lidar3d','omni_robot/gimbal_link/lidar3d_upright')
            local=copy.deepcopy(upright);local.header.frame_id=RAW_CLOUD_FRAME
            before=self.counts['cloud']
            self._pack_cloud(local)
            if self.counts['cloud']>before:self.upright_cloud_pub.publish(upright)
        except (ValueError,TypeError,AttributeError) as e:
            self.report(event='rejected',stream='cloud',reason=str(e))

        finally:
            self._observe_callback('cloud',started,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def _pack_cloud(self,msg):
        # Same production validation, omission of no-return rays, Livox layout
        # and instantaneous timestamps. Only typed byte storage differs.
        try:
            xyz,invalid=points(msg);t=self.check_time('cloud',msg)
            out=PointCloud2();out.header=copy.deepcopy(msg.header);out.header.frame_id='lio_lidar'
            out.height=1;out.width=len(xyz);out.point_step=32;out.row_step=32*len(xyz);out.is_dense=True
            layout=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',24,8)]
            out.fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in layout]
            out.data=array('B',pack_mid360(xyz,msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec))
            self.cloud_pub.publish(out);self.counts['cloud']+=1
            self.report(event='cloud_accepted',stamp_s=t,point_count=len(xyz),invalid_returns=invalid,
                instantaneous=True,point_time_span_s=0,synthetic_intensity_line=True)
        except (ValueError,TypeError,AttributeError) as error:
            self.counts['rejected']+=1;self.report(event='rejected',stream='cloud',reason=str(error))

    def imu(self,msg):
        started=time.monotonic()
        try:
            upright=transform_imu(msg,'omni_robot/gimbal_link/imu3d','omni_robot/gimbal_link/imu3d_upright','mps2')
            local=copy.deepcopy(upright);local.header.frame_id=RAW_IMU_FRAME
            before=self.counts['imu']
            super().imu(local)
            if self.counts['imu']>before:self.upright_imu_pub.publish(upright)
        except (ValueError,TypeError,AttributeError) as e:
            self.report(event='rejected',stream='imu',reason=str(e))

        finally:
            self._observe_callback('imu',started,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def scan(self,msg):
        started=time.monotonic()
        try:
            out=upright_scan(msg)
            t=self.check_time('scan',msg)
            self.scan_pub.publish(out)
        except (ValueError, TypeError, AttributeError) as error:
            self.report(event='rejected',stream='scan',reason=str(error))
        finally:
            self._observe_callback('scan',started,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)



def main():
    rclpy.init();node=InvertedSimFormatAdapter()
    executor=RmwDiagnosticExecutor();executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
