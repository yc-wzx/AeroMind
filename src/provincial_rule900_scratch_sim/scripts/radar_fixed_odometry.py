#!/usr/bin/env python3
"""LIO sensor pose -> fixed-heading chassis centre + radar pose, without encoder."""
from runtime_support import load_existing
load_existing()
import json,math,time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from lio_contract import stamp_s,window_velocity
from gimbal_contract import quaternion,rotation
from radar_fixed_contract import configured_mount

class RadarFixedOdometry(Node):
    def __init__(self):
        super().__init__('radar_fixed_odometry')
        self.mount=configured_mount(self);self.history=deque();self.last=None
        # Bounded history only; executor drains at most8 existing samples per dispatch.
        self._subscription_batch_limits={'/lio/odometry':8}
        self.input_message_info={};self.callback_context={};self.callback_count=0
        self.base_pub=self.create_publisher(Odometry,'/localization/lio_navigation_odometry',100)
        self.radar_pub=self.create_publisher(Odometry,'/localization/radar_pose',100)
        self.diag=self.create_publisher(String,'/navigation/radar_pose_diagnostics',100)
        self.create_subscription(Odometry,'/lio/odometry',self.odom,QoSProfile(depth=32,reliability=ReliabilityPolicy.BEST_EFFORT))

    def record_rmw_info(self,topic,info):
        self.input_message_info[topic]=info

    def report(self,**kw):
        data=dict(getattr(self,'callback_context',{}));data.update(kw)
        data.update(history_count=len(self.history),
            history_span_s=0. if len(self.history)<2 else self.history[-1][0]-self.history[0][0])
        self.diag.publish(String(data=json.dumps(data,allow_nan=False)))

    def odom(self,msg):
        received=time.monotonic();self.callback_count+=1
        now=self.get_clock().now().nanoseconds*1e-9;t=stamp_s(msg.header.stamp)
        self.callback_context=dict(callback_index=self.callback_count,receive_monotonic_s=received,
            callback_system_timestamp_ns=time.time_ns(),
            sim_s=now if math.isfinite(now) else None,
            input_stamp_s=t if math.isfinite(t) else None,
            input_age_s=now-t if math.isfinite(now-t) else None,
            latest_accepted_stamp_s=self.last,
            message_info=self.input_message_info.get('/lio/odometry'))
        try:
            now=self.get_clock().now().nanoseconds*1e-9;t=stamp_s(msg.header.stamp)
            if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='lio_imu':raise ValueError('wrong LIO frame')
            if not -.01<=now-t<=.2:
                # Reject this message, but do not erase actual accepted history.
                # No publication, freshness update or synthetic velocity here.
                # A real gap between accepted stamps still clears history below.
                self.report(event='LIO_rejected',reason='LIO timestamp stale/future',history_retained=True)
                return
            if self.last is not None and t<=self.last:
                self.report(event='duplicate_or_older',stamp_s=t,latest_accepted_stamp_s=self.last)
                return
            p=msg.pose.pose.position;q=msg.pose.pose.orientation;rawq=[q.x,q.y,q.z,q.w]
            body,qb,sensor,psi,theta=self.mount.from_lio([p.x,p.y,p.z],rawq)
            covariance=self.mount.body_covariance(rawq,msg.pose.covariance)
            # All pose/twist fields below are overwritten. Construct a new
            # message instead of recursively copying fields only to replace them.
            # The unmodified off-diagonal Twist covariance must still be copied.
            radar=Odometry();radar.header.stamp.sec=msg.header.stamp.sec
            radar.header.stamp.nanosec=msg.header.stamp.nanosec
            radar.twist.covariance=np.array(msg.twist.covariance,dtype=float,copy=True)
            radar.header.frame_id='odom';radar.child_frame_id='radar_control'
            radar.pose.pose.position.x,radar.pose.pose.position.y,radar.pose.pose.position.z=map(float,sensor)
            qr=quaternion(self.mount.R0@rotation(rawq))
            radar.pose.pose.orientation.x,radar.pose.pose.orientation.y,radar.pose.pose.orientation.z,radar.pose.pose.orientation.w=map(float,qr)
            # Radar pose is for heading/scan association. LIO Twist is not an
            # independently qualified chassis velocity; never publish it here.
            radar.twist.twist.linear.x=radar.twist.twist.linear.y=radar.twist.twist.linear.z=0.
            radar.twist.twist.angular.x=radar.twist.twist.angular.y=radar.twist.twist.angular.z=0.
            radar.twist.covariance[0]=radar.twist.covariance[7]=radar.twist.covariance[14]=1e6
            radar.twist.covariance[21]=radar.twist.covariance[28]=radar.twist.covariance[35]=1e6
            A=np.zeros((6,6));A[:3,:3]=self.mount.R0;A[3:,3:]=self.mount.R0
            radar.pose.covariance=(A@np.array(msg.pose.covariance).reshape(6,6)@A.T).reshape(-1).tolist()
            self.radar_pub.publish(radar)
            if self.last is not None and t-self.last>.2:self.history.clear()
            self.last=t;self.history.append((t,*map(float,body[:2]),float(self.mount.initial_base[3])))
            while self.history and t-self.history[0][0]>.2+1e-9:self.history.popleft()
            try:vx,vy,wz=window_velocity(self.history)
            except ValueError:
                self.report(event='velocity_initializing',stamp_s=t);return
            base=Odometry();base.header.stamp.sec=msg.header.stamp.sec
            base.header.stamp.nanosec=msg.header.stamp.nanosec
            base.header.frame_id='odom';base.child_frame_id='base_link'
            base.pose.pose.position.x,base.pose.pose.position.y,base.pose.pose.position.z=map(float,body)
            base.pose.pose.orientation.x,base.pose.pose.orientation.y,base.pose.pose.orientation.z,base.pose.pose.orientation.w=map(float,qb)
            base.pose.covariance=covariance.reshape(-1).tolist()
            base.twist.twist.linear.x=vx;base.twist.twist.linear.y=vy;base.twist.twist.linear.z=0.
            base.twist.twist.angular.x=base.twist.twist.angular.y=0.;base.twist.twist.angular.z=wz
            base.twist.covariance=[0.]*36
            self.base_pub.publish(base)
            self.report(event='accepted',stamp_s=t,radar_yaw=psi,derived_relative_yaw=theta,
                fixed_body_yaw=float(self.mount.initial_base[3]),base_pose=[*map(float,body),float(self.mount.initial_base[3])],
                source='LIO radar/IMU pose + declared fixed chassis heading; no encoder/GT',
                covariance_scope='conditional exact fixed heading/extrinsics, not calibrated hardware bounds',
                twist_scope='0.2s pose slope; twist covariance uncalibrated')
        except (ValueError,TypeError,AttributeError) as e:
            self.history.clear();self.report(event='LIO_rejected',reason=str(e))

def main():
    rclpy.init();node=RadarFixedOdometry()
    from traced_rmw_executor import TracedRmwExecutor as RmwDiagnosticExecutor
    executor=RmwDiagnosticExecutor();executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
