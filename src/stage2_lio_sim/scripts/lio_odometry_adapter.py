#!/usr/bin/env python3
"""Fixed known-spawn alignment and estimated-pose finite difference. Never GT."""
import copy,json,math
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from lio_contract import stamp_s,align_imu_to_base,window_velocity

class OdometryAdapter(Node):
    def __init__(self):
        super().__init__('lio_odometry_adapter');self.history=deque();self.last=None
        self.pub=self.create_publisher(Odometry,'/localization/lio_navigation_odometry',100)
        self.diag=self.create_publisher(String,'/lio/adapter_diagnostics',100)
        self.create_subscription(Odometry,'/lio/odometry',self.callback,100)

    def callback(self,msg):
        try:
            t=stamp_s(msg.header.stamp);p=msg.pose.pose.position;q=msg.pose.pose.orientation
            values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]+list(msg.pose.covariance)
            age=self.get_clock().now().nanoseconds*1e-9-t
            if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1)>.01:
                raise ValueError('nonfinite/invalid LIO pose')
            if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='lio_imu' or not -.05<=age<=.2:
                raise ValueError('frame or stale/future LIO odometry')
            if self.last is not None and t<=self.last:
                # Two valid paths in SPARK may publish the same correction stamp.
                # Skip duplicates, do not fabricate extra/newer samples.
                self.diag.publish(String(data=json.dumps({'event':'duplicate_or_older','stamp_s':t})));return
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            x,y,z,a=align_imu_to_base(p.x,p.y,p.z,q);current=(t,x,y,a)
            if self.last is not None and t-self.last>.2:self.history.clear()
            self.last=t;self.history.append(current)
            while self.history and t-self.history[0][0]>.2+1e-9:self.history.popleft()
            try:vx,vy,wz=window_velocity(self.history)
            except ValueError:
                self.diag.publish(String(data=json.dumps({'event':'velocity_initializing','stamp_s':t})));return
            out=copy.deepcopy(msg);out.header.frame_id='odom';out.child_frame_id='base_link'
            out.pose.pose.position.x=x;out.pose.pose.position.y=y;out.pose.pose.position.z=z
            # Retain full roll/pitch; compose constant yaw rotation with LIO quaternion.
            s,c=math.sin(math.pi/4),math.cos(math.pi/4)
            out.pose.pose.orientation.x=c*q.x-s*q.y;out.pose.pose.orientation.y=s*q.x+c*q.y
            out.pose.pose.orientation.z=c*q.z+s*q.w;out.pose.pose.orientation.w=c*q.w-s*q.z
            out.twist.twist.linear.x=vx;out.twist.twist.linear.y=vy;out.twist.twist.linear.z=0.
            out.twist.twist.angular.x=out.twist.twist.angular.y=0.;out.twist.twist.angular.z=wz
            # Covariance belongs to LIO local frame. Rotate pose axes into map axes.
            import numpy as np
            R=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
            J=np.zeros((6,6));J[:3,:3]=J[3:,3:]=R
            out.pose.covariance=(J@np.array(msg.pose.covariance).reshape(6,6)@J.T).reshape(-1).tolist()
            # Derivative covariance not calibrated: an unknown block, explicitly reported.
            out.twist.covariance=[0.]*36
            self.pub.publish(out)
            self.diag.publish(String(data=json.dumps({'event':'accepted','stamp_s':t,'pose':[x,y,z,a],
                'body_twist':[vx,vy,wz],'velocity_source':'causal 0.20s LIO pose least-squares slope',
                'velocity_window_s':t-self.history[0][0],'velocity_window_samples':len(self.history),
                'twist_covariance_calibrated':False,'origin_source':'fixed declared spawn, not GT'})))
        except (ValueError,TypeError,AttributeError) as error:
            self.diag.publish(String(data=json.dumps({'event':'rejected','reason':str(error)})))

def main():
    rclpy.init();node=OdometryAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
