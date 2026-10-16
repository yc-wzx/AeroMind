"""Radar pose and fixed physical chassis heading are different quantities.

No encoder or GT input. The constant chassis heading is an explicit mechanical
assumption, not an observed yaw; real body rotation is not observable here.
"""
from runtime_support import load_existing
load_existing()
import math
import numpy as np
from gimbal_contract import Mount,JointBuffer,rotation,rz,quaternion
from lio_contract import stamp_s

MAX_TILT_RAD=.03

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

def planar_yaw(R):
    tilt=math.acos(float(np.clip(R[2,2],-1.,1.)))
    if tilt>MAX_TILT_RAD:raise ValueError('yaw-only mount tilt exceeds declared model')
    return math.atan2(R[1,0],R[0,0])

class FixedHeadingMount(Mount):
    def from_lio(self,position,q):
        p=np.asarray(position,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('invalid LIO position')
        Ri=self.R0@rotation(q);sensor_yaw=planar_yaw(Ri)
        theta=wrap(sensor_yaw-self.initial_base[3]);Rb=rz(self.initial_base[3])
        sensor=self.p0+self.R0@p
        body=sensor-Rb@self.sensor_offset(theta)
        return body,quaternion(Rb),sensor,sensor_yaw,theta

    def body_covariance(self,q,covariance):
        C=np.asarray(covariance,dtype=float).reshape(6,6)
        if not np.isfinite(C).all() or np.any(np.diag(C)<0):raise ValueError('invalid LIO covariance')
        Ri=self.R0@rotation(q);psi=planar_yaw(Ri);theta=wrap(psi-self.initial_base[3])
        axis=Ri[:,0];den=float(axis[0]**2+axis[1]**2)
        yaw_gradient=np.array([-axis[0]*axis[2]/den,-axis[1]*axis[2]/den,1.])@self.R0
        lever=rz(theta)@self.lever
        derivative=-rz(self.initial_base[3])@np.cross([0.,0.,1.],lever)
        J=np.zeros((6,6));J[:3,:3]=self.R0;J[:3,3:]=np.outer(derivative,yaw_gradient)
        # Conditional on exact initial physical heading and mount calibration.
        # Zero orientation covariance is NOT a measured hardware error bound.
        return J@C@J.T

def configured_mount(node):
    if not node.declare_parameter('fixed_chassis_heading_confirmed',False).value:
        raise ValueError('fixed physical chassis heading must be explicitly declared')
    return FixedHeadingMount(node.declare_parameter('initial_base',[4.7,.5,.15,math.pi/2]).value,
        node.declare_parameter('pivot',[.06,0.,.25]).value,
        node.declare_parameter('lever',[.03,0.,0.]).value,
        node.declare_parameter('initial_joint',0.).value)

class RadarHeadingBuffer(JointBuffer):
    """Legacy numeric angle-buffer API backed ONLY by LIO-derived radar poses.

No JointState subscription/publication. Angle is derived radar yaw minus the
explicit fixed body yaw; angular rate is a finite difference, not raw IMU gyro.
"""
    def __init__(self,base_yaw):
        super().__init__();self.base_yaw=float(base_yaw)

    def add_pose(self,msg,now,received):
        try:
            if msg.header.frame_id!='odom' or msg.child_frame_id!='radar_control':raise ValueError('wrong radar pose frame')
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            if not np.isfinite([p.x,p.y,p.z,*msg.pose.covariance]).all():raise ValueError('nonfinite radar pose')
            a=wrap(planar_yaw(rotation([q.x,q.y,q.z,q.w]))-self.base_yaw)
            t=stamp_s(msg.header.stamp);rate=0.
            if self.samples:
                old=self.samples[-1]
                if t<=old[0]:raise ValueError('nonmonotonic radar pose')
                a=old[1]+wrap(a-old[1]);rate=(a-old[1])/(t-old[0])
            self.add(t,a,rate,received,now)
        except (ValueError,TypeError,AttributeError):
            self.fault=True;raise
