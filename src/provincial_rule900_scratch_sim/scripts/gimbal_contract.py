"""Yaw-only moving radar mount. All velocities refer to the chassis origin.

Joint feedback is measured, timestamped and fail-closed. No GT is imported.
The pivot/lever arm and initial pose below are simulation declarations, not
hardware calibration. A pitched gimbal needs an additional joint transform.
"""
import math
from collections import deque
import numpy as np

def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(q@q-1.) > .01:
        raise ValueError('invalid quaternion')
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def quaternion(R):
    # Stable for rotations through +/-180 degrees, including roll/pitch.
    K = np.array([[R[0,0]-R[1,1]-R[2,2], R[1,0]+R[0,1], R[2,0]+R[0,2], R[2,1]-R[1,2]],
                  [R[1,0]+R[0,1], R[1,1]-R[0,0]-R[2,2], R[2,1]+R[1,2], R[0,2]-R[2,0]],
                  [R[2,0]+R[0,2], R[2,1]+R[1,2], R[2,2]-R[0,0]-R[1,1], R[1,0]-R[0,1]],
                  [R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1], np.trace(R)]])/3.
    _, V = np.linalg.eigh(K)
    q = V[:, -1]
    return -q if q[3] < 0 else q

class Mount:
    def __init__(self, initial_base=(4.7, .5, .15, math.pi/2),
                 pivot=(.06, 0., .25), lever=(.03, 0., 0.), initial_joint=0.):
        self.initial_base = np.asarray(initial_base, dtype=float)
        self.pivot, self.lever = np.asarray(pivot, dtype=float), np.asarray(lever, dtype=float)
        if (self.initial_base.shape != (4,) or self.pivot.shape != (3,) or self.lever.shape != (3,)
                or not np.isfinite(np.r_[self.initial_base,self.pivot,self.lever,initial_joint]).all()):
            raise ValueError('invalid mount calibration')
        self.initial_joint = float(initial_joint)
        self.R0 = rz(self.initial_base[3]+initial_joint)
        self.p0 = self.initial_base[:3]+rz(self.initial_base[3])@self.sensor_offset(initial_joint)

    def sensor_offset(self, angle):
        return self.pivot+rz(angle)@self.lever

    def base_pose(self, local_position, local_quaternion, joint_angle):
        p = np.asarray(local_position, dtype=float)
        if p.shape != (3,) or not np.isfinite(np.r_[p,joint_angle]).all():
            raise ValueError('nonfinite LIO position or joint')
        Ri = self.R0@rotation(local_quaternion)
        Rb = Ri@rz(-joint_angle)
        pb = self.p0+self.R0@p-Rb@self.sensor_offset(joint_angle)
        return pb, quaternion(Rb), math.atan2(Rb[1,0],Rb[0,0])

    def sensor_planar_pose(self, base, joint):
        x,y,yaw = base
        off = rz(yaw)@self.sensor_offset(joint)
        return x+off[0], y+off[1], yaw+joint

def body_to_radar(vx, vy, joint):
    if not all(math.isfinite(v) for v in (vx,vy,joint)):
        raise ValueError('nonfinite velocity/angle')
    c,s = math.cos(joint),math.sin(joint)
    return c*vx+s*vy, -s*vx+c*vy

def radar_to_body(vx, vy, joint):
    return body_to_radar(vx,vy,-joint)

class JointBuffer:
    def __init__(self):
        self.samples = deque(maxlen=1000)
        self.fault = True

    def add(self, stamp, angle, rate, received, now):
        if (not all(math.isfinite(v) for v in (stamp,angle,rate,received,now))
                or not -.05 <= now-stamp <= .2
                or (self.samples and stamp <= self.samples[-1][0])):
            self.fault = True
            raise ValueError('invalid/stale/nonmonotonic joint feedback')
        self.samples.append((stamp,angle,rate,received))
        self.fault = False

    def latest(self, now, steady, age=.05):
        if self.fault or not self.samples:
            raise ValueError('joint unavailable')
        t,a,v,r = self.samples[-1]
        if not -.01 <= now-t <= age or not 0 <= steady-r <= .2:
            raise ValueError('joint feedback stale')
        return t,a,v

    def at(self, stamp, now, steady):
        self.latest(now,steady,age=.2)
        if not math.isfinite(stamp) or not -.05 <= now-stamp <= .2:
            raise ValueError('invalid association time')
        data = list(self.samples)
        for row in reversed(data):
            if abs(row[0]-stamp) < 1e-8:
                return row[1],row[2]
        for a,b in zip(data,data[1:]):
            if a[0] < stamp < b[0]:
                if b[0]-a[0] > .025:
                    raise ValueError('joint association gap exceeds25ms')
                f = (stamp-a[0])/(b[0]-a[0])
                # Unwrapped encoder position; never interpolate through wrap.
                return a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2])
        raise ValueError('joint timestamp not bracketed; no extrapolation')

def command_allowed(command, received, now_steady, now_sim, joints):
    if (command is None or received is None or not 0 <= now_steady-received <= .2
            or not all(math.isfinite(v) for v in command)):
        raise ValueError('command missing/stale/nonfinite')
    return joints.latest(now_sim,now_steady)[1]
