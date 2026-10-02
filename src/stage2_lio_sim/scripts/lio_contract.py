"""Instantaneous ray-cloud contract; no invented sequential point times."""
import math
import struct
import numpy as np

RAW_CLOUD_FRAME = 'omni_robot/base_link/lidar3d'
RAW_IMU_FRAME = 'omni_robot/base_link/imu3d'

def stamp_s(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('malformed stamp')
    return stamp.sec + stamp.nanosec * 1e-9

def points(message):
    if message.header.frame_id != RAW_CLOUD_FRAME or message.is_bigendian:
        raise ValueError('unexpected point cloud frame/endian')
    fields = {f.name: f for f in message.fields}
    if message.width * message.height < 5 or message.row_step < message.width * message.point_step:
        raise ValueError('empty/malformed cloud dimensions')
    if len(message.data) != message.row_step * message.height:
        raise ValueError('cloud payload size mismatch')
    for key in ('x','y','z'):
        f = fields.get(key)
        if f is None or f.datatype != 7 or f.count != 1 or f.offset+4 > message.point_step:
            raise ValueError('invalid xyz layout')
    xyz = np.empty((message.width * message.height, 3), dtype=np.float32)
    for col, key in enumerate(('x','y','z')):
        view = np.ndarray((message.height,message.width), dtype='<f4', buffer=bytes(message.data),
            offset=fields[key].offset, strides=(message.row_step,message.point_step))
        xyz[:,col] = view.reshape(-1)
    # No-return NaN/inf XYZ rays are omitted, never mapped to the origin.
    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz,axis=1) >= .1)
    result = xyz[valid]
    if len(result) < 24:
        raise ValueError('insufficient finite ray returns')
    return result, int((~valid).sum())

def pack_mid360(xyz, stamp_ns):
    # SPARK MID360 handler expects absolute nanoseconds in a double timestamp.
    # Every GPU ray is sampled at the same stamp: delta/curvature is exactly 0.
    dtype=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],
        'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],
        'offsets':[0,4,8,12,16,17,24],'itemsize':32})
    out=np.zeros(len(xyz),dtype=dtype)
    for i,key in enumerate(('x','y','z')):out[key]=xyz[:,i]
    out['tag']=0x10;out['timestamp']=stamp_ns
    # Intensity and line are synthetic interface values, not reflectance/channels.
    return out.tobytes()

def valid_imu(message):
    if message.header.frame_id != RAW_IMU_FRAME:
        raise ValueError('unexpected IMU frame')
    stamp_s(message.header.stamp)
    values=[getattr(v,k) for v in (message.linear_acceleration,message.angular_velocity) for k in 'xyz']
    if not all(math.isfinite(x) for x in values):raise ValueError('nonfinite IMU')
    return values

def align_pose(x,y,z,yaw,origin=(4.7,.5,math.pi/2)):
    c,s=math.cos(origin[2]),math.sin(origin[2])
    return origin[0]+c*x-s*y,origin[1]+s*x+c*y,z+.15,math.atan2(math.sin(yaw+origin[2]),math.cos(yaw+origin[2]))

def align_imu_to_base(x,y,z,q):
    """Fixed co-located lidar/IMU at base +(0,0,.25); known spawn, no GT."""
    col_x=2*(q.x*q.z+q.w*q.y)
    col_y=2*(q.y*q.z-q.w*q.x)
    col_z=1-2*(q.x*q.x+q.y*q.y)
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    # Local LIO origin is the initial IMU (.40m above physical floor).
    a=align_pose(x-.25*col_x,y-.25*col_y,z+.25-.25*col_z,yaw)
    return a

def body_velocity(previous,current):
    dt=current[0]-previous[0]
    if not 0 < dt <= .2:raise ValueError('invalid odometry differentiation interval')
    yaw=current[3];c,s=math.cos(yaw),math.sin(yaw)
    vx=(current[1]-previous[1])/dt;vy=(current[2]-previous[2])/dt
    return c*vx+s*vy,-s*vx+c*vy,math.atan2(math.sin(yaw-previous[3]),math.cos(yaw-previous[3]))/dt

def window_velocity(history):
    """Causal 0.20s least-squares slope; includes measured correction jumps.

    Never delete outliers or clamp a velocity to satisfy the terminal gate.
    At least 0.10s of actual nonduplicate LIO poses are required.
    """
    a=np.asarray(history,dtype=float)
    if len(a)<3 or a[-1,0]-a[0,0]<.10-1e-9 or not np.isfinite(a).all() or np.any(np.diff(a[:,0])<=0):
        raise ValueError('velocity window incomplete')
    t=a[:,0]-a[-1,0];t=t-t.mean();den=float(t@t)
    vx=float(t@a[:,1]/den);vy=float(t@a[:,2]/den)
    wz=float(t@np.unwrap(a[:,3])/den);c,s=math.cos(a[-1,3]),math.sin(a[-1,3])
    return c*vx+s*vy,-s*vx+c*vy,wz
