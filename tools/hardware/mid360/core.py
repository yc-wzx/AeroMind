"""Pure checks for the unmodified Livox driver2 MID-360 PointCloud2/IMU stream."""
import copy
import hashlib
import math
from pathlib import Path
import numpy as np

G = 9.80665
POINTS = '/hardware/mid360/points'
IMU_RAW = '/hardware/mid360/imu_raw'
IMU_SI = '/hardware/mid360/imu_si'
ODOM = '/hardware/mid360/lio/odometry'
FORMATS = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def stamp_ns(message):
    s = message.header.stamp
    if not 0 <= s.nanosec < 1_000_000_000:
        raise ValueError('invalid nanosecond field')
    value = s.sec*1_000_000_000+s.nanosec
    if value <= 0:
        raise ValueError('missing/nonpositive sensor timestamp')
    return value

def imu_values(message):
    acc = [getattr(message.linear_acceleration, a) for a in 'xyz']
    gyro = [getattr(message.angular_velocity, a) for a in 'xyz']
    if not all(math.isfinite(v) for v in acc+gyro):
        raise ValueError('nonfinite IMU acceleration/gyro')
    stamp_ns(message)
    return dict(acc=acc, gyro=gyro, acc_norm=math.sqrt(sum(v*v for v in acc)),
                gyro_norm=math.sqrt(sum(v*v for v in gyro)))

def imu_to_si(message):
    """No timestamp/axis/bias editing. Raw acceleration is g in the pinned driver."""
    imu_values(message)
    out = copy.deepcopy(message)
    out.header.frame_id = 'mid360_imu'
    for axis in 'xyz':
        setattr(out.linear_acceleration, axis, getattr(message.linear_acceleration, axis)*G)
    out.linear_acceleration_covariance = [v*G*G for v in message.linear_acceleration_covariance]
    # The built-in IMU does NOT supply an orientation estimate.
    out.orientation_covariance[0] = -1.0
    return out

def inspect_cloud(message):
    n = message.width*message.height
    if n <= 0 or message.point_step <= 0 or message.row_step < message.point_step*message.width:
        raise ValueError('empty cloud or invalid dimensions/row step')
    if len(message.data) != message.row_step*message.height:
        raise ValueError('cloud byte count does not match dimensions')
    required = {'x':7,'y':7,'z':7,'intensity':7,'tag':2,'line':2,'timestamp':8}
    fields = {f.name:f for f in message.fields}
    if len(fields) != len(message.fields):
        raise ValueError('duplicate cloud field name')
    for name, kind in required.items():
        if name not in fields or fields[name].datatype != kind or fields[name].count != 1:
            raise ValueError('missing/incompatible MID360 field '+name)
    names, formats, offsets = [], [], []
    for name in required:
        f = fields[name]
        fmt = np.dtype(('>' if message.is_bigendian else '<')+FORMATS[f.datatype])
        if f.offset < 0 or f.offset+fmt.itemsize > message.point_step:
            raise ValueError('cloud field offset outside point step')
        names.append(name); formats.append(fmt); offsets.append(f.offset)
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=message.point_step))
    a = np.ndarray((message.height,message.width), dtype=dtype, buffer=message.data,
                   strides=(message.row_step,message.point_step)).reshape(-1)
    for name in ['x','y','z','intensity','timestamp']:
        if not np.all(np.isfinite(a[name])):
            raise ValueError('nonfinite cloud '+name)
    t = a['timestamp']; header = stamp_ns(message)
    if np.any(a['line'] >= 4):
        raise ValueError('unexpected MID360 line id (expected 0..3)')
    if abs(float(t[0])-header) > 1_000_000:
        raise ValueError('first-point nanosecond timestamp disagrees with cloud header')
    span = (float(np.max(t))-float(np.min(t)))*1e-9
    if span <= 0 or span > .2:
        raise ValueError('invalid point time span; cannot assume instantaneous simulation cloud')
    usable = (((a['tag'] & 0x30)==0x10)|((a['tag'] & 0x30)==0))
    usable &= a['x']**2+a['y']**2+a['z']**2 >= .5**2
    if not np.any(usable):
        raise ValueError('no usable returns beyond upstream .5m blind range')
    return dict(points=n, usable_points=int(np.count_nonzero(usable)), point_span_s=span,
                within_cloud_regressions=int(np.count_nonzero(np.diff(t)<0)),
                first_point_ns=float(t[0]), last_point_ns=float(t[-1]))

def stream_stats(rows, required_span_s=10.):
    if len(rows) < 2:
        return dict(pass_=False, count=len(rows), problem='missing/insufficient messages')
    stamps = [r['stamp_ns'] for r in rows if r.get('stamp_ns') is not None]
    receive = [r['receive_monotonic_ns'] for r in rows]
    errors = sum(r.get('error') is not None for r in rows)
    if len(stamps) != len(rows):
        errors += len(rows)-len(stamps)
    ds = np.diff(stamps)*1e-9; dr = np.diff(receive)*1e-9
    span = (receive[-1]-receive[0])*1e-9
    return dict(pass_=not errors and len(stamps)==len(rows) and bool(np.all(ds>0)) and
                bool(np.all(dr>=0)) and span>=required_span_s,
                count=len(rows), valid_rows=len(rows)-sum(r.get('error') is not None for r in rows),
                errors=errors, observed_span_s=span, receive_hz=(len(rows)-1)/span if span>0 else None,
                sensor_span_s=(stamps[-1]-stamps[0])*1e-9 if len(stamps)>1 else None,
                max_receive_gap_s=float(max(dr)) if len(dr) else None,
                max_sensor_gap_s=float(max(ds)) if len(ds) else None,
                duplicate_stamps=int(np.count_nonzero(ds==0)), regressed_stamps=int(np.count_nonzero(ds<0)))
