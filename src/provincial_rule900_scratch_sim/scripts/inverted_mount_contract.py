#!/usr/bin/env python3
"""Raw down-facing MID-360 -> virtual upright sensor frames, at unchanged origins.

Native +X must align with gimbal forward. Mount transform is Rx(pi), not the
physical lidar/IMU internal transform. No heading, pose or point time is invented.
"""
import copy
from array import array
import math
import numpy as np

UPRIGHT_FROM_RAW = np.diag([1., -1., -1.])
STANDARD_GRAVITY = 9.80665


def stamp_seconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('malformed message timestamp')
    return stamp.sec + stamp.nanosec * 1e-9


def validate_frames(raw, upright):
    if not raw or not upright or raw == upright:
        raise ValueError('raw and upright frames must be distinct and nonempty')


def transform_cloud(message, raw_frame, upright_frame):
    """Preserve organized layout, padding, all other fields and all point times."""
    validate_frames(raw_frame, upright_frame)
    if message.header.frame_id != raw_frame:
        raise ValueError('wrong raw cloud frame; do not rotate an upright cloud twice')
    stamp_seconds(message.header.stamp)
    if message.width < 1 or message.height < 1 or message.point_step < 1:
        raise ValueError('empty or malformed cloud dimensions')
    if message.row_step < message.width * message.point_step:
        raise ValueError('cloud row_step smaller than payload')
    if len(message.data) != message.height * message.row_step:
        raise ValueError('cloud payload size mismatch')
    if len({f.name for f in message.fields}) != len(message.fields):
        raise ValueError('duplicate cloud field names')
    fields = {f.name: f for f in message.fields}
    ranges = []
    views = {}
    payload = bytearray(message.data)
    for axis in 'xyz':
        f = fields.get(axis)
        if f is None or f.count != 1 or f.datatype not in (7, 8):
            raise ValueError('xyz requires scalar FLOAT32 or FLOAT64 fields')
        size = 4 if f.datatype == 7 else 8
        if f.offset < 0 or f.offset + size > message.point_step:
            raise ValueError('xyz field outside point_step')
        ranges.append((f.offset, f.offset + size))
        dtype = ('>' if message.is_bigendian else '<') + ('f4' if size == 4 else 'f8')
        views[axis] = np.ndarray((message.height, message.width), dtype=dtype,
            buffer=payload, offset=f.offset, strides=(message.row_step, message.point_step))
    for i, a in enumerate(ranges):
        for b in ranges[i+1:]:
            if max(a[0], b[0]) < min(a[1], b[1]):
                raise ValueError('overlapping xyz fields')
    sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    for name, field in fields.items():
        if field.datatype not in sizes or field.count < 1:
            raise ValueError('malformed non-xyz field layout')
        span = (field.offset, field.offset + sizes[field.datatype] * field.count)
        if span[0] < 0 or span[1] > message.point_step:
            raise ValueError('cloud field outside point_step')
        if name not in ('x', 'y', 'z') and any(max(span[0], a[0]) < min(span[1], a[1]) for a in ranges):
            raise ValueError('metadata overlaps xyz; cannot preserve non-xyz payload')
    valid = np.isfinite(views['x']) & np.isfinite(views['y']) & np.isfinite(views['z'])
    finite_count = int(valid.sum())
    if finite_count == 0:
        raise ValueError('cloud contains no finite xyz observations')
    # Missing returns remain missing; no origin substitution or resampling.
    views['y'] *= -1.
    views['z'] *= -1.
    out = copy.deepcopy(message)
    out.header.frame_id = upright_frame
    # Typed uint8 storage is byte-identical and avoids generated ROS setters
    # validating each of several hundred thousand bytes in Python.
    out.data = array('B', payload)
    if finite_count != message.width * message.height:
        out.is_dense = False
    return out, {'points': message.width * message.height,
                 'finite_xyz_points': finite_count,
                 'nonfinite_xyz_points': message.width * message.height - finite_count}


def rotate_covariance(values, scale=1.):
    a = np.asarray(values, dtype=float)
    if a.shape != (9,) or not np.isfinite(a).all():
        raise ValueError('nonfinite or malformed IMU covariance')
    if a[0] == -1.:
        return list(map(float, a))  # ROS "unavailable" sentinel, not a covariance.
    P = a.reshape(3, 3)
    if not np.allclose(P, P.T, atol=1e-10, rtol=0) or np.linalg.eigvalsh(P).min() < -1e-10:
        raise ValueError('IMU covariance is not positive semidefinite/symmetric')
    with np.errstate(over='ignore', invalid='ignore'):
        result = scale**2 * UPRIGHT_FROM_RAW @ P @ UPRIGHT_FROM_RAW.T
    if not np.isfinite(result).all():
        raise ValueError('IMU covariance conversion overflow')
    return result.reshape(-1).tolist()


def transform_imu(message, raw_frame, upright_frame, acceleration_unit='g'):
    validate_frames(raw_frame, upright_frame)
    if message.header.frame_id != raw_frame:
        raise ValueError('wrong raw IMU frame; do not rotate upright IMU twice')
    stamp_seconds(message.header.stamp)
    if acceleration_unit not in ('g', 'mps2'):
        raise ValueError('input acceleration unit must be g or mps2')
    scale = STANDARD_GRAVITY if acceleration_unit == 'g' else 1.
    angular = np.array([getattr(message.angular_velocity, a) for a in 'xyz'])
    accel = np.array([getattr(message.linear_acceleration, a) for a in 'xyz'])
    if not np.isfinite(angular).all() or not np.isfinite(accel).all():
        raise ValueError('nonfinite IMU gyro/acceleration')
    out = copy.deepcopy(message)
    out.header.frame_id = upright_frame
    for a, value in zip('xyz', UPRIGHT_FROM_RAW @ angular):
        setattr(out.angular_velocity, a, float(value))
    with np.errstate(over='ignore', invalid='ignore'):
        normalized_accel = scale * UPRIGHT_FROM_RAW @ accel
    if not np.isfinite(normalized_accel).all():
        raise ValueError('IMU acceleration conversion overflow')
    for a, value in zip('xyz', normalized_accel):
        setattr(out.linear_acceleration, a, float(value))
    out.angular_velocity_covariance = rotate_covariance(message.angular_velocity_covariance)
    out.linear_acceleration_covariance = rotate_covariance(message.linear_acceleration_covariance, scale)
    # MID-360 driver publishes gyro/acc, not a measured absolute orientation.
    out.orientation.x = out.orientation.y = out.orientation.z = out.orientation.w = 0.
    out.orientation_covariance = [-1.] + [0.] * 8
    return out


def normalized_internal_extrinsics(raw_translation, raw_rotation):
    """p_I = R_LI p_L + T_LI, with BOTH coordinate bases rotated by Rx(pi)."""
    T = np.asarray(raw_translation, dtype=float)
    R = np.asarray(raw_rotation, dtype=float)
    if T.shape != (3,) or R.shape != (9,) or not np.isfinite(np.r_[T, R]).all():
        raise ValueError('explicit finite raw lidar->IMU T[3] and R[9] required')
    R = R.reshape(3, 3)
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-8, rtol=0) or abs(np.linalg.det(R) - 1.) > 1e-8:
        raise ValueError('raw lidar->IMU rotation must be a proper rotation')
    return (UPRIGHT_FROM_RAW @ T).tolist(), (UPRIGHT_FROM_RAW @ R @ UPRIGHT_FROM_RAW.T).reshape(-1).tolist()


def confirmed_lio_extrinsics(enabled, confirmed, translation, rotation):
    if not enabled:
        return None
    if not confirmed:
        raise ValueError('LIO disabled: real lidar/IMU extrinsics must be explicitly provided and confirmed')
    return normalized_internal_extrinsics(translation, rotation)
