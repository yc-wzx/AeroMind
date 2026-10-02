"""Planning-only obstacle envelope and sensor-relative sampled command guard.

Neither function changes the arena nor estimates/corrects a global pose.
These engineering reserves are not calibrated confidence bounds.
"""
import math
import numpy as np


def planning_envelope(segments, thickness=.055, padding=.075, spacing=.025):
    if not all(math.isfinite(v) and v > 0 for v in (thickness, padding, spacing)):
        raise ValueError('invalid planning envelope')
    points = []
    for x0, y0, x1, y1 in segments:
        length = math.hypot(x1-x0, y1-y0)
        if length <= 0:
            raise ValueError('zero length wall')
        ux, uy = (x1-x0)/length, (y1-y0)/length
        half = thickness/2 + padding
        # Sample expanded wall perimeter, including caps. Original physical
        # wall and map remain unchanged; EGO's existing .22 inflation follows.
        corners = [(x0-ux*padding-uy*s*half, y0-uy*padding+ux*s*half)
                   for s in (-1, 1)] + [
                   (x1+ux*padding-uy*s*half, y1+uy*padding+ux*s*half)
                   for s in (1, -1)]
        for a, b in zip(corners, corners[1:]+corners[:1]):
            count = max(1, math.ceil(math.dist(a, b)/spacing))
            points.extend((a[0]+(b[0]-a[0])*i/count,
                           a[1]+(b[1]-a[1])*i/count, 0.) for i in range(count+1))
    return points


def scan_points(scan):
    if (scan.header.frame_id != 'omni_robot/base_link/lidar' or
            len(scan.ranges) != 360 or
            not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_max,
                 scan.angle_increment, scan.range_min, scan.range_max)) or
            scan.angle_increment <= 0 or scan.range_min <= 0 or
            scan.range_max <= scan.range_min or
            abs(scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment-scan.angle_max) > .001):
        raise ValueError('scan metadata/known zero-XY extrinsic mismatch')
    ranges = np.asarray(scan.ranges, dtype=float)
    if np.any(np.isnan(ranges)) or np.any(np.isneginf(ranges)):
        raise ValueError('invalid scan ranges')
    valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
    if np.count_nonzero(valid) < 24:
        raise ValueError('insufficient scan returns')
    angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
    return np.column_stack((ranges[valid]*np.cos(angles[valid]),
                            ranges[valid]*np.sin(angles[valid])))


def sensor_command_gap(points, scan_pose, current_pose, command,
                       horizon=.25, step=.025):
    """Raw returns shifted using only measured LIO increments, not GT.

    A sampled short-horizon check; sparse rays, noise, unsensed objects and
    motion-model error prevent a continuous-time or universal guarantee.
    """
    values = [*scan_pose, *current_pose, *command]
    if not all(math.isfinite(v) for v in values) or not np.isfinite(points).all():
        raise ValueError('nonfinite command/pose/returns')
    sx, sy, sa = scan_pose
    x, y, yaw = current_pose
    c, s = math.cos(sa), math.sin(sa)
    world = points @ np.array([[c, s], [-s, c]]) + [sx, sy]
    vx, vy, wz = command
    minimum = math.inf
    count = math.ceil(horizon/step)
    dt = horizon/count
    for index in range(count+1):
        c, s = math.cos(yaw), math.sin(yaw)
        local = (world-[x, y]) @ np.array([[c, -s], [s, c]])
        outside = np.maximum(np.abs(local)-[.26, .21], 0.)
        minimum = min(minimum, float(np.min(np.linalg.norm(outside, axis=1))))
        mid = yaw+wz*dt/2
        x += (math.cos(mid)*vx-math.sin(mid)*vy)*dt
        y += (math.sin(mid)*vx+math.cos(mid)*vy)*dt
        yaw += wz*dt
    return minimum
