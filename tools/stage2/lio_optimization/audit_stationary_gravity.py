"""Read-only gravity evidence in a fixed pre-goal stationary window.

The norm of specific force during acceleration is not the gravity magnitude.
Keep the former whole-recording statistic visible; do not change sensor data,
the 0.2 m/s^2 tolerance, or any navigation acceptance threshold.
"""
import base64
import json
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

WINDOW = (10., 15.)


def audit_rows(ground, kinematics, imu, first_goal):
    checks = {}
    begin, end = WINDOW
    streams = [ground, kinematics, imu]
    checks['fixed_window_precedes_only_goal'] = (
        first_goal is not None and math.isfinite(first_goal) and first_goal > end)
    checks['three_streams_present'] = all(len(s) >= n for s, n in zip(streams, [200, 1000, 1000]))
    checks['finite_and_ordered'] = all(
        all(math.isfinite(v) for v in row) for stream in streams for row in stream)
    checks['continuous_window_coverage'] = all(
        bool(s) and s[0][0] <= begin + tol and s[-1][0] >= end - tol and
        all(0 < b[0] - a[0] <= tol + 1e-9 and 0 < b[1] - a[1] <= .2
            for a, b in zip(s, s[1:]))
        for s, tol in zip(streams, [.04, .008, .008]))
    # Columns: GT(t,receive,x,y,vx,vy,vz,wx,wy,wz), physical(t,receive,vxyz,axyz),
    # IMU(t,receive,axyz). Evidence is entirely offline; nothing is published.
    checks['GT_stationary_at_spawn'] = bool(ground) and all(
        abs(r[2] - 4.7) < 1e-6 and abs(r[3] - .5) < 1e-6 and
        max(abs(v) for v in r[4:]) < .001 for r in ground)
    checks['physical_velocity_and_acceleration_stationary'] = bool(kinematics) and all(
        max(abs(v) for v in r[2:]) < .001 for r in kinematics)
    norms = [math.sqrt(sum(v * v for v in r[2:])) for r in imu]
    checks['every_stationary_specific_force_includes_gravity'] = bool(norms) and all(
        math.isfinite(n) and abs(n - 9.81) < .2 for n in norms)
    return {'pass': all(checks.values()), 'checks': checks,
            'fixed_pre_goal_window_sim_s': list(WINDOW),
            'sample_counts': dict(zip(['GT', 'physical_kinematics', 'IMU'], map(len, streams))),
            'gravity_reference_mps2': 9.81, 'unchanged_tolerance_mps2': .2,
            'stationary_specific_force_mean_mps2': sum(norms)/len(norms) if norms else None,
            'stationary_specific_force_min_mps2': min(norms) if norms else None,
            'stationary_specific_force_max_mps2': max(norms) if norms else None,
            'first_external_goal_sim_s': first_goal,
            'scope': 'Fixed 10..15s pre-goal window, raw GT and physical stationarity required; no moving-norm gravity assumption, no GT feedback.'}


def extract(directory):
    ground, kinematics, imu, goals = [], [], [], []
    with (directory/'lio_full_01.native.jsonl').open() as stream:
        for line in stream:
            r = json.loads(line)
            recv = r['receive_monotonic_ns'] * 1e-9
            if r['topic'] == '/goal_pose': goals.append(r['receive_sim_s'])
            if r['topic'] == '/gazebo/odometry' and WINDOW[0] <= r['message_stamp_s'] <= WINDOW[1]:
                d = r['data']; p = d['pose']['pose']['position']; tw = d['twist']['twist']
                ground.append([r['message_stamp_s'], recv, p['x'], p['y'],
                               *(tw[k][a] for k in ['linear','angular'] for a in 'xyz')])
            elif r['topic'] == '/simulation/imu_kinematics':
                d = json.loads(r['data']['data'])
                if WINDOW[0] <= d['stamp_s'] <= WINDOW[1]:
                    kinematics.append([d['stamp_s'], recv, *d['world_velocity'], *d['world_acceleration']])
    with (directory/'lio_full_01.sensors.jsonl').open() as stream:
        for line in stream:
            r = json.loads(line)
            if r['topic'] == '/simulation/imu3d' and WINDOW[0] <= r['message_stamp_s'] <= WINDOW[1]:
                m = deserialize_message(base64.b64decode(r['cdr'], validate=True), Imu)
                stamp = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
                if abs(stamp-r['message_stamp_s']) > 1e-9: raise ValueError('IMU stamp mismatch')
                imu.append([stamp, r['receive_monotonic_ns']*1e-9,
                            *(getattr(m.linear_acceleration,k) for k in 'xyz')])
    return ground, kinematics, imu, goals[0] if len(goals) == 1 else None


def evaluate(directory):
    return audit_rows(*extract(Path(directory)))
