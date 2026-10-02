#!/usr/bin/env python3
"""Opt-in LIO planner obstacle envelope + independent raw-scan stop gate."""
import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from lio_navigation_interface import LioNavigationInterface
from gazebo_navigation_interface import stamp_seconds
from provincial_safety_geometry import predict_command_gap
from lio_safety_geometry import planning_envelope, scan_points, sensor_command_gap


class _DriveGate:
    def __init__(self, owner, publisher):
        self.owner, self.publisher = owner, publisher

    def publish(self, command):
        self.publisher.publish(self.owner.check_sensor_command(command))


class LioSafeNavigationInterface(LioNavigationInterface):
    def __init__(self):
        self.safety_scan = None
        self.safety_scan_fault = True
        self.last_sensor_diag = -math.inf
        super().__init__()
        self.planning_padding = float(self.declare_parameter('lio_planning_wall_padding_m', .075).value)
        self.sensor_reserve = float(self.declare_parameter('lio_sensor_gap_reserve_m', .035).value)
        self.pose_safety_margin = float(self.declare_parameter('lio_pose_safety_margin_m', .04).value)
        self.sensor_gap_mode = str(self.declare_parameter('lio_sensor_gap_mode', 'diagnostic_only').value)
        if (self.planning_padding != .075 or self.sensor_reserve != .035 or
                self.pose_safety_margin != .04 or self.sensor_gap_mode != 'diagnostic_only'):
            raise ValueError('unvalidated LIO safety profile')
        self.prior_points = planning_envelope(self.field['collision_segments'],
            self.provincial_wall_thickness, self.planning_padding)
        self.field_points = list(self.prior_points)
        self.sensor_safety_pub = self.create_publisher(String, '/lio/safety_diagnostics', 100)
        self.drive_pub = _DriveGate(self, self.drive_pub)
        self.get_logger().info('LIO safety: planning-only .075m wall envelope; .12m map rectangle reserve + explicit timing budget; raw-scan gap diagnostic only, scan validity remains a stop gate')

    def planned_trajectory_callback(self, marker):
        super().planned_trajectory_callback(marker)
        if self.planned_path_min_gap is not None:
            self.planned_path_safe = self.planned_path_min_gap >= self.provincial_min_body_gap+self.pose_safety_margin

    def observe_guard_resume(self, action, now_steady):
        effective = ('unsafe_final_command' if getattr(self, 'last_sensor_action', 'allowed') != 'allowed' else action)
        super().observe_guard_resume(effective, now_steady)

    def scan_callback(self, scan):
        stamp = stamp_seconds(scan.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        try:
            points = scan_points(scan)
            if not self.odom_history or not -.05 <= now-stamp <= .2:
                raise ValueError('missing LIO or stale/future scan')
            matched = min(self.odom_history, key=lambda p: abs(p[0]-stamp))
            if abs(matched[0]-stamp) > .025:
                raise ValueError('scan/LIO association exceeds25ms')
            if self.safety_scan is not None and stamp <= self.safety_scan['stamp']:
                raise ValueError('nonmonotonic scan')
            self.safety_scan = dict(points=points, pose=tuple(matched[1:]),
                                    pose_stamp=matched[0], stamp=stamp, received=time.monotonic())
            self.safety_scan_fault = False
        except (ValueError, AttributeError, TypeError) as error:
            self.safety_scan_fault = True
            self.get_logger().warn('LIO SENSOR GUARD REJECTED: '+str(error))
            return
        super().scan_callback(scan)

    def check_sensor_command(self, command):
        now = self.get_clock().now().nanoseconds*1e-9
        steady = time.monotonic()
        observation = self.safety_scan
        values = (command.linear.x, command.linear.y, command.angular.z)
        action, gap, age = 'allowed', None, None
        map_gap, required_map_gap, pose_age = None, None, None
        if not all(math.isfinite(v) for v in values):
            action = 'nonfinite_command_stop'
        elif (self.safety_scan_fault or observation is None or
              not 0 <= now-observation['stamp'] <= .2 or
              steady-observation['received'] > .2):
            action = 'scan_unavailable_stop'
        else:
            age = now-observation['stamp']
            try:
                gap = sensor_command_gap(observation['points'], observation['pose'],
                                         (self.x, self.y, self.yaw), values)
                # Point minimum is noise biased in this narrow corridor. Keep
                # it visible as a SHADOW diagnostic; never silently call it a
                # passed sensor-distance gate. Map footprint gate below is
                # strictly stronger than the frozen .08m gate.
                if self.estimate is None or self.last_stage2_stamp is None:
                    action = 'pose_unavailable_stop'
                elif not all(math.isfinite(v) for v in self.estimate):
                    action = 'nonfinite_pose_stop'
                else:
                    pose_age = now-self.last_stage2_stamp
                    if not -.05 <= pose_age <= .05:
                        action = 'pose_age_stop'
                    else:
                        map_gap = predict_command_gap(*self.estimate, *values, self.provincial_walls)
                        motion = math.hypot(values[0], values[1])+math.hypot(.26,.21)*abs(values[2])
                        # Engineering timing assumptions: original20ms update
                        # plus half25ms swept-sample interval. Not a guaranteed
                        # localization bound or continuous-time safety proof.
                        required_map_gap = self.provincial_min_body_gap+self.pose_safety_margin+motion*(max(0.,pose_age)+.02+.0125)
                        if map_gap < required_map_gap:
                            action = 'reserved_map_gap_stop'
            except ValueError:
                action = 'nonfinite_geometry_stop'
        result = command if action == 'allowed' else Twist()
        # No slowing, axis projection or substituted path: rejected is zero.
        if steady-self.last_sensor_diag >= .05 or action != getattr(self, 'last_sensor_action', None):
            self.sensor_safety_pub.publish(String(data=json.dumps({
                'sim_s': now, 'monotonic_s': steady, 'action': action,
                'scan_stamp_s': None if observation is None else observation['stamp'],
                'scan_age_s': age, 'LIO_pose_stamp_s': self.last_stage2_stamp,
                'scan_associated_LIO_pose': None if observation is None else list(observation['pose']),
                'scan_associated_LIO_stamp_s': None if observation is None else observation.get('pose_stamp'),
                'guard_current_pose': [v if math.isfinite(v) else repr(v) for v in (self.x, self.y, self.yaw)],
                'LIO_pose_age_s': None if self.last_stage2_stamp is None else now-self.last_stage2_stamp,
                'sampled_sensor_gap_m': gap,
                'required_sensor_gap_m': self.provincial_min_body_gap+self.sensor_reserve,
                'raw_sensor_gap_mode': self.sensor_gap_mode,
                'raw_sensor_gap_would_stop': None if gap is None else gap < self.provincial_min_body_gap+self.sensor_reserve,
                'latest_LIO_pose_for_map_guard': None if self.estimate is None else [v if math.isfinite(v) else repr(v) for v in self.estimate],
                'sampled_reserved_map_gap_m': map_gap,
                'required_map_gap_m': required_map_gap,
                'map_pose_age_s': pose_age,
                'requested': [v if math.isfinite(v) else repr(v) for v in values],
                'final': [result.linear.x, result.linear.y, result.angular.z],
                'waypoint': self.active_waypoint_diag_id,
                'pending': len(self.waypoints),
                'scope': 'Latest LIO rectangle map guard; raw scan gap SHADOW only, validity stops retained; no GT or pose correction; engineering reserve not certified error bound'
            }, allow_nan=False)))
            self.last_sensor_diag = steady
        self.last_sensor_action = action
        return result


def main():
    rclpy.init()
    node = LioSafeNavigationInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
