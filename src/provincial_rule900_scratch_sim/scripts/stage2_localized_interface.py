#!/usr/bin/env python3
"""Opt-in Stage 2 local scan correction; frozen/raw-drift entries unchanged."""
import copy
from collections import deque
import json
import math
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from stage2_navigation_interface import Stage2NavigationInterface
from stage2_simulated_odometry import extract
from gazebo_navigation_interface import GazeboNavigationInterface, stamp_seconds, quaternion_from_yaw
from stage2_wall_localization import Correction, FaceMatcher, SCAN_FRAME, validate_scan_metadata, verify_lidar_contract


class _LocalizationTaskPaused(Exception):
    """End only the inherited task tail, after stop and odometry publication."""


class LocalizedInterface(Stage2NavigationInterface):
    def __init__(self):
        self.raw_history = deque(maxlen=150)
        self.correction = Correction()
        self.last_match_sim = self.last_match_steady = self.last_scan_stamp = None
        self.scan_input_fault = False
        self._localization_stage_hold = None
        self._localization_update_fresh = None
        super().__init__()
        self.matcher = FaceMatcher(self.field['collision_segments'], self.provincial_wall_thickness)
        self.gain = float(self.declare_parameter('localization_correction_gain', .5).value)
        self.match_timeout = float(self.declare_parameter('localization_match_timeout_s', .5).value)
        if not 0 < self.gain <= 1 or not 0 < self.match_timeout <= .5:
            raise ValueError('invalid localization gain/timeout')
        self.scan_frame = str(self.declare_parameter('localization_scan_frame_id', SCAN_FRAME).value)
        world = str(self.declare_parameter('localization_world_sdf', str(
            Path(get_package_share_directory('uav_bringup'))/'worlds/provincial_2025_training.sdf')).value)
        self.lidar_contract = verify_lidar_contract(world, self.scan_frame)
        self.localized_pub = self.create_publisher(Odometry, '/localization/navigation_odometry', 100)
        self.localization_diag = self.create_publisher(String, '/localization/diagnostics', 100)
        self.get_logger().info('STAGE2 LOCALIZATION: raw injected odom + /scan + known collision faces; NO GT correction')
        self.localization_diag.publish(String(data=json.dumps({'event':'lidar_contract', **self.lidar_contract})))

    def gazebo_odom(self, message):
        try:
            pose, _ = extract(message)
            stamp = stamp_seconds(message.header.stamp)
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if (message.header.frame_id != 'odom' or message.child_frame_id != 'base_link' or
                    not -.05 <= age <= .2 or
                    (self.last_stage2_stamp is not None and stamp <= self.last_stage2_stamp)):
                raise ValueError('invalid/stale/nonmonotonic raw input')
        except ValueError as e:
            self.get_logger().error('LOCALIZED RAW INPUT REJECTED: '+str(e))
            return
        self.last_stage2_stamp = stamp
        self.raw_history.append((stamp, *pose))
        localized = copy.deepcopy(message)
        corrected = self.correction.apply(pose)
        localized.pose.pose.position.x, localized.pose.pose.position.y = corrected[:2]
        localized.pose.pose.orientation = quaternion_from_yaw(corrected[2])
        # Retain raw body twist/covariance. No differentiation of noisy pose,
        # no claim that the inherited covariance is calibrated posterior risk.
        self.localized_pub.publish(localized)
        GazeboNavigationInterface.gazebo_odom(self, localized)

    def scan_callback(self, scan):
        stamp = stamp_seconds(scan.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        report = {'stamp_s': stamp, 'accepted': False}
        try:
            validate_scan_metadata(scan)
            if scan.header.frame_id != self.scan_frame:
                raise ValueError('unexpected scan frame: '+scan.header.frame_id)
            if not 0 <= scan.header.stamp.nanosec < 1_000_000_000 or scan.header.stamp.sec < 0:
                raise ValueError('invalid scan timestamp fields')
        except (ValueError, AttributeError, TypeError) as error:
            # Malformed data cannot register a cloud, renew sensor freshness,
            # or keep motion enabled under the previous good localization.
            self.scan_input_fault = True
            report.update(reason=str(error), input_fault=True)
            self.localization_diag.publish(String(data=json.dumps(report, allow_nan=False)))
            return
        if not self.raw_history or not -.05 <= now-stamp <= .2:
            report['reason'] = 'missing odometry or stale/future scan'
        elif self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            report['reason'] = 'nonmonotonic scan'
        else:
            t, *raw = min(self.raw_history, key=lambda p: abs(p[0]-stamp))
            if abs(t-stamp) > .025:
                report['reason'] = 'scan/odom association exceeds 25ms'
            else:
                prior = self.correction.apply(raw)
                began = time.monotonic()
                report.update(self.matcher.match(prior, scan), raw_pose=raw, associated_odom_stamp_s=t,
                              prior_pose=prior)
                report['compute_wall_s'] = time.monotonic()-began
                if report['accepted']:
                    self.correction.update(raw, report['delta'], self.gain)
                    self.last_match_sim, self.last_match_steady = stamp, time.monotonic()
                    self.scan_input_fault = False
                    # Scan registration must use the same newly corrected frame.
                    self.odom_history = deque(((ts, *self.correction.apply(p[1:]))
                                              for p in self.raw_history for ts in [p[0]]), maxlen=150)
                report['correction_transform'] = self.correction.offset
            self.last_scan_stamp = stamp
        self.localization_diag.publish(String(data=json.dumps(report, allow_nan=False)))
        if not report['accepted']:
            # Timestamp, association and fit failures neither refresh the
            # original scan watchdog nor contaminate obstacle/map state.
            # Existing accepted localization expires on its original clocks.
            return
        super().scan_callback(scan)

    def localization_fresh(self):
        return (not self.scan_input_fault and
                self.last_match_sim is not None and self.last_match_steady is not None and
                0 <= self.get_clock().now().nanoseconds*1e-9-self.last_match_sim <= self.match_timeout and
                time.monotonic()-self.last_match_steady <= self.match_timeout)

    def route_goal(self, goal):
        if not self.localization_fresh():
            self.get_logger().warn('GOAL REJECTED: LOCALIZATION NOT FRESH')
            return
        super().route_goal(goal)

    def _task_localization_fresh(self):
        snapshot = getattr(self, '_localization_update_fresh', None)
        return self.localization_fresh() if snapshot is None else snapshot

    def _stage_identity(self):
        waypoint = self.active_waypoint
        if waypoint is None:
            return None
        p = waypoint.pose.position
        return (self.route_diag_id, getattr(self, 'active_waypoint_diag_id', None),
                id(waypoint), float(p.x), float(p.y))

    def _publish_hold_clock(self, phase, now, hold):
        self.localization_diag.publish(String(data=json.dumps({
            'event': 'localization_task_clock', 'phase': phase,
            'route': hold['identity'][0], 'waypoint': hold['identity'][1],
            'monotonic_s': now, 'sim_s': self.get_clock().now().nanoseconds*1e-9,
            'hold_duration_s': max(0.0, now-hold['started_at']),
            'stage_sent_at_before_hold': hold['sent_at'],
            'progress_at_before_hold': hold['progress_at'],
            'reason': 'localization input unavailable; not executable no-progress',
        }, allow_nan=False)))

    def _sync_stage_clock(self, fresh, now):
        identity = self._stage_identity()
        hold = getattr(self, '_localization_stage_hold', None)
        if hold is not None and hold['identity'] != identity:
            # Never restore an old task's clock into a replacement/next task.
            self._publish_hold_clock('discard_previous_task', now, hold)
            hold = self._localization_stage_hold = None
        if identity is None:
            return
        if not fresh:
            if hold is None:
                hold = self._localization_stage_hold = dict(
                    identity=identity, started_at=now,
                    sent_at=self.stage_sent_at,
                    progress_at=getattr(self, 'stage_last_progress_at', None))
                self._publish_hold_clock('paused', now, hold)
            # Keep inherited time-based approach/retry eligibility frozen.
            # None remains None: an exhausted stage is not revived here.
            if hold['sent_at'] is not None:
                self.stage_sent_at = now
            if hold['progress_at'] is not None:
                self.stage_last_progress_at = now
        elif hold is not None:
            # Give this same task its existing full executable retry window.
            # No goal resend, retry-counter reset, plan reset or threshold change.
            if hold['sent_at'] is not None:
                self.stage_sent_at = now
            if hold['progress_at'] is not None:
                self.stage_last_progress_at = now
            self._publish_hold_clock('resumed', now, hold)
            self._localization_stage_hold = None

    def send_next_waypoint(self):
        if not self._task_localization_fresh():
            # Includes pending-only tasks; keep the queue and guard state intact.
            return
        super().send_next_waypoint()

    def dynamic_waypoint_blocked(self, waypoint):
        if not self._task_localization_fresh():
            # Invalid localization must not requeue a task using old scan points.
            # The independent sensor watchdog still forces final motion to zero.
            return False
        return super().dynamic_waypoint_blocked(waypoint)

    def observe_guard_resume(self, action, now_steady):
        if not self._task_localization_fresh():
            super().observe_guard_resume('stale_stop', now_steady)
            # This frozen-base hook is AFTER final command + odom publication
            # and BEFORE completion/retry/recovery. A private exception returns
            # through our update wrapper without changing shared base code or
            # hiding active_waypoint, and without fabricating a completion log.
            raise _LocalizationTaskPaused()
        super().observe_guard_resume(action, now_steady)

    def update(self):
        received = self.last_odom_received
        fresh = self.localization_fresh()
        self._localization_update_fresh = fresh
        try:
            self._sync_stage_clock(fresh, time.monotonic())
            if not fresh:
                self.last_odom_received = None  # Reuse production immediate sensor watchdog.
            try:
                super().update()
            except _LocalizationTaskPaused:
                pass
        finally:
            self.last_odom_received = received
            self._localization_update_fresh = None


def main():
    rclpy.init()
    node = LocalizedInterface()
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
