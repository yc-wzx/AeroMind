#!/usr/bin/env python3
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log


ROOT = Path("/home/rmnav/AeroMind")
sys.path.insert(0, str(ROOT / "src/uav_planning/scripts"))
from grid_route import GridRoute
OUT = ROOT / "tools" / "results"


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z))


class LongRangeDiagnosticRunner(Node):
    def __init__(self, name, goal_x, goal_y, timeout):
        super().__init__("stage15_long_range_diagnostic_runner")
        self.name = name
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.timeout = timeout
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(Odometry, "/gazebo/odometry", self.gt_cb, 30)
        self.create_subscription(Odometry, "/ground/odometry", self.odom_cb, 30)
        self.create_subscription(PoseStamped, "/ground/planning/commanded_setpoint",
                                 self.setpoint_cb, 30)
        self.create_subscription(Log, "/rosout", self.log_cb, 100)
        self.gt = None
        self.odom = None
        self.setpoint = None
        self.active = False
        self.route_accepted = False
        self.start_wall = None
        self.start_epoch = None
        self.start_sim = None
        self.last_sample_sim = -1e9
        self.samples = []
        self.events = []
        self.event_counts = {}
        self.last_event_by_type = {}
        self.stage_count = 0
        self.retry_count = 0
        self.recovery_count = 0
        self.dynamic_skip_count = 0
        self.preflight_raw_grid_path = []
        self.preflight_waypoints = []
        self.preflight_waypoint_clearances = []
        self.preflight_segment_clearances = []

    def gt_cb(self, msg):
        self.gt = msg
        if self.active:
            self.record_if_due(msg)

    def odom_cb(self, msg):
        self.odom = msg

    def setpoint_cb(self, msg):
        self.setpoint = msg

    def log_cb(self, msg):
        text = msg.msg
        keys = (
            "RMUC route diagnostic", "RMUC waypoint diagnostic",
            "RMUC dynamic skip diagnostic", "RMUC dynamic requeue diagnostic",
            "RMUC retry diagnostic", "RMUC recovery diagnostic",
            "RMUC mission accepted", "Navigation stage:", "Retrying RMUC stage",
            "Skipping ", "stage made no progress", "stage exhausted its retries",
            "terminal point of the current trajectory is in obstacle",
            "First 3 control points in obstacles", "EGO_DIAG",
            "Emergency stop", "process has died",
        )
        key = next((value for value in keys if value in text), None)
        if not self.active or key is None:
            return
        self.event_counts[key] = self.event_counts.get(key, 0) + 1
        if "RMUC route diagnostic" in text:
            self.route_accepted = True
        if "Navigation stage:" in text:
            self.stage_count += 1
        if "Retrying RMUC stage" in text:
            self.retry_count += 1
        if "recovery diagnostic" in text:
            self.recovery_count += 1
        if "dynamic skip diagnostic" in text:
            self.dynamic_skip_count += 1
        # Save all goal and diagnostic events; sample the high-rate EGO errors.
        keep = ("diagnostic" in key or key in ("RMUC mission accepted", "Navigation stage:",
                "Retrying RMUC stage", "stage made no progress",
                "stage exhausted its retries", "EGO_DIAG", "Emergency stop",
                "process has died"))
        if keep and (self.event_counts[key] <= 100 or
                     self.event_counts[key] % 100 == 0):
            event = {
                "wall_elapsed_s": time.monotonic() - self.start_wall,
                "wall_epoch": stamp_seconds(msg.stamp),
                "kind": key,
                "logger": msg.name,
                "severity": msg.level,
                "message": text,
            }
            self.events.append(event)
            self.last_event_by_type[key] = event

    def record_if_due(self, msg):
        sim_t = stamp_seconds(msg.header.stamp)
        if sim_t - self.last_sample_sim < 0.10:
            return
        self.last_sample_sim = sim_t
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        op = self.odom.pose.pose.position if self.odom else None
        oq = self.odom.pose.pose.orientation if self.odom else None
        sp = self.setpoint.pose.position if self.setpoint else None
        self.samples.append({
            "wall_elapsed_s": time.monotonic() - self.start_wall,
            "sim_t": sim_t,
            "ground_truth_x": p.x, "ground_truth_y": p.y,
            "ground_truth_yaw": yaw_from_quaternion(q),
            "ground_truth_speed": math.hypot(v.x, v.y),
            "odom_x": op.x if op else float("nan"),
            "odom_y": op.y if op else float("nan"),
            "odom_yaw": yaw_from_quaternion(oq) if oq else float("nan"),
            "ground_truth_goal_error_m": math.hypot(p.x-self.goal_x, p.y-self.goal_y),
            "odom_goal_error_m": math.hypot(op.x-self.goal_x, op.y-self.goal_y) if op else float("nan"),
            "setpoint_x": sp.x if sp else float("nan"),
            "setpoint_y": sp.y if sp else float("nan"),
        })

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def wait_ready(self):
        end = time.monotonic() + 30.0
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.gt is not None and self.odom is not None and
                    self.goal_pub.get_subscription_count() > 0 and
                    "ego_planner_node" in self.get_node_names()):
                return True
        return False

    def run(self):
        if not self.wait_ready():
            raise RuntimeError("odometry, navigation goal subscriber, or EGO planner unavailable")
        start = self.gt.pose.pose.position
        start_pose = [start.x, start.y, yaw_from_quaternion(self.gt.pose.pose.orientation)]
        map_path = ROOT / "install/uav_bringup/share/uav_bringup/maps/rmuc_2025.pgm"
        if not map_path.exists():
            map_path = ROOT / "src/uav_bringup/maps/rmuc_2025.pgm"
        grid = GridRoute(str(map_path))
        try:
            grid.route((start.x, start.y), (self.goal_x, self.goal_y))
        except ValueError as error:
            raise RuntimeError(f"Refusing to publish a goal without a clear GridRoute: {error}")
        self.preflight_raw_grid_path = [list(point) for point in grid.last_raw_grid_path]
        self.preflight_waypoints = [list(point) for point in grid.last_waypoints]
        self.preflight_waypoint_clearances = list(grid.last_waypoint_clearances)
        self.preflight_segment_clearances = list(grid.last_segment_clearances)
        self.start_wall = time.monotonic()
        self.start_epoch = time.time()
        self.start_sim = stamp_seconds(self.gt.header.stamp)
        self.active = True
        self.publish_goal()
        result = "timeout"
        reached_since = None
        ego_missing_since = None
        while rclpy.ok() and time.monotonic() - self.start_wall < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.gt is None:
                continue
            sim_t = stamp_seconds(self.gt.header.stamp)
            p = self.gt.pose.pose.position
            speed = math.hypot(self.gt.twist.twist.linear.x,
                               self.gt.twist.twist.linear.y)
            goal_error = math.hypot(p.x-self.goal_x, p.y-self.goal_y)
            if goal_error <= 0.10 and speed <= 0.10:
                if reached_since is None:
                    reached_since = time.monotonic()
                elif time.monotonic() - reached_since >= 1.5:
                    result = "success"
                    break
            else:
                reached_since = None
            names = self.get_node_names()
            if "ego_planner_node" not in names:
                if ego_missing_since is None:
                    ego_missing_since = time.monotonic()
                elif time.monotonic() - ego_missing_since >= 2.0:
                    result = "ego_planner_died"
                    break
            else:
                ego_missing_since = None
        self.active = False
        final = self.gt
        final_p = final.pose.pose.position
        final_op = self.odom.pose.pose.position if self.odom else None
        self.save(start_pose, result, final, final_op)
        return result

    def save(self, start_pose, result, final, final_op):
        OUT.mkdir(parents=True, exist_ok=True)
        csv_path = OUT / (self.name + ".telemetry.csv")
        if self.samples:
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.samples[0]))
                writer.writeheader()
                writer.writerows(self.samples)
        events_path = OUT / (self.name + ".events.jsonl")
        with events_path.open("w", encoding="utf-8") as f:
            for event in self.events:
                f.write(json.dumps(event, separators=(",", ":")) + "\n")
        p = final.pose.pose.position
        summary = {
            "name": self.name,
            "result": result,
            "start_ground_truth": start_pose,
            "goal": [self.goal_x, self.goal_y],
            "preflight_raw_grid_path": self.preflight_raw_grid_path,
            "preflight_waypoints": self.preflight_waypoints,
            "preflight_waypoint_clearance_m": self.preflight_waypoint_clearances,
            "preflight_segment_min_clearance_m": self.preflight_segment_clearances,
            "final_ground_truth": [p.x, p.y],
            "final_odom": [final_op.x, final_op.y] if final_op else None,
            "ground_truth_goal_error_m": math.hypot(p.x-self.goal_x, p.y-self.goal_y),
            "wall_duration_s": time.monotonic()-self.start_wall,
            "sim_duration_s": stamp_seconds(final.header.stamp)-self.start_sim,
            "ego_planner_present_at_end": "ego_planner_node" in self.get_node_names(),
            "route_stages_sent": self.stage_count,
            "retry_count": self.retry_count,
            "recovery_count": self.recovery_count,
            "dynamic_skip_diag_count": self.dynamic_skip_count,
            "event_counts": self.event_counts,
            "events": self.events,
            "samples": len(self.samples),
            "telemetry_csv": str(csv_path),
            "events_jsonl": str(events_path),
        }
        summary_path = OUT / (self.name + ".summary.json")
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True)
    parser.add_argument("--goal-x", type=float, default=17.0)
    parser.add_argument("--goal-y", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args()
    rclpy.init()
    node = LongRangeDiagnosticRunner(args.name, args.goal_x, args.goal_y, args.timeout)
    try:
        result = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result == "success" else 2


if __name__ == "__main__":
    raise SystemExit(main())