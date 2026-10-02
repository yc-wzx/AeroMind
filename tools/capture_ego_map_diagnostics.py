#!/usr/bin/env python3
import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from traj_utils.msg import Bspline

ROOT = Path("/home/rmnav/AeroMind")


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class DiagnosticCapture(Node):
    def __init__(self, name, duration):
        super().__init__("stage15_ego_diagnostic_capture")
        self.name = name
        self.duration = duration
        out = ROOT / "tools" / "results"
        out.mkdir(parents=True, exist_ok=True)
        self.csv_path = out / (name + ".ego_diag.csv")
        self.events_path = out / (name + ".ego_events.jsonl")
        self.rows = []
        self.latest_gt = None
        self.latest_odom = None
        self.latest_goal = None
        self.latest_traj = None
        self.latest_map = None
        self.latest_scan = None
        self.gt_wall = None
        self.odom_wall = None
        self.scan_wall = None
        self.map_wall = None
        self.start_wall = time.monotonic()
        self.last_sample_wall = 0.0
        self.create_subscription(Odometry, "/gazebo/odometry", self.gt_cb, 30)
        self.create_subscription(Odometry, "/ground/odometry", self.odom_cb, 30)
        self.create_subscription(PoseStamped, "/navigation/segment_goal", self.goal_cb, 10)
        self.create_subscription(PointCloud2, "/cloud_registered_2d", self.scan_cb, 10)
        self.create_subscription(PointCloud2, "/grid_map/occupancy_inflate", self.map_cb, 2)
        self.create_subscription(Bspline, "/planning/bspline", self.traj_cb, 10)
        self.create_subscription(Log, "/rosout", self.log_cb, 100)
        self.timer = self.create_timer(1.0, self.write_sample)

    def gt_cb(self, msg):
        self.latest_gt = msg
        self.gt_wall = time.monotonic()

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.odom_wall = time.monotonic()

    def goal_cb(self, msg):
        self.latest_goal = msg
        self.write_event({"kind": "segment_goal", "wall_elapsed": time.monotonic()-self.start_wall,
                          "stamp": stamp_seconds(msg.header.stamp), "frame": msg.header.frame_id,
                          "x": msg.pose.position.x, "y": msg.pose.position.y})

    def scan_cb(self, msg):
        self.latest_scan = msg
        self.scan_wall = time.monotonic()

    def map_cb(self, msg):
        now = time.monotonic()
        if now - self.last_sample_wall < 0.8:
            return
        self.latest_map = (msg, list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)))
        self.map_wall = now

    def traj_cb(self, msg):
        points = [(float(p.x), float(p.y), float(p.z)) for p in msg.pos_pts]
        self.latest_traj = {"traj_id": msg.traj_id, "points": points,
                            "start_time": stamp_seconds(msg.start_time)}
        self.write_event({"kind": "bspline", "wall_elapsed": time.monotonic()-self.start_wall,
                          "traj_id": msg.traj_id, "start_time": self.latest_traj["start_time"],
                          "control_points": points})

    def log_cb(self, msg):
        text = msg.msg
        keys = ("EGO_DIAG", "terminal point of the current trajectory is in obstacle",
                "First 3 control points in obstacles", "Emergency stop",
                "process has died")
        if any(k in text for k in keys):
            self.write_event({"kind": "rosout", "wall_elapsed": time.monotonic()-self.start_wall,
                              "stamp": stamp_seconds(msg.stamp), "logger": msg.name,
                              "severity": msg.level, "message": text})

    def write_event(self, event):
        with self.events_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(event, separators=(",", ":")) + "\n")

    @staticmethod
    def nearest_with_point(points, query):
        if not points:
            return float("nan"), None
        qx, qy, qz = query
        best_d2 = float("inf")
        best = None
        for p in points:
            dx = float(p[0])-qx
            dy = float(p[1])-qy
            dz = float(p[2])-qz
            d2 = dx*dx+dy*dy+dz*dz
            if d2 < best_d2:
                best_d2, best = d2, (float(p[0]), float(p[1]), float(p[2]))
        return math.sqrt(best_d2), best

    @classmethod
    def nearest(cls, points, query):
        return cls.nearest_with_point(points, query)

    def write_sample(self):
        now = time.monotonic()
        if self.latest_gt is None or self.latest_map is None or now-self.last_sample_wall < 0.8:
            return
        self.last_sample_wall = now
        gt = self.latest_gt
        gp = gt.pose.pose.position
        sim_t = stamp_seconds(gt.header.stamp)
        gt_xyz = (gp.x, gp.y, gp.z)
        od = self.latest_odom
        if od is not None:
            op = od.pose.pose.position
            od_xyz = (op.x, op.y, op.z)
        else:
            od_xyz = (float("nan"),)*3
        goal = self.latest_goal
        goal_xy = (goal.pose.position.x, goal.pose.position.y) if goal is not None else None
        m, points = self.latest_map
        nearest_robot, nearest_robot_point = self.nearest_with_point(points, gt_xyz)
        nearest_goal, nearest_goal_point = (
            self.nearest_with_point(points, (goal_xy[0], goal_xy[1], 0.0))
            if goal_xy else (float("nan"), None))
        rows = {
            "wall_elapsed_s": now-self.start_wall,
            "sim_t": sim_t,
            "gt_x": gt_xyz[0], "gt_y": gt_xyz[1], "gt_z": gt_xyz[2],
            "odom_x": od_xyz[0], "odom_y": od_xyz[1], "odom_z": od_xyz[2],
            "gt_odom_error_m": math.dist(gt_xyz, od_xyz) if od is not None else float("nan"),
            "active_goal_x": goal_xy[0] if goal_xy else float("nan"),
            "active_goal_y": goal_xy[1] if goal_xy else float("nan"),
            "goal_distance_m": math.dist(gt_xyz[:2], goal_xy) if goal_xy else float("nan"),
            "map_frame": m.header.frame_id,
            "map_stamp": stamp_seconds(m.header.stamp),
            "map_age_wall_s": now-self.map_wall if self.map_wall else float("nan"),
            "inflated_voxel_count": len(points),
            "nearest_robot_obstacle_m": nearest_robot,
            "nearest_robot_obstacle_xyz": json.dumps(nearest_robot_point, separators=(",", ":")) if nearest_robot_point else "",
            "nearest_goal_obstacle_m": nearest_goal,
            "nearest_goal_obstacle_xyz": json.dumps(nearest_goal_point, separators=(",", ":")) if nearest_goal_point else "",
            "scan_age_wall_s": now-self.scan_wall if self.scan_wall else float("nan"),
            "odom_age_wall_s": now-self.odom_wall if self.odom_wall else float("nan"),
            "scan_age_sim_s": sim_t-stamp_seconds(self.latest_scan.header.stamp) if self.latest_scan else float("nan"),
            "odom_age_sim_s": sim_t-stamp_seconds(od.header.stamp) if od else float("nan"),
            "scan_frame": self.latest_scan.header.frame_id if self.latest_scan else "",
            "scan_point_count": self.latest_scan.width if self.latest_scan else 0,
            "traj_id": self.latest_traj["traj_id"] if self.latest_traj else "",
            "traj_start_time": self.latest_traj["start_time"] if self.latest_traj else float("nan"),
            "traj_control_points": json.dumps(self.latest_traj["points"], separators=(",", ":")) if self.latest_traj else "",
        }
        if self.latest_traj and self.latest_traj["points"]:
            cps = self.latest_traj["points"]
            rows["traj_start_nearest_obstacle_m"] = self.nearest(points, cps[0])[0]
            rows["traj_endpoint_nearest_obstacle_m"] = self.nearest(points, cps[-1])[0]
            rows["traj_first4_nearest_obstacles_m"] = json.dumps(
                [self.nearest(points, q)[0] for q in cps[:4]], separators=(",", ":"))
        else:
            rows["traj_start_nearest_obstacle_m"] = float("nan")
            rows["traj_endpoint_nearest_obstacle_m"] = float("nan")
            rows["traj_first4_nearest_obstacles_m"] = ""
        self.rows.append(rows)
        with self.csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows))
            writer.writeheader()
            writer.writerows(self.rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--duration", type=float, default=360.0)
    args = ap.parse_args()
    rclpy.init()
    node = DiagnosticCapture(args.name, args.duration)
    end = time.monotonic()+args.duration
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"csv": str(node.csv_path), "events": str(node.events_path),
                      "samples": len(node.rows)}, indent=2))


if __name__ == "__main__":
    main()
