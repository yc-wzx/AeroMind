#!/usr/bin/env python3
"""Offline A/B/C path, footprint, retry and yaw diagnosis; never publishes ROS data."""
import csv
import json
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/uav_planning/scripts"))
from grid_route import GridRoute

# Keep this post-processing tool importable without sourcing ROS 2. These
# geometry primitives mirror the read-only benchmark logger's SDF-box math.
GOALS = {"B": (5.8, 1.8), "C": (8.7, 4.25)}


def polygon(cx, cy, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx+c*x-s*y, cy+s*x+c*y)
            for x, y in ((-length/2, -width/2), (length/2, -width/2),
                         (length/2, width/2), (-length/2, width/2))]


def wall_polygon(segment, thickness=0.055):
    x0, y0, x1, y1 = segment
    return polygon((x0+x1)/2, (y0+y1)/2,
                   math.atan2(y1-y0, x1-x0), math.hypot(x1-x0, y1-y0), thickness)


def point_segment_distance(p, a, b):
    vx, vy = b[0]-a[0], b[1]-a[1]
    t = max(0.0, min(1.0, ((p[0]-a[0])*vx+(p[1]-a[1])*vy)/(vx*vx+vy*vy)))
    return math.hypot(p[0]-a[0]-t*vx, p[1]-a[1]-t*vy)


def polygon_distance(a, b):
    for poly in (a, b):
        for p, q in zip(poly, poly[1:]+poly[:1]):
            axis = (-(q[1]-p[1]), q[0]-p[0])
            pa = [x*axis[0]+y*axis[1] for x, y in a]
            pb = [x*axis[0]+y*axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                break
        else:
            continue
        break
    else:
        return 0.0
    return min(point_segment_distance(p, u, v)
               for first, second in ((a, b), (b, a))
               for p in first for u, v in zip(second, second[1:]+second[:1]))

DATA = ROOT / "tools/results/provincial_low_speed"
OUT = ROOT / "tools/results/provincial_local_safety"
FIELD = json.loads((ROOT / "src/uav_planning/config/provincial_2025_provisional.json").read_text())
WALLS = [wall_polygon(segment) for segment in FIELD["collision_segments"]]
GRID = GridRoute(ROOT / "src/uav_bringup/maps/provincial_2025_provisional.pgm", clearance=0.40)
TRACE_NAMES = {"B": "diagnostic_b_turn", "C": "diagnostic_c_shoot"}


def rows(name):
    with (DATA / f"{name}.csv").open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def events(name):
    return [json.loads(line) for line in (DATA / f"{name}.events.jsonl").read_text().splitlines()]


def stats(values):
    v = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if not len(v):
        return None
    return {"count": int(len(v)), "mean": round(float(v.mean()), 5),
            "p95": round(float(np.percentile(v, 95)), 5),
            "max": round(float(v.max()), 5), "min": round(float(v.min()), 5)}


def nearest_on_path(point, path):
    best = (math.inf, None, None)
    x, y = point
    for index, (a, b) in enumerate(zip(path, path[1:])):
        vx, vy = b[0]-a[0], b[1]-a[1]
        denominator = vx*vx+vy*vy
        if denominator <= 1e-12:
            continue
        t = max(0.0, min(1.0, ((x-a[0])*vx+(y-a[1])*vy)/denominator))
        foot = (a[0]+t*vx, a[1]+t*vy)
        distance = math.hypot(x-foot[0], y-foot[1])
        if distance < best[0]:
            best = (distance, foot, index)
    return best


def sample_path(path, spacing=0.01):
    samples = []
    for a, b in zip(path, path[1:]):
        n = max(1, math.ceil(math.dist(a, b)/spacing))
        samples.extend((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                       for t in (index/n for index in range(n)))
    return samples+[tuple(path[-1])]


def point_to_wall(point):
    x, y = point
    best = (math.inf, None)
    for index, wall in enumerate(WALLS):
        xs, ys = zip(*wall)
        if min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys):
            return 0.0, index
        distance = min(point_segment_distance(point, a, b)
                       for a, b in zip(wall, wall[1:]+wall[:1]))
        if distance < best[0]:
            best = distance, index
    return best


def linf_to_wall(point):
    x, y = point
    return min(max(max(min(v[0] for v in wall)-x, 0.0, x-max(v[0] for v in wall)),
                   max(min(v[1] for v in wall)-y, 0.0, y-max(v[1] for v in wall)))
               for wall in WALLS)


def grid_center_clearance(point):
    cell = GRID.cell(*point)
    return float(GRID.clearance[cell]) if cell is not None else 0.0


def finite_setpoints(trace):
    return [row for row in trace if math.isfinite(row.get("setpoint_x", math.nan))
            and math.isfinite(row.get("setpoint_y", math.nan))]


def local_lateral_errors(trace):
    valid = finite_setpoints(trace)
    result = []
    for before, now, after in zip(valid, valid[1:], valid[2:]):
        if after["sim_t"]-before["sim_t"] > 0.25:
            continue
        vx = after["setpoint_x"]-before["setpoint_x"]
        vy = after["setpoint_y"]-before["setpoint_y"]
        length = math.hypot(vx, vy)
        if length < 0.005 or length > 0.10:
            continue
        dx = now["gt_x"]-now["setpoint_x"]
        dy = now["gt_y"]-now["setpoint_y"]
        result.append({"lateral_m":abs(dx*vy-dy*vx)/length,
                       "sim_time_s":now["sim_t"],
                       "gt_xy":[now["gt_x"],now["gt_y"]],
                       "setpoint_xy":[now["setpoint_x"],now["setpoint_y"]],
                       "setpoint_position_error_m":now["setpoint_error"],
                       "reference_step_m":length})
    return result


def retry_details(name, trace):
    active_waypoint = None
    records = []
    for event in events(name):
        message = event["message"]
        if "Navigation stage:" in message:
            match = re.search(r"Navigation stage: \(([-.\d]+), ([-.\d]+)\)", message)
            if match:
                active_waypoint = [float(match.group(1)), float(match.group(2))]
        if "Retrying RMUC stage" not in message:
            continue
        row = min(trace, key=lambda r: abs(r["wall_s"]-event["wall_s"]))
        center, wall_index = point_to_wall((row["gt_x"],row["gt_y"]))
        records.append({
            "wall_time_s": event["wall_s"], "sim_time_s": round(row["sim_t"],3),
            "gt_pose": [round(row["gt_x"],3),round(row["gt_y"],3),round(row["gt_yaw"],3)],
            "global_waypoint": active_waypoint,
            "published_bspline_endpoint": event.get("published_bspline_endpoint"),
            "ego_internal_local_target": "UNAVAILABLE: not published by EGO",
            "nearest_wall_index": wall_index,
            "center_to_wall_m": round(center,3),
            "body_to_wall_m": round(row["wall_clearance"],3),
            "gt_to_commanded_setpoint_m": round(row["setpoint_error"],3)
            if math.isfinite(row["setpoint_error"]) else None,
            "ego_occupancy_event": False,
            "retry_trigger": "interface timer >4 s, speed <0.08 m/s, goal not completed",
        })
    return records


def analyze_route(route, name):
    trace = rows(name)
    goal = GOALS[route]
    start = (trace[0]["gt_x"], trace[0]["gt_y"])
    waypoints = GRID.route(start, goal)
    raw = GRID.last_raw_grid_path[:]
    global_dense = sample_path(raw)
    valid = finite_setpoints(trace)
    if not valid:
        raise RuntimeError(f"No XY setpoints captured for {name}")
    global_to_ego = [nearest_on_path((r["setpoint_x"],r["setpoint_y"]),raw)[0]
                     for r in valid]
    ego_center = [point_to_wall((r["setpoint_x"],r["setpoint_y"]))[0]
                  for r in valid]
    gt_center = [point_to_wall((r["gt_x"],r["gt_y"]))[0] for r in trace]
    global_center = [point_to_wall(p)[0] for p in global_dense]
    model_a_margins = [linf_to_wall((r["setpoint_x"],r["setpoint_y"]))-0.25
                       for r in valid]
    model_b_margins = [distance-math.hypot(0.26,0.21) for distance in ego_center]
    model_c_margins = [min(polygon_distance(
        polygon(r["setpoint_x"],r["setpoint_y"],r["gt_yaw"],0.52,0.42),wall)
        for wall in WALLS) for r in valid]
    lateral_errors = local_lateral_errors(trace)
    spatial_tree = cKDTree([(r["setpoint_x"],r["setpoint_y"]) for r in valid])
    spatial_distances,_ = spatial_tree.query([(r["gt_x"],r["gt_y"]) for r in trace])
    closest = min(trace,key=lambda r:r["wall_clearance"])
    ego_point = (closest["setpoint_x"],closest["setpoint_y"])
    global_dev,global_point,_ = nearest_on_path(ego_point,raw)
    center_global,_ = point_to_wall(global_point)
    center_ego,ego_wall_id = point_to_wall(ego_point)
    center_gt,gt_wall_id = point_to_wall((closest["gt_x"],closest["gt_y"]))
    critical = {
        "gt": [round(closest["gt_x"],4),round(closest["gt_y"],4),round(closest["gt_yaw"],4)],
        "ego_setpoint": [round(ego_point[0],4),round(ego_point[1],4)],
        "nearest_global": [round(global_point[0],4),round(global_point[1],4)],
        "global_to_ego_offset_m": round(global_dev,4),
        "ego_to_gt_error_m": round(closest["setpoint_error"],4),
        "global_center_wall_m": round(center_global,4),
        "ego_center_wall_m": round(center_ego,4),
        "gt_center_wall_m": round(center_gt,4),
        "gt_body_wall_m": round(closest["wall_clearance"],4),
        "nearest_wall_index": gt_wall_id,
        "yaw_relative_deg": round(math.degrees(closest["yaw_relative_rad"]),2),
        "effective_width_m": round(closest["effective_width"],4),
        "published_bspline_endpoint": [round(closest["bspline_endpoint_x"],3),
                                       round(closest["bspline_endpoint_y"],3)],
    }
    result = {
        "trace": name,
        "global_waypoints": [[round(x,3),round(y,3)] for x,y in waypoints],
        "global_raw_grid_min_clearance_m": round(min(grid_center_clearance(p) for p in global_dense),4),
        "global_center_physical_wall_min_m": round(min(global_center),4),
        "ego_center_physical_wall_min_m": round(min(ego_center),4),
        "gt_center_physical_wall_min_m": round(min(gt_center),4),
        "gt_body_physical_wall_min_m": round(min(r["wall_clearance"] for r in trace),4),
        "global_to_ego_offset_m": stats(global_to_ego),
        "ego_to_gt_position_error_m": stats(r["setpoint_error"] for r in valid),
        "ego_to_gt_lateral_error_m": stats(r["lateral_m"] for r in lateral_errors),
        "gt_to_ego_spatial_path_offset_m": stats(spatial_distances),
        "largest_lateral_error_samples": sorted(lateral_errors,key=lambda r:r["lateral_m"],reverse=True)[:3],
        "gt_to_corridor_center_m": stats(r["centerline_cte"] for r in trace),
        "model_A_fixed_square_0_25_m": {
            "minimum_signed_margin_m":round(min(model_a_margins),4),
            "unsafe_samples":sum(m<=0 for m in model_a_margins),"samples":len(valid)},
        "model_B_fixed_circle_0_334_m": {
            "minimum_signed_margin_m":round(min(model_b_margins),4),
            "unsafe_samples":sum(m<=0 for m in model_b_margins),"samples":len(valid)},
        "model_C_oriented_rectangle": {
            "minimum_body_wall_gap_m":round(min(model_c_margins),4),
            "overlap_samples":sum(m<=1e-8 for m in model_c_margins),"samples":len(valid)},
        "critical_min_body_wall": critical,
        "retry_events":retry_details(name,trace),
    }
    plot_route(route,raw,trace,critical)
    return result


def plot_route(route, raw, trace, critical):
    fig, axes = plt.subplots(1,2,figsize=(15,7.2),dpi=150)
    gx,gy,yaw = critical["gt"]
    body = polygon(gx,gy,yaw,.52,.42)
    valid = finite_setpoints(trace)
    for ax in axes:
        for wall in WALLS:
            ax.add_patch(Polygon(wall,closed=True,facecolor="black",edgecolor="black",alpha=.55))
        ax.plot([p[0] for p in raw],[p[1] for p in raw],"--",lw=1.7,color="tab:blue",label="GridRoute raw A*")
        ax.plot([r["setpoint_x"] for r in valid],[r["setpoint_y"] for r in valid],
                color="tab:orange",lw=1.7,label="EGO commanded XY")
        ax.plot([r["gt_x"] for r in trace],[r["gt_y"] for r in trace],
                color="tab:green",lw=1.0,alpha=.9,label="Gazebo GT")
        ax.add_patch(Polygon(body,closed=True,facecolor="tab:red",edgecolor="darkred",alpha=.45,
                             label="0.52 x 0.42 m footprint at min gap"))
        ax.scatter([critical["nearest_global"][0],critical["ego_setpoint"][0],gx],
                   [critical["nearest_global"][1],critical["ego_setpoint"][1],gy],
                   c=["tab:blue","tab:orange","tab:green"],s=35,zorder=8)
        ax.set_aspect("equal",adjustable="box")
        ax.set_xlabel("x (m)");ax.set_ylabel("y (m)")
        ax.grid(alpha=.2)
    ax=axes[0]
    xs=[p[0] for p in raw]+[r["gt_x"] for r in trace]
    ys=[p[1] for p in raw]+[r["gt_y"] for r in trace]
    ax.set_xlim(min(xs)-.5,max(xs)+.5)
    ax.set_ylim(min(ys)-.5,max(ys)+.5)
    ax.set_title(f"Test {route}: full route")
    axes[1].set_xlim(gx-.46,gx+.46)
    axes[1].set_ylim(gy-.46,gy+.46)
    axes[1].set_title(f"Minimum body-wall gap: {critical['gt_body_wall_m']:.3f} m")
    axes[1].legend(loc="best",fontsize=7)
    fig.tight_layout()
    fig.savefig(OUT / f"test_{route.lower()}_four_layers.png")
    plt.close(fig)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    yaw_stats = {}
    for route,name in (("A","test_a_start"),("B","test_b_turn"),("C","test_c_shoot")):
        trace=rows(name)
        half=np.asarray([r["effective_width"]/2 for r in trace])
        yaw_stats[route]={"required_half_width_m":stats(half),
                          "fraction_over_0_25":round(float(np.mean(half>0.25)),4),
                          "fraction_over_0_22":round(float(np.mean(half>0.22)),4),
                          "sample_count":len(trace)}
    result={"status":"OFFLINE_DIAGNOSIS_ONLY", "no_navigation_behavior_changed":True,
            "yaw_footprint_original_runs":yaw_stats,
            "B":analyze_route("B",TRACE_NAMES["B"]),
            "C":analyze_route("C",TRACE_NAMES["C"]),
            "C_original_retries":retry_details("test_c_shoot",rows("test_c_shoot")),
            "tracking_original_runs":{
                route:stats(r["setpoint_error"] for r in rows(name))
                for route,name in (("A","test_a_start"),("B","test_b_turn"),("C","test_c_shoot"))},
            "limitations":["Original A/B/C CSV saved only scalar setpoint error; rerun B/C records XY.",
                           "EGO internal local_target_pt_ is not published; B-spline endpoint is a proxy, not identical.",
                           "Model A uses idealized fixed square inflation against SDF boxes, not EGO's recorded live occupancy grid."]}
    (OUT/"analysis.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
