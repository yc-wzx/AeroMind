"""Read-only geometry, reachability, and footprint audit for the provisional arena."""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import label

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src/uav_planning/scripts"))
from generate_provincial_provisional_arena import SOURCE, FIELD, WORLD, MAP, build_geometry
from grid_route import GridRoute

RESULT = ROOT / "tools/results/provincial_stage15_provisional_offline_audit.json"
BENCHMARKS = ROOT / "tools/config/provincial_stage15_provisional_benchmarks.yaml"
ROUTES = {
    "start": ([4.7, 0.5], [4.7, 1.15]),
    "turn": ([4.7, 1.15], [5.8, 1.8]),
    "shoot": ([5.8, 1.8], [8.7, 4.25]),
    "full_auto": ([4.7, 0.5], [8.7, 4.25]),
    "return": ([8.7, 4.25], [4.7, 0.5]),
}
CLEARANCES = [0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55]


def same_component(grid, start, goal, clearance):
    a, b = grid.cell(*start), grid.cell(*goal)
    if a is None or b is None:
        return False
    safe = grid.clearance >= clearance
    if not (safe[a] and safe[b]):
        return False
    labels, _ = label(safe)
    return labels[a] == labels[b]


def route_metrics(grid, start, goal):
    try:
        waypoints = grid.route(start, goal)
        raw = grid.last_raw_grid_path
        return {
            "reachable": True,
            "raw_path_length_m": round(
                sum(math.dist(a, b) for a, b in zip(raw, raw[1:])), 3),
            "raw_path_min_occupied_cell_clearance_m": round(
                min(float(grid.clearance[grid.cell(*p)]) for p in raw), 3),
            "smoothed_path_min_occupied_cell_clearance_m": round(
                min(grid.last_segment_clearances), 3),
            "waypoints": [[round(x, 3), round(y, 3)] for x, y in waypoints],
        }
    except ValueError as error:
        return {"reachable": False, "reason": str(error)}


def polygon_edges(source):
    w = source["auto_passage_clear_width_m"]
    x0, y0 = source["reference_route"][0]
    _, bend_y = source["reference_route"][1]
    far_x, _ = source["reference_route"][2]
    top = source["shooting_zone"]["y"] + source["shooting_zone"]["height"] / 2
    L, R0 = x0 - w / 2, x0 + w / 2
    F, R = far_x - w / 2, far_x + w / 2
    B, T = bend_y - w / 2, bend_y + w / 2
    return [
        ((L, 0), (L, T)), ((L, T), (F, T)),
        ((F, T), (F, top)), ((F, top), (R, top)),
        ((R, top), (R, B)), ((R, B), (R0, B)),
        ((R0, B), (R0, 0)), ((R0, 0), (L, 0)),
    ]


def rectangle_clearance_on_centerline(source, rects, yaw):
    length, width = 0.52, 0.42
    bx, by = np.meshgrid(np.linspace(-length / 2, length / 2, 27),
                            np.linspace(-width / 2, width / 2, 22))
    bx, by = bx.ravel(), by.ravel()
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = c * bx - s * by, s * bx + c * by
    min_margin = float("inf")
    edges = polygon_edges(source)
    route = source["reference_route"]
    for a, b in zip(route, route[1:]):
        distance = math.dist(a, b)
        for t in np.linspace(0, 1, max(2, math.ceil(distance / 0.02) + 1)):
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            X, Y = x + dx, y + dy
            inside = np.zeros(X.shape, dtype=bool)
            for x0, y0, x1, y1 in rects:
                inside |= ((x0 <= X) & (X <= x1) & (y0 <= Y) & (Y <= y1))
            if not inside.all():
                return {"collision_free": False, "minimum_wall_margin_m": 0.0}
            for (x0, y0), (x1, y1) in edges:
                vx, vy = x1 - x0, y1 - y0
                u = np.clip(((X - x0) * vx + (Y - y0) * vy) /
                            (vx * vx + vy * vy), 0, 1)
                margin = np.hypot(X - (x0 + u * vx), Y - (y0 + u * vy))
                min_margin = min(min_margin, float(margin.min()))
    return {"collision_free": True, "minimum_wall_margin_m": round(min_margin, 3)}


def check_assets(source, field, rects, walls):
    assert field["course_surfaces"] == [
        {
            "x": round((x0 + x1) / 2, 6),
            "y": round((y0 + y1) / 2, 6),
            "width": round(x1 - x0, 6),
            "height": round(y1 - y0, 6),
        }
        for x0, y0, x1, y1 in rects
    ]
    assert field["collision_segments"] == walls
    pixels = np.asarray(Image.open(MAP), dtype=np.uint8)
    resolution = source["map_resolution_m"]
    assert pixels.shape == (
        round(source["arena"]["height"] / resolution),
        round(source["arena"]["width"] / resolution))
    for row in range(pixels.shape[0]):
        y = (pixels.shape[0] - row - 0.5) * resolution
        xs = (np.arange(pixels.shape[1]) + 0.5) * resolution
        expected = np.zeros(xs.shape, dtype=bool)
        for x0, y0, x1, y1 in rects:
            expected |= (x0 <= xs) & (xs < x1) & (y0 <= y) & (y < y1)
        assert np.array_equal(pixels[row] == 255, expected)
    sdf = ET.parse(WORLD).getroot()
    models = {model.get("name"): model for model in sdf.findall("./world/model")}
    assert len([name for name in models if name.startswith("boundary_")]) == len(walls)
    for index, (x0, y0, x1, y1) in enumerate(walls):
        model = models[f"boundary_{index}"]
        x, y, _, _, _, yaw = map(float, model.findtext("pose").split())
        size = list(map(float, model.findtext("./link/collision/geometry/box/size").split()))
        assert abs(x - (x0 + x1) / 2) < 1e-5
        assert abs(y - (y0 + y1) / 2) < 1e-5
        assert abs(size[0] - math.hypot(x1 - x0, y1 - y0)) < 1e-5
        assert abs(size[1] - source["training_wall_thickness_m"]) < 1e-5
        assert abs(yaw - math.atan2(y1 - y0, x1 - x0)) < 1e-5
    for index, (x0, y0, x1, y1) in enumerate(rects):
        model = models[f"course_{index}"]
        x, y, _, _, _, _ = map(float, model.findtext("pose").split())
        size = list(map(float, model.findtext("./link/visual/geometry/box/size").split()))
        assert abs(x - (x0 + x1) / 2) < 1e-5
        assert abs(y - (y0 + y1) / 2) < 1e-5
        assert abs(size[0] - (x1 - x0)) < 1e-5
        assert abs(size[1] - (y1 - y0)) < 1e-5
    return {"visual_from_source": True, "collision_from_source": True,
            "pgm_from_source": True, "wall_count": len(walls)}


def main():
    source = json.loads(SOURCE.read_text(encoding="utf-8-sig"))
    field = json.loads(FIELD.read_text(encoding="utf-8"))
    rects, walls = build_geometry(source)
    consistency = check_assets(source, field, rects, walls)
    width = source["auto_passage_clear_width_m"]
    length, body_width = 0.52, 0.42
    circ = math.hypot(length / 2, body_width / 2)
    grid_margin = source["map_resolution_m"] / math.sqrt(2)
    assumed_safety = 0.03
    proposed = round(circ + grid_margin + assumed_safety, 2)
    grid = GridRoute(MAP, clearance=proposed)
    sweep = {}
    for clearance in CLEARANCES:
        test_grid = GridRoute(MAP, clearance=clearance)
        sweep[f"{clearance:.2f}"] = {
            name: route_metrics(test_grid, start, goal)["reachable"]
            for name, (start, goal) in ROUTES.items()
        }
    thresholds = {}
    candidate_routes = {}
    possible = np.unique(grid.clearance)
    possible = possible[possible <= 0.8]
    for name, (start, goal) in ROUTES.items():
        threshold = next(
            (float(value) for value in possible[::-1]
             if same_component(grid, start, goal, float(value))), 0.0)
        thresholds[name] = round(threshold, 3)
        candidate_routes[name] = route_metrics(grid, start, goal)
    rect_models = {
        "yaw_90deg_fixed": rectangle_clearance_on_centerline(
            source, rects, math.pi / 2),
        "yaw_45deg_fixed": rectangle_clearance_on_centerline(
            source, rects, math.pi / 4),
    }
    results = {
        "status": "PROVISIONAL_STAGE15_OFFLINE_AUDIT",
        "source": str(SOURCE.relative_to(ROOT)),
        "final_arena_verified": False,
        "auto_passage_width_m": width,
        "width_evidence": source["source"]["width_derivation"],
        "asset_consistency": consistency,
        "robot_footprint_m": [length, body_width],
        "rectangular_aligned_side_margin_m": round((width - body_width) / 2, 3),
        "rectangular_sideways_side_margin_m": round((width - length) / 2, 3),
        "circumscribed_radius_m": round(circ, 3),
        "grid_diagonal_half_cell_m": round(grid_margin, 3),
        "assumed_ideal_tracking_safety_budget_m": assumed_safety,
        "available_safety_after_circumscribed_and_grid_m": round(
            width / 2 - circ - grid_margin, 3),
        "derived_circular_clearance_candidate_m": proposed,
        "current_0_55_minimum_corridor_width_m": 1.10,
        "clearance_sweep": sweep,
        "maximum_reachable_clearance_m": thresholds,
        "candidate_routes": candidate_routes,
        "rectangular_footprint_centerline_checks": rect_models,
    }
    RESULT.parent.mkdir(parents=True, exist_ok=True)
    RESULT.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n",
                      encoding="utf-8")
    manifest = {
        "name": "PROVISIONAL_STAGE15_BENCHMARKS",
        "source": "2025 provisional provincial rules",
        "final_arena_verified": False,
        "arena": str(SOURCE.relative_to(ROOT)),
        "ground_pgm": str(MAP.relative_to(ROOT)),
        "clearance_candidate_m": proposed,
        "routes": {
            name: {
                "start": start, "goal": goal,
                "classification": "TRAINING-ONLY BENCHMARK CANDIDATES",
                "offline_reachable": candidate_routes[name]["reachable"],
                "raw_path_length_m": candidate_routes[name].get("raw_path_length_m"),
                "maximum_reachable_clearance_m": thresholds[name],
            }
            for name, (start, goal) in ROUTES.items()
        },
    }
    BENCHMARKS.write_text(yaml.safe_dump(
        manifest, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
