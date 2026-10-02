"""Read-only Stage 1.5 component and benchmark audit for the RMUC ground map."""
import argparse
import hashlib
import heapq
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import label

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/uav_planning/scripts"))
from grid_route import GridRoute  # noqa: E402

MAP = ROOT / "src/uav_bringup/maps/rmuc_2025.pgm"


def component_audit(grid):
    labels, count = label(grid.safe)
    sizes = np.bincount(labels.ravel())
    components = []
    for component_id in range(1, count + 1):
        rows, cols = np.where(labels == component_id)
        if not len(rows):
            continue
        points = np.column_stack((rows, cols))
        best = points[np.argmax(grid.clearance[rows, cols])]
        lo = grid.point((int(rows.max()), int(cols.min())))
        hi = grid.point((int(rows.min()), int(cols.max())))
        representative = grid.point(tuple(map(int, best)))
        components.append({
            "id": component_id,
            "cell_count": int(sizes[component_id]),
            "world_bounds_xy_m": [round(lo[0], 3), round(lo[1], 3),
                                  round(hi[0], 3), round(hi[1], 3)],
            "representative_xy_m": [round(v, 3) for v in representative],
        })
    components.sort(key=lambda item: item["cell_count"], reverse=True)
    return labels, components


def route_metrics(grid, labels, start, goal):
    waypoints = grid.route(start, goal)
    raw = grid.last_raw_grid_path
    segment_points = [tuple(start)] + list(waypoints)
    headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(segment_points, segment_points[1:])]
    turn_count = sum(abs((b - a + math.pi) % (2 * math.pi) - math.pi) > math.radians(25)
                     for a, b in zip(headings, headings[1:]))
    source, target = grid.cell(*start), grid.cell(*goal)
    return {
        "start": [float(v) for v in start],
        "goal": [float(v) for v in goal],
        "connected_component": int(labels[source]),
        "euclidean_distance_m": round(math.dist(start, goal), 3),
        "astar_path_length_m": round(sum(math.dist(a, b) for a, b in zip(raw, raw[1:])), 3),
        "waypoint_path_length_m": round(sum(math.dist(a, b) for a, b in
                                            zip(segment_points, segment_points[1:])), 3),
        "minimum_static_clearance_m": round(min(grid.last_segment_clearances), 3),
        "start_clearance_m": round(float(grid.clearance[source]), 3),
        "goal_clearance_m": round(float(grid.clearance[target]), 3),
        "turn_count_over_25deg": turn_count,
        "waypoints": [[round(v, 3) for v in p] for p in waypoints],
    }


def geodesic_distances(grid, labels, start):
    source = grid.cell(*start)
    component_id = labels[source]
    distances = np.full(grid.safe.shape, np.inf)
    distances[source] = 0.0
    queue = [(0.0, source)]
    offsets = [(dr, dc, math.hypot(dr, dc) * grid.resolution)
               for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
    while queue:
        distance, (row, col) = heapq.heappop(queue)
        if distance > distances[row, col] + 1e-10:
            continue
        for dr, dc, step in offsets:
            neighbor = row + dr, col + dc
            nr, nc = neighbor
            if not (0 <= nr < grid.height and 0 <= nc < grid.width and
                    labels[neighbor] == component_id):
                continue
            if dr and dc and not (grid.safe[row + dr, col] and
                                  grid.safe[row, col + dc]):
                continue
            proposed = distance + step
            if proposed < distances[neighbor] - 1e-10:
                distances[neighbor] = proposed
                heapq.heappush(queue, (proposed, neighbor))
    return distances


def find_long_candidates(grid, labels, start, limit=10):
    distances = geodesic_distances(grid, labels, start)
    component_id = labels[grid.cell(*start)]
    rows, cols = np.where((labels == component_id) &
                          (grid.clearance >= 1.0) & np.isfinite(distances))
    cells = sorted(zip(rows, cols), key=lambda cell: distances[cell], reverse=True)
    goals = []
    for cell in cells:
        point = grid.point(cell)
        if any(math.dist(point, prior) < 0.35 for prior in goals):
            continue
        goals.append(point)
        if len(goals) >= 120:
            break
    candidates = []
    for goal in goals:
        try:
            result = route_metrics(grid, labels, start, goal)
        except ValueError:
            continue
        if result["minimum_static_clearance_m"] >= grid.clearance_required - 1e-3:
            candidates.append(result)
    candidates.sort(key=lambda item: item["astar_path_length_m"], reverse=True)
    return candidates[:limit], round(float(np.max(distances[np.isfinite(distances)])), 3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    grid = GridRoute(MAP, clearance=0.55)
    labels, components = component_audit(grid)
    old = {}
    for name, start, goal in [
        ("old_baseline", (3, 3), (8.8, 1.5)),
        ("old_long_range", (8.78, 1.5), (17, 8)),
    ]:
        a, b = grid.cell(*start), grid.cell(*goal)
        old[name] = {"start": list(start), "goal": list(goal),
                     "start_component": int(labels[a]), "goal_component": int(labels[b])}
        try:
            grid.route(start, goal)
            old[name]["status"] = "REACHABLE"
        except ValueError as error:
            old[name]["status"] = "INVALIDATED FOR CURRENT PLANAR SIMULATION"
            old[name]["reason"] = str(error)
    routes = {}
    for name, start, goal in [
        ("start_area", (3, 3), (2, 4.5)),
        ("baseline", (3, 3), (5, 7)),
        ("turn", (3, 3), (5.5, 5)),
        ("long_range", (3, 3), (9, 9.75)),
        ("round_trip_out", (3, 3), (5, 7)),
        ("round_trip_back", (5, 7), (3, 3)),
    ]:
        routes[name] = route_metrics(grid, labels, start, goal)
    long_candidates, max_geodesic = find_long_candidates(grid, labels, (3, 3))
    report = {
        "map": str(MAP), "map_sha256": hashlib.sha256(MAP.read_bytes()).hexdigest(),
        "resolution_m": grid.resolution, "clearance_required_m": grid.clearance_required,
        "component_connectivity": "4-neighbor, equal to GridRoute diagonal graph without corner cutting",
        "major_components": [entry for entry in components if entry["cell_count"] >= 1000],
        "old_benchmarks": old, "proposed_routes": routes,
        "maximum_safe_grid_geodesic_m": max_geodesic,
        "farthest_route_candidates_with_endpoint_clearance_1m": long_candidates,
        "scope": "provisional offline analysis of the current planar map; arena verification failed, no navigation goal published; physical 3D connectivity not established",
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(args.output)
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
