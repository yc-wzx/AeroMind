"""Occupancy-map prior and clearance-aware waypoints for the RMUC arena."""
import ast
import heapq
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, distance_transform_edt, label


class GridRoute:
    def __init__(self, pgm, resolution=None, clearance=0.55):
        self.pgm = Path(pgm)
        self.pixels = np.asarray(Image.open(self.pgm), dtype=np.uint8)
        metadata = self.pgm.with_suffix(".yaml")
        map_resolution = None
        origin = (0.0, 0.0, 0.0)
        if metadata.exists():
            for line in metadata.read_text(encoding="utf-8").splitlines():
                key, separator, value = line.partition(":")
                if not separator:
                    continue
                if key.strip() == "resolution":
                    map_resolution = float(value.strip())
                elif key.strip() == "origin":
                    parsed = ast.literal_eval(value.strip())
                    if len(parsed) != 3:
                        raise ValueError(f"Invalid map origin in {metadata}")
                    origin = tuple(float(part) for part in parsed)
        self.resolution = float(
            resolution if resolution is not None else (map_resolution or 0.05))
        if abs(origin[2]) > 1e-9:
            raise ValueError("GridRoute only supports an axis-aligned RMUC map origin")
        self.origin_x, self.origin_y = origin[:2]
        self.clearance_required = float(clearance)
        self.clearance = distance_transform_edt(self.pixels >= 250) * self.resolution
        self.safe = self.clearance >= self.clearance_required
        # Four-connected components match the eight-neighbor A* graph because
        # diagonal moves are allowed only when both side cells are safe.
        self.component_labels, self.component_count = label(self.safe)
        self.height, self.width = self.pixels.shape
        self.last_raw_grid_path = []
        self.last_waypoints = []
        self.last_waypoint_clearances = []
        self.last_segment_clearances = []

    def cell(self, x, y):
        col = math.floor((x - self.origin_x) / self.resolution)
        row = self.height - 1 - math.floor((y - self.origin_y) / self.resolution)
        if not (0 <= row < self.height and 0 <= col < self.width):
            return None
        return row, col

    def point(self, cell):
        row, col = cell
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (self.height - row - 0.5) * self.resolution)

    def boundary_points(self):
        occupied = self.pixels < 250
        edge = occupied & binary_dilation(~occupied)
        rows, cols = np.nonzero(edge)
        return [(self.origin_x + (col + 0.5) * self.resolution,
                 self.origin_y + (self.height - row - 0.5) * self.resolution,
                 0.0)
                for row, col in zip(rows[::2], cols[::2])]

    def near_static_obstacle(self, x, y, tolerance=0.08):
        cell = self.cell(x, y)
        return cell is not None and self.clearance[cell] <= tolerance

    def ray_has_unmapped_return(self, origin, angle, distance, margin=0.60):
        # Check whether a lidar hit occurs before any wall in the static map.
        step = self.resolution * 0.5
        limit = distance + margin
        count = int(math.ceil(limit / step))
        cos_angle, sin_angle = math.cos(angle), math.sin(angle)
        for index in range(1, count + 1):
            ray_distance = min(index * step, limit)
            cell = self.cell(origin[0] + ray_distance * cos_angle,
                             origin[1] + ray_distance * sin_angle)
            if cell is None or self.pixels[cell] < 250:
                return False
        return True

    def line_min_clearance(self, a, b):
        distance = math.dist(a, b)
        steps = max(1, int(math.ceil(distance / (self.resolution * 0.5))))
        values = []
        for index in range(steps + 1):
            t = index / steps
            cell = self.cell(a[0] + t * (b[0] - a[0]),
                             a[1] + t * (b[1] - a[1]))
            if cell is None:
                return 0.0
            values.append(float(self.clearance[cell]))
        return min(values)

    def line_safe(self, a, b, clearance=None):
        distance = math.dist(a, b)
        required = self.clearance_required if clearance is None else clearance
        steps = max(2, int(distance / self.resolution) + 1)
        for index in range(1, steps):
            t = index / steps
            cell = self.cell(a[0] + t * (b[0] - a[0]),
                             a[1] + t * (b[1] - a[1]))
            if cell is None or self.clearance[cell] < required:
                return False
        return True

    def _save_route_diagnostics(self, start, waypoints, segments):
        self.last_waypoints = [tuple(map(float, point)) for point in waypoints]
        self.last_waypoint_clearances = []
        self.last_segment_clearances = []
        for point, segment in zip(self.last_waypoints, segments):
            cell = self.cell(*point)
            self.last_waypoint_clearances.append(
                float(self.clearance[cell]) if cell is not None else 0.0)
            self.last_segment_clearances.append(
                self.line_min_clearance(segment[0], segment[1]))

    def route(self, start, goal):
        start = tuple(map(float, start))
        goal = tuple(map(float, goal))
        source, target = self.cell(*start), self.cell(*goal)
        if (source is None or target is None or
                not self.safe[source] or not self.safe[target]):
            raise ValueError("Start or goal is outside the free clearance area")
        if self.component_labels[source] != self.component_labels[target]:
            raise ValueError("start and goal are in different static connected components")
        if self.line_safe(start, goal):
            self.last_raw_grid_path = [start, goal]
            self._save_route_diagnostics(start, [goal], [(start, goal)])
            return [goal]

        offsets = [(dr, dc, math.hypot(dr, dc))
                   for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
        queue = [(0.0, source)]
        costs = {source: 0.0}
        parent = {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == target:
                break
            for dr, dc, step in offsets:
                neighbor = current[0] + dr, current[1] + dc
                if not (0 <= neighbor[0] < self.height and
                        0 <= neighbor[1] < self.width and self.safe[neighbor]):
                    continue
                if dr and dc and not (
                        self.safe[current[0] + dr, current[1]] and
                        self.safe[current[0], current[1] + dc]):
                    continue
                # Slightly favor the middle of the corridor over wall-hugging.
                clearance = float(self.clearance[neighbor])
                new_cost = costs[current] + step * (1.0 + 0.8 / max(clearance, 0.1))
                if new_cost >= costs.get(neighbor, float("inf")):
                    continue
                costs[neighbor], parent[neighbor] = new_cost, current
                heuristic = math.hypot(neighbor[0] - target[0],
                                       neighbor[1] - target[1])
                heapq.heappush(queue, (new_cost + heuristic, neighbor))
        if target not in parent:
            raise ValueError("No collision-free route to goal")
        cells = [target]
        while cells[-1] != source:
            cells.append(parent[cells[-1]])
        raw_cells = list(reversed(cells))
        self.last_raw_grid_path = (
            [start] + [self.point(cell) for cell in raw_cells[1:-1]] + [goal])
        path = [start] + [self.point(cell) for cell in raw_cells[1:-1]] + [goal]
        waypoints = []
        segments = []
        index = 0
        while index < len(path) - 1:
            last = index + 1
            for candidate in range(index + 1, len(path)):
                if math.dist(path[index], path[candidate]) > 3.5:
                    break
                if self.line_safe(path[index], path[candidate]):
                    last = candidate
            segments.append((path[index], path[last]))
            waypoints.append(path[last])
            index = last
        self._save_route_diagnostics(start, waypoints, segments)
        return waypoints
