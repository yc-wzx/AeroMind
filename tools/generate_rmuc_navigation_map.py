#!/usr/bin/env python3
"""Build separate RMUC ground-traversability and LiDAR-slice occupancy maps.

The formal ground map projects only arena obstacle triangles that intersect
the robot's SDF collision height band. The upstream 2D map is preserved as a
separate reference layer. The LiDAR map is a sensor-height diagnostic only and
is never used as the formal GridRoute map.
"""
from pathlib import Path
import argparse
import json
import math
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / "src/uav_bringup"
WORLD = BRINGUP / "worlds/rmuc_2025.sdf"
MODEL = BRINGUP / "models/rmuc_2025/model.sdf"
MESH = BRINGUP / "models/rmuc_2025/meshes/rmuc_2025.stl"
SOURCE_PGM = BRINGUP / "maps/rmuc_2025_source.pgm"
PGM = BRINGUP / "maps/rmuc_2025.pgm"
LIDAR_PGM = BRINGUP / "maps/rmuc_2025_lidar_slice.pgm"
YAML = BRINGUP / "maps/rmuc_2025.yaml"
AUDIT_JSON = ROOT / "tools/results/rmuc_2025_ground_map_audit.json"
FREE_THRESHOLD = 250


def pose_values(node):
    text = node.findtext("pose") if node is not None else None
    if not text or not text.strip():
        return np.zeros(6, dtype=float)
    values = np.fromstring(text, sep=" ")
    if values.size != 6:
        raise ValueError(f"Expected an SDF six-value pose, got {text!r}")
    return values


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = map(float, pose)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    matrix = np.eye(4)
    matrix[:3, :3] = rz @ ry @ rx
    matrix[:3, 3] = (x, y, z)
    return matrix


def map_metadata():
    text = YAML.read_text(encoding="utf-8")
    resolution_match = re.search(r"(?m)^resolution\s*:\s*([0-9.eE+-]+)\s*$", text)
    origin_match = re.search(r"(?m)^origin\s*:\s*\[([^\]]+)\]\s*$", text)
    if not resolution_match or not origin_match:
        raise ValueError(f"Missing resolution or origin in {YAML}")
    resolution = float(resolution_match.group(1))
    origin = tuple(float(part.strip()) for part in origin_match.group(1).split(","))
    if len(origin) != 3:
        raise ValueError(f"Expected a 3D origin in {YAML}")
    return text, resolution, origin


def align_map_origin():
    text, resolution, origin = map_metadata()
    aligned = re.sub(r"(?m)^origin\s*:\s*\[[^\]]+\]\s*$",
                     "origin: [0, 0, 0]", text)
    if aligned != text:
        YAML.write_text(aligned, encoding="utf-8")
    return resolution, (0.0, 0.0, 0.0), origin


def visual_mesh_transform():
    world_root = ET.parse(WORLD).getroot()
    world = world_root.find("world")
    include = world.find("include")
    if include is not None and include.findtext("name") != "rmuc_2025_arena":
        include = None
    if include is None:
        raise ValueError("World does not include model rmuc_2025_arena")
    world_from_include = pose_matrix(pose_values(include))

    model = ET.parse(MODEL).getroot().find("model")
    link = model.find("link")
    visual = link.find("visual")
    mesh_node = visual.find("./geometry/mesh")
    uri = mesh_node.findtext("uri", "")
    if not uri.endswith("/meshes/rmuc_2025.stl"):
        raise ValueError(f"Unexpected visual mesh URI: {uri}")
    scale = np.fromstring(mesh_node.findtext("scale", "1 1 1"), sep=" ")
    if scale.size != 3:
        raise ValueError(f"Invalid visual mesh scale: {scale}")
    world_from_mesh = (
        world_from_include
        @ pose_matrix(pose_values(model))
        @ pose_matrix(pose_values(link))
        @ pose_matrix(pose_values(visual))
    )

    robot = world.find("./model[@name='omni_robot']")
    if robot is None:
        raise ValueError("World does not contain omni_robot")
    robot_link = robot.find("link")
    sensor = robot_link.find("./sensor[@name='lidar']")
    if sensor is None:
        raise ValueError("omni_robot has no lidar sensor")
    world_from_sensor = (
        pose_matrix(pose_values(robot))
        @ pose_matrix(pose_values(robot_link))
        @ pose_matrix(pose_values(sensor))
    )
    sensor_z = float(world_from_sensor[2, 3])
    if abs(world_from_sensor[2, 0]) > 1e-8 or abs(world_from_sensor[2, 1]) > 1e-8:
        raise ValueError("LiDAR scan plane is tilted; a horizontal PGM slice is invalid")
    return world_from_mesh, scale, sensor_z, uri


def robot_collision_z_range():
    world = ET.parse(WORLD).getroot().find("world")
    robot = world.find("./model[@name='omni_robot']")
    if robot is None:
        raise ValueError("World does not contain omni_robot")
    model_pose = pose_matrix(pose_values(robot))
    bounds = []
    for link in robot.findall("link"):
        world_from_link = model_pose @ pose_matrix(pose_values(link))
        for collision in link.findall("collision"):
            box = collision.find("./geometry/box/size")
            if box is None:
                raise ValueError(
                    f"Unsupported robot collision geometry in {collision.get('name')}; "
                    "ground-map height must come from explicit collision geometry")
            size = np.fromstring(box.text or "", sep=" ")
            if size.size != 3 or np.any(size <= 0):
                raise ValueError(f"Invalid collision box size in {collision.get('name')}")
            world_from_box = world_from_link @ pose_matrix(pose_values(collision))
            half = size * 0.5
            corners = np.array([
                [sx * half[0], sy * half[1], sz * half[2], 1.0]
                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
            ])
            world_z = (world_from_box @ corners.T)[2]
            bounds.extend((float(world_z.min()), float(world_z.max())))
    if not bounds:
        raise ValueError("omni_robot has no supported collision boxes")
    return min(bounds), max(bounds)


def read_binary_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    record = np.dtype([
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (9,)),
        ("attribute", "<u2"),
    ])
    if len(data) != 84 + count * 50:
        raise ValueError(f"Expected binary STL with {count} triangles: {path}")
    triangles = np.frombuffer(data, dtype=record, offset=84, count=count)
    return triangles["vertices"].reshape(count, 3, 3).astype(np.float64)



def mesh_face_open_mask(triangles):
    """Mark STL facets touching a boundary or non-manifold mesh edge."""
    vertices = np.asarray(triangles, dtype="<f4").reshape(-1, 3)
    _, inverse = np.unique(vertices, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edges.sort(axis=1)
    vertex_count = int(inverse.max()) + 1
    keys = edges[:, 0].astype(np.uint64) * vertex_count + edges[:, 1]
    _, edge_inverse, edge_counts = np.unique(
        keys, return_counts=True, return_inverse=True)
    face_edge_counts = edge_counts[edge_inverse.reshape(-1, 3)]
    open_faces = np.any(face_edge_counts != 2, axis=1)
    topology = {
        "unique_vertices": int(vertex_count),
        "unique_edges": int(len(edge_counts)),
        "boundary_edges": int(np.count_nonzero(edge_counts == 1)),
        "nonmanifold_edges": int(np.count_nonzero(edge_counts > 2)),
        "faces_touching_boundary_or_nonmanifold_edges": int(open_faces.sum()),
    }
    return open_faces, topology

def mark_xy(projected, x, y, resolution, origin):
    height, width = projected.shape
    col = math.floor((x - origin[0]) / resolution)
    row = height - 1 - math.floor((y - origin[1]) / resolution)
    if 0 <= row < height and 0 <= col < width:
        projected[row, col] = True


def mark_segment(projected, a, b, resolution, origin):
    distance = math.hypot(float(b[0] - a[0]), float(b[1] - a[1]))
    steps = max(1, int(math.ceil(distance / (resolution * 0.25))))
    for index in range(steps + 1):
        point = a + (index / steps) * (b - a)
        mark_xy(projected, float(point[0]), float(point[1]), resolution, origin)


def rasterize_polygon(projected, polygon, resolution, origin):
    if len(polygon) < 2:
        return
    height, width = projected.shape
    xy = np.asarray(polygon, dtype=float)[:, :2]
    for index in range(len(xy)):
        mark_segment(projected, xy[index], xy[(index + 1) % len(xy)],
                     resolution, origin)
    if len(xy) < 3:
        return
    min_col = max(0, math.floor((float(xy[:, 0].min()) - origin[0]) / resolution))
    max_col = min(width - 1, math.floor((float(xy[:, 0].max()) - origin[0]) / resolution))
    min_row = max(0, height - 1 - math.floor((float(xy[:, 1].max()) - origin[1]) / resolution))
    max_row = min(height - 1, height - 1 - math.floor((float(xy[:, 1].min()) - origin[1]) / resolution))
    if min_col > max_col or min_row > max_row:
        return
    cols = np.arange(min_col, max_col + 1)
    rows = np.arange(min_row, max_row + 1)
    xs = origin[0] + (cols + 0.5) * resolution
    ys = origin[1] + (height - rows - 0.5) * resolution
    xx, yy = np.meshgrid(xs, ys)
    inside = np.zeros(xx.shape, dtype=bool)
    xj, yj = xy[-1]
    for xi, yi in xy:
        crosses = ((yi > yy) != (yj > yy)) & (
            xx < (xj - xi) * (yy - yi) / ((yj - yi) + 1e-300) + xi
        )
        inside ^= crosses
        xj, yj = xi, yi
    projected[min_row:max_row + 1, min_col:max_col + 1] |= inside


def clip_polygon_z(polygon, bound, keep_above):
    if not polygon:
        return []
    output = []
    previous = polygon[-1]
    previous_inside = (previous[2] >= bound) if keep_above else (previous[2] <= bound)
    for current in polygon:
        current_inside = (current[2] >= bound) if keep_above else (current[2] <= bound)
        if current_inside != previous_inside:
            denominator = current[2] - previous[2]
            if abs(denominator) > 1e-15:
                ratio = (bound - previous[2]) / denominator
                output.append(previous + ratio * (current - previous))
        if current_inside:
            output.append(current)
        previous = current
        previous_inside = current_inside
    return output


def clip_triangle_to_slab(triangle, z_min, z_max):
    polygon = [point.copy() for point in triangle]
    polygon = clip_polygon_z(polygon, z_min, True)
    polygon = clip_polygon_z(polygon, z_max, False)
    return polygon


def detect_ground_plane_z(triangles):
    z_span = np.ptp(triangles[:, :, 2], axis=1)
    flat = z_span <= 1e-5
    if not np.any(flat):
        raise ValueError("Visual mesh has no horizontal plane to identify as the ground datum")
    xy_span = np.ptp(triangles[:, :, :2], axis=1)
    bbox_area = xy_span[:, 0] * xy_span[:, 1]
    candidates = np.flatnonzero(flat)
    largest = candidates[int(np.argmax(bbox_area[candidates]))]
    ground_z = float(np.mean(triangles[largest, :, 2]))
    coplanar_ground = flat & (np.max(np.abs(triangles[:, :, 2] - ground_z), axis=1) <= 5e-4)
    return ground_z, int(coplanar_ground.sum())


def is_ground_support_triangle(triangle, ground_plane_z):
    return (float(np.ptp(triangle[:, 2])) <= 1e-5 and
            float(np.max(np.abs(triangle[:, 2] - ground_plane_z))) <= 5e-4)


def project_mesh_slab(triangles, z_min, z_max, resolution, origin, shape,
                      ground_plane_z):
    projected = np.zeros(shape, dtype=bool)
    intersecting_triangles = 0
    excluded_ground_triangles = 0
    for triangle in triangles:
        if is_ground_support_triangle(triangle, ground_plane_z):
            excluded_ground_triangles += 1
            continue
        polygon = clip_triangle_to_slab(triangle, z_min, z_max)
        if len(polygon) >= 2:
            intersecting_triangles += 1
            rasterize_polygon(projected, polygon, resolution, origin)
    return projected, intersecting_triangles, excluded_ground_triangles


def project_mesh_slice(triangles, sensor_z, resolution, origin, shape,
                       ground_plane_z=None):
    projected = np.zeros(shape, dtype=bool)
    intersecting_triangles = 0
    epsilon = 1e-8
    for triangle in triangles:
        if (ground_plane_z is not None and
                is_ground_support_triangle(triangle, ground_plane_z)):
            continue
        if np.all(np.abs(triangle[:, 2] - sensor_z) <= epsilon):
            intersecting_triangles += 1
            rasterize_polygon(projected, list(triangle), resolution, origin)
            continue
        intersections = []
        for first, second in ((0, 1), (1, 2), (2, 0)):
            a, b = triangle[first], triangle[second]
            za, zb = float(a[2] - sensor_z), float(b[2] - sensor_z)
            if abs(za) <= epsilon:
                intersections.append(a)
            if abs(zb) <= epsilon:
                intersections.append(b)
            if za * zb < 0:
                intersections.append(a + (-za / (zb - za)) * (b - a))
        unique = []
        for point in intersections:
            if not any(np.linalg.norm(point - prior) < 1e-7 for prior in unique):
                unique.append(point)
        if len(unique) >= 2:
            intersecting_triangles += 1
            if len(unique) >= 3:
                rasterize_polygon(projected, unique, resolution, origin)
            else:
                mark_segment(projected, unique[0], unique[1], resolution, origin)
    return projected, intersecting_triangles


def save_occupancy(path, source, occupied):
    image = np.asarray(source, dtype=np.uint8).copy()
    image[occupied] = 0
    Image.fromarray(image, mode="L").save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vertical-margin", type=float, default=0.02,
                        help="extra vertical collision margin in metres (default: 0.02)")
    parser.add_argument("--robot-z-min", type=float, default=None,
                        help="override world-space robot collision minimum z")
    parser.add_argument("--robot-z-max", type=float, default=None,
                        help="override world-space robot collision maximum z")
    args = parser.parse_args()
    if args.vertical_margin < 0:
        parser.error("--vertical-margin must be non-negative")
    if not SOURCE_PGM.exists():
        raise FileNotFoundError(
            f"Missing upstream source map {SOURCE_PGM}; run tools/import_rmuc_assets.py first")

    resolution, origin, old_origin = align_map_origin()
    source = np.asarray(Image.open(SOURCE_PGM).convert("L"), dtype=np.uint8)
    transform, scale, sensor_z, uri = visual_mesh_transform()
    z_min_model, z_max_model = robot_collision_z_range()
    robot_z_min = z_min_model if args.robot_z_min is None else args.robot_z_min
    robot_z_max = z_max_model if args.robot_z_max is None else args.robot_z_max
    if robot_z_min >= robot_z_max:
        parser.error("robot z minimum must be below robot z maximum")
    sweep_z_min = robot_z_min - args.vertical_margin
    sweep_z_max = robot_z_max + args.vertical_margin

    local_triangles = read_binary_stl(MESH)
    open_faces, mesh_topology = mesh_face_open_mask(local_triangles)
    triangles = local_triangles * scale[None, None, :]
    triangles = triangles @ transform[:3, :3].T + transform[:3, 3]
    ground_plane_z, ground_plane_triangles = detect_ground_plane_z(triangles)
    non_support_faces = np.array([
        not is_ground_support_triangle(triangle, ground_plane_z)
        for triangle in triangles
    ], dtype=bool)
    open_faces &= non_support_faces
    lidar_slice, lidar_triangles = project_mesh_slice(
        triangles, sensor_z, resolution, origin, source.shape, ground_plane_z)
    open_mesh_lidar, _ = project_mesh_slice(
        triangles[open_faces], sensor_z, resolution, origin, source.shape)
    sampled_body_surface = np.zeros(source.shape, dtype=bool)
    for sample_z in np.arange(robot_z_min, robot_z_max + 1e-9, 0.01):
        sample, _ = project_mesh_slice(
            triangles, float(sample_z), resolution, origin, source.shape, ground_plane_z)
        sampled_body_surface |= sample
    nominal_surface, nominal_body_triangles, _ = project_mesh_slab(
        triangles, robot_z_min, robot_z_max, resolution, origin, source.shape, ground_plane_z)
    ground_surface, body_triangles, excluded_ground_triangles = project_mesh_slab(
        triangles, sweep_z_min, sweep_z_max, resolution, origin, source.shape, ground_plane_z)
    # Rasterization at the expanded limits can differ by a boundary pixel. The
    # safety-expanded set must always contain the nominal collision-height set.
    ground_surface |= nominal_surface

    source_occupied = source < FREE_THRESHOLD
    lidar_candidates = lidar_slice & ~source_occupied
    legacy_sampled_upper_only = lidar_candidates & ~sampled_body_surface
    legacy_exact_body_hits = legacy_sampled_upper_only & nominal_surface
    legacy_margin_body_hits = legacy_sampled_upper_only & ground_surface
    type_b_nominal = lidar_candidates & nominal_surface
    type_c_nominal = lidar_candidates & ~nominal_surface & open_mesh_lidar
    type_a_nominal = lidar_candidates & ~nominal_surface & ~open_mesh_lidar
    type_b_margin = lidar_candidates & ground_surface
    type_c_margin = lidar_candidates & ~ground_surface & open_mesh_lidar
    type_a_margin = lidar_candidates & ~ground_surface & ~open_mesh_lidar
    legacy_type_a = legacy_sampled_upper_only & ~ground_surface & ~open_mesh_lidar
    legacy_type_c = legacy_sampled_upper_only & ~ground_surface & open_mesh_lidar
    ground_occupied = ground_surface

    blank = np.full(source.shape, 255, dtype=np.uint8)
    save_occupancy(PGM, blank, ground_occupied)
    lidar_image = np.full(source.shape, 255, dtype=np.uint8)
    lidar_image[lidar_slice] = 0
    Image.fromarray(lidar_image, mode="L").save(LIDAR_PGM)

    audit = {
        "classification": {
            "nominal_body_band": {
                "A_upper_only_closed_surface": int(type_a_nominal.sum()),
                "B_intersects_robot_collision_z_range": int(type_b_nominal.sum()),
                "C_open_or_nonmanifold_upper_surface": int(type_c_nominal.sum()),
            },
            "prior_500_candidate_cells": {
                "sampled_slice_upper_only": int(legacy_sampled_upper_only.sum()),
                "exact_body_band_reclassified_as_B": int(legacy_exact_body_hits.sum()),
                "vertical_margin_reclassified_as_B": int(
                    (legacy_margin_body_hits & ~nominal_surface).sum()),
                "still_A_after_margin": int(legacy_type_a.sum()),
                "still_C_unresolved_after_margin": int(legacy_type_c.sum()),
            },
            "with_vertical_margin": {
                "A_outside_expanded_sweep": int(type_a_margin.sum()),
                "B_intersects_expanded_sweep": int(type_b_margin.sum()),
                "A_promoted_to_B_by_margin": int(type_b_margin.sum() - type_b_nominal.sum()),
            },
            "C_open_or_nonmanifold_upper_surface": int(type_c_margin.sum()),
            "method": (
                "Exact triangle clipping to the robot collision z slab; candidates "
                "with no mesh surface in the slab are excluded from ground occupancy. "
                "Gazebo collision is generated from this visual STL with walkable "
                "support triangles removed; no hidden solid volume is inferred from "
                "the LiDAR slice."
            ),
        },
        "geometry": {
            "visual_mesh_uri": uri,
            "triangles": int(len(triangles)),
            "lidar_world_z_m": float(sensor_z),
            "arena_ground_plane_z_m": float(ground_plane_z),
            "arena_ground_plane_triangles": int(ground_plane_triangles),
            "excluded_traversable_ground_triangles": int(excluded_ground_triangles),
            "mesh_topology": mesh_topology,
            "robot_collision_z_min_m": float(robot_z_min),
            "robot_collision_z_max_m": float(robot_z_max),
            "vertical_margin_m": float(args.vertical_margin),
            "ground_sweep_z_min_m": float(sweep_z_min),
            "ground_sweep_z_max_m": float(sweep_z_max),
        },
        "map": {
            "resolution_m": float(resolution),
            "shape_rows_cols": [int(source.shape[0]), int(source.shape[1])],
            "aligned_origin": list(origin),
            "upstream_source_occupied_cells": int(source_occupied.sum()),
            "lidar_slice_triangles": int(lidar_triangles),
            "lidar_slice_mesh_cells": int(lidar_slice.sum()),
            "lidar_only_candidate_cells": int(lidar_candidates.sum()),
            "ground_slab_triangles": int(body_triangles),
            "ground_mesh_cells": int(ground_surface.sum()),
            "ground_occupied_cells": int(ground_occupied.sum()),
        },
        "outputs": {
            "ground_traversability_pgm": str(PGM),
            "lidar_slice_pgm": str(LIDAR_PGM),
            "source_pgm": str(SOURCE_PGM),
        },
    }
    AUDIT_JSON.parent.mkdir(parents=True, exist_ok=True)
    AUDIT_JSON.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")

    print(f"visual_mesh={uri}")
    print(f"map_origin_old={list(old_origin)} map_origin_aligned={list(origin)}")
    print(f"map_resolution={resolution:.6f} size={source.shape[1]}x{source.shape[0]}")
    print(f"lidar_world_z={sensor_z:.6f} robot_collision_z=[{robot_z_min:.3f},{robot_z_max:.3f}]")
    print(f"vertical_margin={args.vertical_margin:.3f} ground_sweep_z=[{sweep_z_min:.3f},{sweep_z_max:.3f}]")
    print(f"arena_ground_plane_z={ground_plane_z:.6f}m excluded_support_triangles={excluded_ground_triangles}")
    print(f"mesh_triangles={len(triangles)} lidar_slice_triangles={lidar_triangles} "
          f"ground_slab_triangles={body_triangles}")
    print(f"upstream_source_occupied={int(source_occupied.sum())} lidar_slice_cells={int(lidar_slice.sum())} "
          f"lidar_only_candidates={int(lidar_candidates.sum())}")
    print(f"candidate_nominal_A_closed_upper={int(type_a_nominal.sum())} "
          f"B_body_collision={int(type_b_nominal.sum())} "
          f"C_open_upper={int(type_c_nominal.sum())}")
    print(f"candidate_with_margin_A={int(type_a_margin.sum())} "
          f"B={int(type_b_margin.sum())} C={int(type_c_margin.sum())} "
          f"margin_promoted={int(type_b_margin.sum() - type_b_nominal.sum())}")
    print(f"prior_500: sampled_upper_only={int(legacy_sampled_upper_only.sum())} "
          f"exact_band_reclassified_B={int(legacy_exact_body_hits.sum())} "
          f"margin_reclassified_B={int((legacy_margin_body_hits & ~nominal_surface).sum())} "
          f"still_A={int(legacy_type_a.sum())} still_C={int(legacy_type_c.sum())}")
    print(f"ground_mesh_cells={int(ground_surface.sum())} "
          f"ground_occupied_cells={int(ground_occupied.sum())}")
    print(f"ground_map={PGM}")
    print(f"lidar_slice_map={LIDAR_PGM}")
    print(f"audit={AUDIT_JSON}")


if __name__ == "__main__":
    main()

