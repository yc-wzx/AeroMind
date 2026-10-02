#!/usr/bin/env python3
"""Create an arena collision STL from the visual mesh, excluding the walkable ground skin.

The output preserves all non-ground triangles at their real 3D heights. The large
horizontal support surface is omitted from physics collision so it does not
intersect the robot body; arena walls, ramps, and elevated structures remain.
"""
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / "src/uav_bringup"
MODEL = BRINGUP / "models/rmuc_2025/model.sdf"
MESH = BRINGUP / "models/rmuc_2025/meshes/rmuc_2025.stl"
OUT = BRINGUP / "models/rmuc_2025/meshes/rmuc_2025_obstacles_collision.stl"
sys.path.insert(0, str(ROOT / "tools"))
from generate_rmuc_navigation_map import (  # noqa: E402
    detect_ground_plane_z,
    is_ground_support_triangle,
    pose_matrix,
    pose_values,
    read_binary_stl,
    visual_mesh_transform,
)


def main():
    tree = ET.parse(MODEL)
    root = tree.getroot()
    visual_mesh = root.find("./model/link/visual/geometry/mesh")
    collision_mesh = root.find("./model/link/collision/geometry/mesh")
    if visual_mesh is None or collision_mesh is None:
        raise ValueError("Arena visual/collision must both use mesh geometry")
    uri = visual_mesh.findtext("uri", "")
    collision_uri = collision_mesh.findtext("uri", "")
    expected_uri = "model://rmuc_2025/meshes/rmuc_2025_obstacles_collision.stl"
    if collision_uri != expected_uri:
        raise ValueError(f"Expected collision URI {expected_uri}, got {collision_uri}")
    visual_scale = np.fromstring(visual_mesh.findtext("scale", "1 1 1"), sep=" ")
    collision_scale = np.fromstring(collision_mesh.findtext("scale", "1 1 1"), sep=" ")
    if visual_scale.size != 3 or collision_scale.size != 3 or not np.allclose(
            visual_scale, collision_scale):
        raise ValueError("Visual/collision mesh scales must match")
    visual_link = root.find("./model/link/visual")
    collision_link = root.find("./model/link/collision")
    visual_pose = pose_values(visual_link)
    collision_pose = pose_values(collision_link)
    if not np.allclose(visual_pose, collision_pose):
        raise ValueError("Visual/collision mesh poses must match")

    data = MESH.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid STL: {MESH}")
    count = struct.unpack_from("<I", data, 80)[0]
    record_dtype = np.dtype([
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (9,)),
        ("attribute", "<u2"),
    ])
    if len(data) != 84 + count * 50:
        raise ValueError(f"Expected binary STL with {count} triangles: {MESH}")
    records = np.frombuffer(data, dtype=record_dtype, offset=84, count=count)
    local_triangles = records["vertices"].reshape(count, 3, 3).astype(np.float64)

    transform, scale, _, visual_uri = visual_mesh_transform()
    if visual_uri != uri:
        raise ValueError("Navigation map and arena SDF resolve different visual meshes")
    world_triangles = local_triangles * scale[None, None, :]
    world_triangles = world_triangles @ transform[:3, :3].T + transform[:3, 3]
    ground_z, support_count = detect_ground_plane_z(world_triangles)
    keep = np.array([
        not is_ground_support_triangle(triangle, ground_z)
        for triangle in world_triangles
    ], dtype=bool)
    if int((~keep).sum()) != support_count:
        raise ValueError("Ground-plane filter count disagrees with navigation map generator")

    output_records = records[keep]
    with OUT.open("wb") as output:
        output.write(b"RMUC 2025 3D obstacle mesh; walkable support skin excluded".ljust(80, b"\0"))
        output.write(struct.pack("<I", len(output_records)))
        output.write(output_records.tobytes())

    print(f"visual_mesh={uri}")
    print(f"collision_mesh={OUT}")
    print(f"arena_ground_plane_z={ground_z:.6f} m")
    print(f"excluded_walkable_support_triangles={support_count}")
    print(f"retained_3d_collision_triangles={len(output_records)}")
    print("collision geometry retains original 3D heights; no occupancy extrusion")


if __name__ == "__main__":
    main()

