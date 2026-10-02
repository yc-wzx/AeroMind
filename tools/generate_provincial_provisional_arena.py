"""Generate the flat provisional provincial world, navigation field and PGM.

All three products are derived from tools/config/provincial_2025_provisional_source.json.
The 2025 drawing is a development reference, not the verified final arena.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tools/config/provincial_2025_provisional_source.json"
FIELD = ROOT / "src/uav_planning/config/provincial_2025_provisional.json"
WORLD = ROOT / "src/uav_bringup/worlds/provincial_2025_training.sdf"
MAP = ROOT / "src/uav_bringup/maps/provincial_2025_provisional.pgm"


def centered_rect(x0, y0, x1, y1):
    return {
        "x": round((x0 + x1) / 2, 6),
        "y": round((y0 + y1) / 2, 6),
        "width": round(x1 - x0, 6),
        "height": round(y1 - y0, 6),
    }


def build_geometry(source):
    width = source["auto_passage_clear_width_m"]
    thickness = source["training_wall_thickness_m"]
    route = source["reference_route"]
    start, first, second, goal = route
    assert start[0] == first[0] and first[1] == second[1] and second[0] == goal[0]
    assert width > 0 and thickness > 0
    left, first_right = start[0] - width / 2, start[0] + width / 2
    second_left, right = second[0] - width / 2, second[0] + width / 2
    lower, upper = first[1] - width / 2, first[1] + width / 2
    top = source["shooting_zone"]["y"] + source["shooting_zone"]["height"] / 2
    assert 0 < lower < upper < top < source["arena"]["height"]
    assert 0 < left < first_right < second_left < right < source["arena"]["width"]
    rects = [
        (left, 0.0, first_right, upper),
        (left, lower, right, upper),
        (second_left, lower, right, top),
    ]
    half = thickness / 2
    aw, ah = source["arena"]["width"], source["arena"]["height"]
    # Each segment is the centerline of a wall whose inner face lies on the
    # traversable polygon boundary. The arena perimeter closes the start end.
    walls = [
        (-half, -half, aw + half, -half),
        (aw + half, -half, aw + half, ah + half),
        (aw + half, ah + half, -half, ah + half),
        (-half, ah + half, -half, -half),
        (left - half, 0, left - half, upper),
        (first_right + half, 0, first_right + half, lower),
        (first_right, lower - half, right, lower - half),
        (left, upper + half, second_left, upper + half),
        (second_left - half, upper, second_left - half, top),
        (right + half, lower, right + half, top),
        (second_left, top + half, right, top + half),
    ]
    return rects, [[round(v, 6) for v in line] for line in walls]


def build_map(source, rects):
    resolution = source["map_resolution_m"]
    width = round(source["arena"]["width"] / resolution)
    height = round(source["arena"]["height"] / resolution)
    xs = (np.arange(width) + 0.5) * resolution
    ys = (height - np.arange(height) - 0.5) * resolution
    X, Y = np.meshgrid(xs, ys)
    free = np.zeros((height, width), dtype=bool)
    for x0, y0, x1, y1 in rects:
        free |= (x0 <= X) & (X < x1) & (y0 <= Y) & (Y < y1)
    return Image.fromarray(np.where(free, 255, 0).astype(np.uint8), mode="L")


def main():
    source = json.loads(SOURCE.read_text(encoding="utf-8-sig"))
    rects, walls = build_geometry(source)
    field = {
        "name": source["name"],
        "world_name": source["world_name"],
        "source_note": source["source"]["width_derivation"],
        "arena": source["arena"],
        "start": source["start"],
        "shooting_zone": source["shooting_zone"],
        "target_zone": source["target_zone"],
        "reference_route": source["reference_route"],
        "course_surfaces": [centered_rect(*r) for r in rects],
        "collision_segments": walls,
        "assumptions": {
            "course_width_m": source["auto_passage_clear_width_m"],
            "course_width_basis": source["source"]["width_derivation"],
            "boundary_type": source["source"]["fixed_wall_uncertainty"],
            "terrain": source["terrain"],
            "final_arena_verified": False,
        },
    }
    FIELD.parent.mkdir(parents=True, exist_ok=True)
    FIELD.write_text(json.dumps(field, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    MAP.parent.mkdir(parents=True, exist_ok=True)
    build_map(source, rects).save(MAP)
    MAP.with_suffix(".yaml").write_text(
        "image: provincial_2025_provisional.pgm\n"
        f"resolution: {source['map_resolution_m']}\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    subprocess.run(
        [sys.executable, str(ROOT / "tools/generate_competition_world.py"),
         "--field", str(FIELD), "--world", str(WORLD)],
        check=True,
    )
    print(json.dumps({
        "source": str(SOURCE),
        "field": str(FIELD),
        "world": str(WORLD),
        "map": str(MAP),
        "clear_width_m": source["auto_passage_clear_width_m"],
        "provisional": True,
    }, indent=2))


if __name__ == "__main__":
    main()
