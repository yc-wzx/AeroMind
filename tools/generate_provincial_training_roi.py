"""Build a provisional flat 2D training map from the 2025 provincial rules sketch.

This is not a verified arena map: lane width and boundaries are provisional.
The generated PGM is for offline reachability diagnosis only.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
FIELD = ROOT / "src/uav_planning/config/competition_field_2025.json"
MAP = ROOT / "src/uav_bringup/maps/provincial_2025_flat_training.pgm"
RESULT = ROOT / "tools/results/provincial_stage15_training_offline_audit.json"
RESOLUTION = 0.05
CLEARANCE = 0.55


def main() -> None:
    field = json.loads(FIELD.read_text(encoding="utf-8"))
    width = round(field["arena"]["width"] / RESOLUTION)
    height = round(field["arena"]["height"] / RESOLUTION)
    image = Image.new("L", (width, height), 0)
    draw = ImageDraw.Draw(image)

    # Only the three automatic-lane rectangles are relevant. They are 2025
    # sketch-derived training approximations, not organizer-confirmed edges.
    for region in field["course_surfaces"][:3]:
        x0 = (region["x"] - region["width"] / 2) / RESOLUTION
        x1 = (region["x"] + region["width"] / 2) / RESOLUTION
        y0 = (region["y"] - region["height"] / 2) / RESOLUTION
        y1 = (region["y"] + region["height"] / 2) / RESOLUTION
        draw.rectangle(
            (int(x0), height - 1 - int(y1), int(x1), height - 1 - int(y0)),
            fill=255,
        )
    for x1, y1, x2, y2 in field["collision_segments"]:
        draw.line(
            (
                (round(x1 / RESOLUTION), height - 1 - round(y1 / RESOLUTION)),
                (round(x2 / RESOLUTION), height - 1 - round(y2 / RESOLUTION)),
            ),
            fill=0,
            width=2,
        )

    MAP.parent.mkdir(parents=True, exist_ok=True)
    image.save(MAP)
    MAP.with_suffix(".yaml").write_text(
        "image: provincial_2025_flat_training.pgm\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )

    sys.path.insert(0, str(ROOT / "src/uav_planning/scripts"))
    from grid_route import GridRoute

    route = GridRoute(MAP, clearance=CLEARANCE)
    pairs = {
        "legacy_center_start_to_shooting_center": ((4.7, 0.5), (8.7, 4.25)),
        "provisional_flat_approach": ((4.7, 0.7), (4.7, 1.3)),
        "provisional_turn": ((4.7, 1.3), (5.8, 1.8)),
        "provisional_full_route": ((4.7, 0.7), (8.7, 4.0)),
    }
    checks = {}
    for name, (start, goal) in pairs.items():
        result = {"start": start, "goal": goal}
        for label, point in (("start", start), ("goal", goal)):
            cell = route.cell(*point)
            result[label + "_clearance_m"] = (
                round(float(route.clearance[cell]), 3) if cell is not None else None
            )
        try:
            waypoints = route.route(start, goal)
            raw = route.last_raw_grid_path
            result.update(
                reachable=True,
                waypoints=len(waypoints),
                raw_path_length_m=round(
                    sum(math.dist(a, b) for a, b in zip(raw, raw[1:])), 3
                ),
                raw_path_min_clearance_m=round(
                    min(float(route.clearance[route.cell(*point)]) for point in raw),
                    3,
                ),
                smoothed_segment_min_clearance_m=round(
                    min(route.last_segment_clearances), 3
                ),
            )
        except ValueError as error:
            result.update(reachable=False, reason=str(error))
        checks[name] = result
    output = {
        "source": "2025 provincial tentative rules figure; reconstructed flat lane",
        "status": "TRAINING_ONLY_NOT_ARENA_VERIFICATION",
        "resolution_m": RESOLUTION,
        "clearance_m": CLEARANCE,
        "terrain": "flat per current user confirmation; not a wheel-contact model",
        "checks": checks,
    }
    RESULT.parent.mkdir(parents=True, exist_ok=True)
    RESULT.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
