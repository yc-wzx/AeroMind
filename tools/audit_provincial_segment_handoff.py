#!/usr/bin/env python3
"""Offline rectangular-body audit of A/B/C boundary poses on the training SDF.

No ROS node is started and no navigation target is published.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/uav_planning/scripts'))
from provincial_safety_geometry import body_wall_gap, rectangle

SOURCE = ROOT / 'tools/results/provincial_stage15_safety_fix'
WORLD = ROOT / 'src/uav_bringup/worlds/provincial_2025_training.sdf'
OUT = ROOT / 'tools/results/provincial_stage15_terminal_validation_20260928/handoff_offline.json'


def sdf_walls():
    walls = []
    for model in ET.parse(WORLD).findall('.//model'):
        if not model.get('name', '').startswith('boundary_'):
            continue
        pose = [float(value) for value in model.findtext('pose').split()]
        size = [float(value) for value in
                model.findtext('link/collision/geometry/box/size').split()]
        walls.append(rectangle(pose[0], pose[1], pose[5], size[0], size[1]))
    return walls


def line_minimum(start, goal, yaw, walls, sample_step=0.025):
    count = max(1, math.ceil(math.dist(start, goal) / sample_step))
    gaps = [body_wall_gap(start[0]+(goal[0]-start[0])*i/count,
                          start[1]+(goal[1]-start[1])*i/count,
                          yaw, walls) for i in range(count+1)]
    return min(gaps), count+1


def rotation_minimum(point, walls, yaw_start=0.0, yaw_end=math.pi/2):
    steps = math.ceil(abs(yaw_end-yaw_start)/math.radians(1))
    return min(body_wall_gap(*point, yaw_start+(yaw_end-yaw_start)*i/steps,
                             walls) for i in range(steps+1))


def main():
    walls = sdf_walls()
    a = json.loads((SOURCE/'trial_a_27.summary.json').read_text())
    b = json.loads((SOURCE/'trial_b_16.summary.json').read_text())
    c = json.loads((SOURCE/'trial_c_23.summary.json').read_text())
    reference = json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())['reference_route']
    segments = {
        'A_end_to_B_goal': [a['final_gt_xy'], reference[1], [5.8, 1.8]],
        'B_end_to_C_goal': [b['final_gt_xy'], reference[2], reference[3]],
    }
    legs = {}
    for name, points in segments.items():
        legs[name] = {}
        for label, yaw in (('zero', 0.0), ('A_held_90deg', math.pi/2)):
            measures = [line_minimum(first, second, yaw, walls)
                        for first, second in zip(points, points[1:])]
            legs[name][label] = {
                'min_sampled_gap_m': min(measure[0] for measure in measures),
                'sample_count': sum(measure[1] for measure in measures),
                'passes_0p08m_static_gate': min(measure[0] for measure in measures) >= 0.08,
            }
    rotations = {
        name: {'min_sampled_gap_m': rotation_minimum(point, walls),
               'passes_0p08m_static_gate': rotation_minimum(point, walls) >= 0.08}
        for name, point in {
            'A_final_actual': a['final_gt_xy'],
            'B_nominal_start': [4.7, 1.15],
            'first_corner': [4.7, 1.8],
            'B_final_actual': b['final_gt_xy'],
            'second_corner': [8.7, 1.8],
        }.items()
    }
    result = {
        'scope': 'TRAINING-ONLY OFFLINE STATIC GEOMETRY; NOT A FULL RUN',
        'sdf': str(WORLD), 'body_m': [0.52, 0.42],
        'path_sample_step_max_m': 0.025, 'rotation_sample_step_max_deg': 1.0,
        'gate_m': 0.08,
        'historical_pose': {
            'A_final': [*a['final_gt_xy'], a['final_gt_yaw_rad']],
            'B_initial': b['initial_gt_pose'],
            'B_final': [*b['final_gt_xy'], b['final_gt_yaw_rad']],
            'C_initial': c['initial_gt_pose'],
            'C_final': [*c['final_gt_xy'], c['final_gt_yaw_rad']],
            'final_yaw_required': False,
        },
        'legs': legs, 'in_place_rotations': rotations,
        'limitations': ['Straight reference legs at fixed yaw only',
                        'Discrete static geometry, not a continuous-time sweep',
                        'Does not validate planner, controller, or moving obstacles'],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    if OUT.exists():
        raise FileExistsError(OUT)
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
