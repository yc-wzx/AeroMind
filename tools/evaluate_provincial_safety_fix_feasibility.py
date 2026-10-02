#!/usr/bin/env python3
"""Offline feasibility comparison; does not import ROS or publish commands."""
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/uav_planning/scripts'))
from provincial_safety_geometry import body_wall_gap, wall_box, reference_stages

FIELD = json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())
WALLS = [wall_box(segment, .055) for segment in FIELD['collision_segments']]
DATA = ROOT/'tools/results/provincial_low_speed'
OUT = ROOT/'tools/results/provincial_stage15_safety_fix'


def positions(name):
    with (DATA/(name+'.csv')).open(newline='') as stream:
        return [(float(row['gt_x']), float(row['gt_y']), float(row['gt_yaw']))
                for row in csv.DictReader(stream)]


def sampled_reference(stages, start, spacing=.01):
    dense = []
    for a, b in zip([start]+stages[:-1], stages):
        count = max(1, math.ceil(math.dist(a,b)/spacing))
        dense.extend((a[0]+(b[0]-a[0])*i/count,
                      a[1]+(b[1]-a[1])*i/count) for i in range(count+1))
    return dense


def metrics(points, yaw):
    gaps = [body_wall_gap(x,y, yaw if yaw is not None else measured_yaw, WALLS)
            for x,y,measured_yaw in points]
    return {'min_gap_m': round(min(gaps),5),
            'below_0_08_count': sum(gap < .08 for gap in gaps),
            'samples': len(gaps)}


def main():
    result={'classification':'TRAINING-ONLY OFFLINE FEASIBILITY',
            'official_arena_verified':False,
            'guarded_trial_threshold_m':.08,
            'observed_trajectories':{},'reference_route':{}}
    names={'A':'test_a_start','B':'diagnostic_b_turn','C':'diagnostic_c_shoot'}
    for route,name in names.items():
        trace=positions(name)
        result['observed_trajectories'][route]={
            'record':name,
            'measured_yaw':metrics(trace,None),
            'fixed_yaw_0':metrics(trace,0.0),
            'fixed_yaw_90':metrics(trace,math.pi/2)}
    cases={'A':((4.7,.5),(4.7,1.15)),
           'B':((4.7,1.15),(5.8,1.8)),
           'C':((5.8,1.8),(8.7,4.25))}
    for route,(start,goal) in cases.items():
        stages=reference_stages(start,goal,FIELD['reference_route'])
        points=[(x,y,0.0) for x,y in sampled_reference(stages,start)]
        result['reference_route'][route]={
            'stages':stages,'fixed_yaw_0':metrics(points,0.0),
            'fixed_yaw_90':metrics(points,math.pi/2)}
    OUT.mkdir(parents=True,exist_ok=True)
    target=OUT/'offline_feasibility.json'
    target.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(target)


if __name__ == '__main__':
    main()
