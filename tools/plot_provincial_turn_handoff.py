#!/usr/bin/env python3
"""Compare the first EGO turn plan before/after measured-speed handoff."""
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import rectangle, wall_box

OUT = ROOT/'tools/results/provincial_stage15_safety_fix'
FIELD = json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())
WALLS = [wall_box(segment, .055) for segment in FIELD['collision_segments']]


def read_trial(name):
    with (OUT/(name+'.csv')).open(newline='') as stream:
        trace = [{key: float(value) for key, value in row.items()}
                 for row in csv.DictReader(stream)]
    plans = [json.loads(line) for line in (OUT/(name+'.plans.jsonl')).read_text().splitlines()]
    turn_plan = next(plan for plan in plans if plan['active_goal'] and
                     plan['active_goal'][1] > 2.0)
    return trace, turn_plan


def main():
    cases = [('trial_c_11', 'Earlier handoff: C11', 'tab:red'),
             ('trial_c_21', 'Measured-speed handoff: C21', 'tab:green')]
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=150,
                             sharex=True, sharey=True)
    for ax, (name, title, color) in zip(axes, cases):
        trace, plan = read_trial(name)
        for wall in WALLS:
            ax.add_patch(Polygon(wall, closed=True, facecolor='.25', edgecolor='.15', alpha=.78))
        plan_xy = plan['points']
        ax.plot([p[0] for p in plan_xy], [p[1] for p in plan_xy],
                color='tab:purple', lw=1.5, label='First EGO turn plan')
        ax.plot([row['gt_x'] for row in trace], [row['gt_y'] for row in trace],
                color=color, lw=1.6, label='Gazebo GT')
        ax.plot([8.7, 8.7], [1.8, 4.25], '--', color='tab:blue',
                lw=1.4, label='Reference centreline')
        start = plan['gt_pose']
        ax.add_patch(Polygon(rectangle(start[0], start[1], start[2], .52, .42),
                             closed=True, facecolor=color, edgecolor=color,
                             alpha=.28, label='Body at handoff'))
        ax.scatter([start[0]], [start[1]], s=20, color=color)
        ax.set_title(f'{title}\nGT handoff x={start[0]:.3f} m')
        ax.set_xlim(8.15, 9.15)
        ax.set_ylim(1.35, 3.25)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('x (m)')
        ax.grid(alpha=.2)
    axes[0].set_ylabel('y (m)')
    axes[1].legend(loc='upper left', fontsize=7)
    fig.suptitle('Training-only C turn: same provisional collision geometry')
    fig.tight_layout()
    path = OUT/'turn_handoff_c_before_after.png'
    fig.savefig(path)
    print(path)


if __name__ == '__main__':
    main()
