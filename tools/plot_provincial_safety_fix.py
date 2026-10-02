#!/usr/bin/env python3
"""Plot old and guarded turn passages on the same provisional collision boxes."""
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import rectangle,wall_box

FIELD=json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())
WALLS=[wall_box(segment,.055) for segment in FIELD['collision_segments']]
OLD=ROOT/'tools/results/provincial_low_speed'
NEW=ROOT/'tools/results/provincial_stage15_safety_fix'
CASES={
    'B':('diagnostic_b_turn','trial_b_05',[(4.7,1.15),(4.7,1.8),(5.8,1.8)],
         (4.35,5.3,1.2,2.3)),
    'C':('diagnostic_c_shoot','trial_c_04',[(5.8,1.8),(8.7,1.8),(8.7,4.25)],
         (8.1,9.15,1.45,2.5)),
}


def read(path):
    with path.open(newline='') as stream:
        return [{k:float(v) for k,v in row.items()} for row in csv.DictReader(stream)]


def draw(route,old_name,new_name,reference,limits):
    old=read(OLD/(old_name+'.csv'))
    new=read(NEW/(new_name+'.csv'))
    fig,axes=plt.subplots(1,2,figsize=(12,6),dpi=140,sharex=True,sharey=True)
    for ax,trace,title,color in ((axes[0],old,'Previous diagnostic','tab:red'),
                                 (axes[1],new,'Guarded centreline trial','tab:green')):
        for wall in WALLS:
            ax.add_patch(Polygon(wall,closed=True,facecolor='.25',edgecolor='.15',alpha=.75))
        ax.plot([p[0] for p in reference],[p[1] for p in reference],
                '--',color='tab:blue',lw=1.5,label='Configured reference')
        valid=[r for r in trace if 'setpoint_x' in r and r['setpoint_x']==r['setpoint_x']]
        if valid:
            ax.plot([r['setpoint_x'] for r in valid],[r['setpoint_y'] for r in valid],
                    color='tab:orange',lw=1.5,label='EGO commanded XY')
        ax.plot([r['gt_x'] for r in trace],[r['gt_y'] for r in trace],
                color=color,lw=1.4,label='Gazebo GT')
        closest=min(trace,key=lambda r:r['wall_clearance'])
        body=rectangle(closest['gt_x'],closest['gt_y'],closest['gt_yaw'],.52,.42)
        ax.add_patch(Polygon(body,closed=True,facecolor=color,alpha=.32,
                             edgecolor=color,label='Body at minimum gap'))
        ax.set_title(f"{title}\nmin GT body gap = {closest['wall_clearance']:.3f} m")
        ax.set_xlim(limits[0],limits[1]);ax.set_ylim(limits[2],limits[3])
        ax.set_aspect('equal',adjustable='box');ax.set_xlabel('x (m)');ax.grid(alpha=.2)
    axes[0].set_ylabel('y (m)')
    axes[1].legend(loc='best',fontsize=7)
    fig.suptitle(f'Provincial trial {route}: same provisional walls, separate runs')
    fig.tight_layout()
    fig.savefig(NEW/f'trial_{route.lower()}_before_after.png')
    plt.close(fig)


def main():
    for route,args in CASES.items():
        draw(route,*args)


if __name__=='__main__':
    main()
