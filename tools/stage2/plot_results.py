#!/usr/bin/env python3
"""Plots actual recorded samples; no interpolation or artificial GT samples."""
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import body_wall_gap
from analyze_provincial_forward_clearance import walls_from_sdf

BASE=ROOT/'tools/results/stage2_steps123_20261001'
walls=walls_from_sdf(ROOT/'src/uav_bringup/worlds/provincial_2025_training.sdf')
fig,axs=plt.subplots(2,2,figsize=(13,9))
for wall in walls:
    polygon=wall['polygon']+[wall['polygon'][0]]
    axs[0,0].fill([p[0] for p in polygon],[p[1] for p in polygon],color='.75')
reference=[[4.7,.5],[4.7,1.8],[8.7,1.8],[8.7,4.25]]
axs[0,0].plot([p[0] for p in reference],[p[1] for p in reference],':',color='.4',label='Reference')
for profile,color in zip(('zero','scale','yaw_bias','random_walk'),('tab:blue','tab:green','tab:red','tab:purple')):
    directory=BASE/profile;name='stage2_'+profile+'_01'
    summary=json.loads((directory/(name+'.summary.json')).read_text())
    start=summary['goal_delivery']['publish_monotonic_ns']
    first=None;gt=[];nav=[];diagnostics=[]
    for line in (directory/(name+'.native.jsonl')).open():
        row=json.loads(line)
        if row['receive_monotonic_ns']<start:continue
        if row['topic'] in ('/gazebo/odometry','/simulation/navigation_odometry'):
            p=row['data']['pose']['pose'];q=p['orientation']
            angle=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
            target=gt if row['topic']=='/gazebo/odometry' else nav
            target.append((row['message_stamp_s'],p['position']['x'],p['position']['y'],angle))
        elif row['topic']=='/simulation/odometry_diagnostics':
            d=json.loads(row['data']['data'])
            if d.get('accepted'):diagnostics.append(d)
    axs[0,0].plot([p[1] for p in gt],[p[2] for p in gt],color=color,label=profile)
    axs[0,0].plot([p[1] for p in nav],[p[2] for p in nav],'--',color=color,alpha=.75)
    if gt:
        offset=gt[0][0]
        gaps=[body_wall_gap(x,y,a,[w['polygon'] for w in walls]) for _,x,y,a in gt]
        axs[1,0].plot([p[0]-offset for p in gt],gaps,color=color,label=profile)
        axs[0,1].plot([d['stamp_s']-offset for d in diagnostics],
                       [d['position_error_m']*100 for d in diagnostics],color=color,label=profile)
        axs[1,1].plot([d['stamp_s']-offset for d in diagnostics],
                       [math.degrees(d['yaw_error_rad']) for d in diagnostics],color=color,label=profile)
axs[0,0].set(xlim=(4.1,9.2),ylim=(0,4.8),aspect='equal',xlabel='World x (m)',ylabel='World y (m)',title='Recorded GT (solid) / injected odom (dashed)')
axs[0,1].set(xlabel='Task sim time (s)',ylabel='Position error (cm)',title='Injected pose error vs ground truth')
axs[1,0].axhline(.08,color='black',linestyle='--',label='Existing 0.08 m gate')
axs[1,0].set(xlabel='Task sim time (s)',ylabel='Sampled body-wall gap (m)',title='Native GT samples; no continuous-time guarantee',ylim=(.06,.23))
axs[1,1].set(xlabel='Task sim time (s)',ylabel='Yaw error (degrees)',title='Injected yaw minus GT yaw')
for ax in axs.flat:ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.tight_layout();target=BASE/'stage2_steps123_native_samples.png'
if target.exists():raise FileExistsError(target)
fig.savefig(target,dpi=150);print(target)
