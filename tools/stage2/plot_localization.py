#!/usr/bin/env python3
"""Original native samples only; no invented higher-rate ground truth."""
import json
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_localized import ROOT, load_jsonl, read, yaw

BASE=ROOT/'tools/results/stage2_localization_20261001'


def plot():
    fig, axes=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
    for row,profile in enumerate(('zero','yaw_bias','random_walk')):
        directory=BASE/profile;name='stage2_localized_'+profile+'_01'
        data=load_jsonl(directory/(name+'.native.jsonl'))
        start=read(directory/(name+'.summary.json'))['goal_delivery']['publish_monotonic_ns']
        by_topic={}
        for r in data:
            if r['receive_monotonic_ns']>=start and r['message_stamp_s'] is not None:
                by_topic.setdefault(r['topic'],{})[round(r['message_stamp_s'],8)]=r['data']
        gt=by_topic['/gazebo/odometry'];raw=by_topic['/simulation/navigation_odometry'];loc=by_topic['/localization/navigation_odometry']
        keys=sorted(set(gt)&set(raw)&set(loc))
        poses=[]
        for series in (gt,raw,loc):
            poses.append(np.array([[series[k]['pose']['pose']['position']['x'],series[k]['pose']['pose']['position']['y']] for k in keys]))
        for points,label,color in zip(poses,('GT','raw drift','scan corrected'),('black','tab:red','tab:blue')):
            axes[row,0].plot(points[:,0],points[:,1],label=label,color=color,linewidth=1)
        axes[row,0].set(title=profile,xlabel='world x (m)',ylabel='world y (m)');axes[row,0].axis('equal');axes[row,0].legend()
        t=np.array(keys)-keys[0]
        axes[row,1].plot(t,np.linalg.norm(poses[1]-poses[0],axis=1)*1000,label='raw drift')
        axes[row,1].plot(t,np.linalg.norm(poses[2]-poses[0],axis=1)*1000,label='scan corrected')
        axes[row,1].set(title=profile+' pose error',xlabel='task simulation time (s)',ylabel='position error (mm)');axes[row,1].legend();axes[row,1].grid(alpha=.25)
    target=BASE/'localization_comparison.png'
    if target.exists():raise FileExistsError(target)
    fig.savefig(target,dpi=160)
    print(target)


if __name__=='__main__':plot()
