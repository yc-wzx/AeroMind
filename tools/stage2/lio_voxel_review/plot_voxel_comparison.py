"""Raw nearest-stamp offline error curves; no interpolation or ROS transport."""
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/stage2/lio_optimization'))
from analyze_variance import aligned, errors
OUT=ROOT/'tools/results/stage2_step4_lio_voxel_review_20261002'


def main():
    image=OUT/'voxel_replay_comparison.png'
    if image.exists():raise FileExistsError(image)
    fig,axes=plt.subplots(1,2,figsize=(13,4.8),layout='constrained')
    for ax,tag,source in zip(axes,['old','new'],[
        ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002/full_reserved_map_01',
        ROOT/'tools/results/stage2_step4_lio_optimization_20261002/full_instant_cloud_01']):
        truth=[];begin=None
        with (source/'lio_full_01.native.jsonl').open() as stream:
            for line in stream:
                row=json.loads(line)
                if row['topic']=='/gazebo/odometry':truth.append(aligned(row,False))
                if row['topic']=='/goal_pose':begin=row['receive_sim_s']
        gt=np.array(truth)
        for suffix,label in [('010','10cm LIO voxel'),('005','5cm LIO voxel')]:
            poses=[];last=-float('inf')
            with (OUT/(tag+'_full_voxel_'+suffix)/'output.native.jsonl').open() as stream:
                for line in stream:
                    row=json.loads(line)
                    if row['topic']=='/offline_lio/odometry' and row['message_stamp_s']>last:
                        poses.append(aligned(row,True));last=row['message_stamp_s']
            pair=errors(np.array(poses),gt);pair=pair[pair[:,0]>=begin]
            ax.plot(pair[:,0]-begin,pair[:,1]*100,label=label,linewidth=1)
        ax.set_xlabel('Task elapsed simulation time (s)');ax.set_ylabel('XY residual to raw GT (cm)')
        ax.set_ylim(0,6);ax.grid(alpha=.3);ax.legend()
        ax.set_title('Recording '+('1' if tag=='old' else '2')+' | identical sensor bytes')
    fig.suptitle('Offline comparison: nearest raw GT within 25ms; no interpolated truth')
    fig.savefig(image,dpi=140);plt.close(fig)
    print(image)


if __name__=='__main__':main()
