#!/usr/bin/env python3
"""Scientific plots of original native GT; no interpolation or ROS publishing."""
import json,math,sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from summarize_provincial_stage15_completion import BASE,ROOT,FINAL
from analyze_provincial_forward_clearance import walls_from_sdf
from provincial_safety_geometry import body_wall_gap,rectangle
out=BASE/'final_plots'
if out.exists():raise FileExistsError(out)
out.mkdir();walls=walls_from_sdf(BASE/FINAL[0]/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf');polygons=[w['polygon'] for w in walls];traces=[]
for i,name in enumerate(FINAL):
 for side in ('forward','return'):
  leg=name+'_'+side;p=BASE/name/leg;summary=json.loads((p/(leg+'.summary.json')).read_text());end=summary['terminal_snapshot']['gt_stamp_s'];start=end-summary['duration_sim_s'];trace=[]
  for line in (p/(leg+'.native.jsonl')).open():
   if '/gazebo/odometry' not in line:continue
   row=json.loads(line)
   if row['topic']!='/gazebo/odometry' or not start<=row['message_stamp_s']<=end:continue
   d=row['data']['pose']['pose'];xy=d['position'];q=d['orientation'];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2));trace.append((row['message_stamp_s']-start,xy['x'],xy['y'],yaw,body_wall_gap(xy['x'],xy['y'],yaw,polygons)))
  traces.append((name,side,trace))
fig,axes=plt.subplots(1,2,figsize=(13,5.5),constrained_layout=True)
for wall in walls:axes[0].add_patch(Polygon(wall['polygon'],facecolor='#30343c',edgecolor='black'))
ref=json.loads((ROOT/'src/uav_planning/config/provincial_2025_provisional.json').read_text())['reference_route'];axes[0].plot([p[0] for p in ref],[p[1] for p in ref],':',c='gray',label='Configured reference')
colors=['#1565c0','#1b9e77','#d95f02'];markers=[]
for idx,(name,side,trace) in enumerate(traces):
 c=colors[FINAL.index(name)];label=name[-2:]+' '+side
 axes[0].plot([x[1] for x in trace],[x[2] for x in trace],'-' if side=='forward' else '--',lw=1.4,c=c,label=label)
 lowest=min(trace,key=lambda x:x[4]);axes[0].add_patch(Polygon(rectangle(lowest[1],lowest[2],lowest[3],.52,.42),fill=False,edgecolor=c,lw=.8))
 axes[0].scatter([lowest[1]],[lowest[2]],s=20,c=c);markers.append((label,lowest[4]))
 axes[1].plot([x[0] for x in trace],[x[4] for x in trace],'-' if side=='forward' else '--',lw=1.2,c=c,label=label)
 axes[1].scatter([lowest[0]],[lowest[4]],s=20,c=c)
axes[0].set_aspect('equal');axes[0].set_xlim(4.15,9.25);axes[0].set_ylim(0,4.8);axes[0].set(xlabel='World x (m)',ylabel='World y (m)',title='Final version: three Full / Return sessions');axes[0].legend(fontsize=8,ncol=2)
axes[1].axhline(.08,c='red',ls=':',label='0.08 m sampled gate');axes[1].set(xlabel='Elapsed simulation time since goal (s)',ylabel='Original-sample body / wall gap (m)',title='Native GT geometry, approximately 50 Hz');axes[1].legend(fontsize=8,ncol=2)
fig.savefig(out/'forward_return_native_gt.png',dpi=180);plt.close(fig)
(out/'plot_metadata.json').write_text(json.dumps({'original_samples_only':True,'interpolation_used':False,'contact_detection':False,'continuous_safety_proven':False,'trials':list(FINAL),'minimum_gaps_m':dict(markers)},indent=2))
print(out/'forward_return_native_gt.png')
