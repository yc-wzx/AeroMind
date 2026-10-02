#!/usr/bin/env python3
"""Read-only simultaneous pose/geometry analysis; no command publication."""
import bisect
import csv
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from audit_localized import ROOT, load_jsonl, read, sha, yaw
from analyze_provincial_forward_clearance import walls_from_sdf
from provincial_safety_geometry import body_wall_gap, rectangle, polygon_distance

BASE=ROOT/'tools/results/stage2_localization_20261001'
OUT=ROOT/'tools/results/stage2_scan_guard_20261001'


def pose(row):
    p=row['data']['pose']['pose']
    return p['position']['x'],p['position']['y'],yaw(p['orientation'])


def support_y(angle):
    return .26*abs(math.sin(angle))+.21*abs(math.cos(angle))


def world_velocity(row, angle):
    v=row['data']['linear'];c,s=math.cos(angle),math.sin(angle)
    return c*v['x']-s*v['y'],s*v['x']+c*v['y']


def analyze(profile):
    directory=BASE/profile;name='stage2_localized_'+profile+'_01'
    native=directory/(name+'.native.jsonl');allrows=load_jsonl(native)
    topic={}
    for row in allrows:topic.setdefault(row['topic'],[]).append(row)
    indexed={k:{round(r['message_stamp_s'],8):r for r in v} for k,v in topic.items()
             if k in ('/gazebo/odometry','/simulation/navigation_odometry','/localization/navigation_odometry')}
    cached={k:sorted(v,key=lambda r:r['receive_monotonic_ns']) for k,v in topic.items()}
    times={k:[r['receive_monotonic_ns'] for r in v] for k,v in cached.items()}
    def previous(key, gt, limit):
        i=bisect.bisect_right(times.get(key,[]),gt['receive_monotonic_ns'])-1
        if i<0:return None,None
        r=cached[key][i];age=(gt['receive_monotonic_ns']-r['receive_monotonic_ns'])*1e-9
        return (r,age) if age<=limit else (None,age)
    audit=read(directory/'independent_audit_v3.json');minimum=audit['native']['minimum'];tmin=minimum['sim_t']
    walls=walls_from_sdf(directory/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf')
    boxes=[w['polygon'] for w in walls];wall=next(w for w in walls if w['name']==minimum['nearest_wall'])
    boundary=min(p[1] for p in wall['polygon']);points=[]
    for gt in topic['/gazebo/odometry']:
        t=gt['message_stamp_s']
        if not tmin-2.-1e-9<=t<=tmin+2.+1e-9:continue
        key=round(t,8);raw=indexed['/simulation/navigation_odometry'].get(key);loc=indexed['/localization/navigation_odometry'].get(key)
        if raw is None or loc is None:raise RuntimeError('Missing same-stamp pose in narrow interval')
        g,a,b=pose(gt),pose(raw),pose(loc)
        gt_rect=rectangle(*g,.52,.42);loc_rect=rectangle(*b,.52,.42)
        gap=body_wall_gap(*g,boxes);lgap=body_wall_gap(*b,boxes)
        normal=b[1]-g[1];angular_support=support_y(b[2])-support_y(g[2])
        physical_wall_gap=polygon_distance(gt_rect,wall['polygon']);estimated_wall_gap=polygon_distance(loc_rect,wall['polygon'])
        # This plane decomposition is only exact away from wall endpoints.
        bx=[p[0] for p in wall['polygon']]
        plane_valid=all(min(bx)<=p[0]<=max(bx) for p in gt_rect+loc_rect) and max(p[1] for p in gt_rect+loc_rect)<boundary
        row={'sim_t':t,'relative_to_min_s':t-tmin,'gt_x':g[0],'gt_y':g[1],'gt_yaw':g[2],
             'raw_x':a[0],'raw_y':a[1],'raw_yaw':a[2],'localized_x':b[0],'localized_y':b[1],'localized_yaw':b[2],
             'gt_gap_m':gap,'localized_gap_m':lgap,'wall_gt_gap_m':physical_wall_gap,'wall_localized_gap_m':estimated_wall_gap,
             'normal_position_error_m':normal,'yaw_error_rad':math.atan2(math.sin(b[2]-g[2]),math.cos(b[2]-g[2])),
             'yaw_support_error_m':angular_support,'gap_overestimate_m':estimated_wall_gap-physical_wall_gap,
             'plane_decomposition_valid':plane_valid,'decomposition_residual_m':estimated_wall_gap-physical_wall_gap+normal+angular_support,
             'same_stamp_position_error_m':math.dist(g[:2],b[:2]),'odom_gt_receive_offset_s':(loc['receive_monotonic_ns']-gt['receive_monotonic_ns'])*1e-9}
        for key,label in (('/cmd_vel','raw_cmd'),('/model/omni_robot/cmd_vel','final_cmd')):
            command,age=previous(key,gt,.2);row[label+'_receive_age_s']=age
            if command is not None:
                v=command['data'];vx,vy=world_velocity(command,g[2])
                row.update({label+'_body_vx':v['linear']['x'],label+'_body_vy':v['linear']['y'],
                            label+'_wz':v['angular']['z'],label+'_world_vx':vx,label+'_world_vy':vy})
        act,age=previous('/ground/planning/actuation_diagnostics',gt,.2);row['guard_receive_age_s']=age
        if act is not None:
            d=json.loads(act['data']['data'])
            row.update({k:d.get(k) for k in ('action','source','waypoint','predicted_gap_m','planned_path_min_gap_m')})
            row['guard_receive_sim_s']=act['receive_sim_s']
        setpoint,age=previous('/ground/planning/commanded_setpoint',gt,.2);row['setpoint_receive_age_s']=age
        if setpoint is not None:
            row['setpoint_x']=setpoint['data']['pose']['position']['x'];row['setpoint_y']=setpoint['data']['pose']['position']['y']
        plan,age=previous('/ground/planning/planned_trajectory',gt,3.);row['plan_receive_age_s']=age
        if plan is not None:
            row['plan_marker_stamp_s']=plan['message_stamp_s'];row['plan_marker_id']=plan['data']['id']
            closest=min(plan['data']['points'],key=lambda p:math.hypot(p['x']-g[0],p['y']-g[1]))
            row['nearest_received_plan_x']=closest['x'];row['nearest_received_plan_y']=closest['y']
        spline,age=previous('/planning/bspline',gt,3.);row['spline_receive_age_s']=age
        if spline is not None:
            row['received_trajectory_id']=spline['data'].get('traj_id')
            row['received_trajectory_start']=json.dumps(spline['data'].get('start_time'))
        points.append(row)
    if len(points)<150:raise RuntimeError('Insufficient 4-second raw evidence')
    atmin=min(points,key=lambda p:abs(p['sim_t']-tmin))
    if abs(atmin['gt_gap_m']-minimum['gap_m'])>1e-10:raise RuntimeError('Historical minimum disagrees')
    target=OUT/(profile+'_narrow_timeline_v2.csv')
    if target.exists():raise FileExistsError(target)
    fields=list(dict.fromkeys(k for p in points for k in p))
    with target.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(points)
    intervals=[b['sim_t']-a['sim_t'] for a,b in zip(points,points[1:])]
    result={'profile':profile,'samples':len(points),'interval':[points[0]['sim_t'],points[-1]['sim_t']],
       'max_sample_interval_s':max(intervals),'minimum':atmin,'minimum_gap_above_threshold_m':atmin['gt_gap_m']-.08,
       'maximum_gap_overestimate_in_window_m':max(p['gap_overestimate_m'] for p in points),
       'maximum_absolute_normal_error_m':max(abs(p['normal_position_error_m']) for p in points),
       'maximum_yaw_support_error_m':max(abs(p['yaw_support_error_m']) for p in points),
       'maximum_decomposition_residual_m':max(abs(p['decomposition_residual_m']) for p in points if p['plane_decomposition_valid']),
       'plane_decomposition_all_valid':all(p['plane_decomposition_valid'] for p in points),
       'native_sha256':sha(native),'timeline_sha256':sha(target),'continuous_time_safety_proven':False,
       'limits':['Same-stamp sensor poses are sampled, not a continuous-time bound.',
         'Twist and unstamped diagnostics are associated by earlier recorder reception; not identical generation time.',
         'World velocities use GT yaw for read-only analysis only.',
         'Nearest received plan point is spatial context, not commanded progression along its spline.',
         'Inherited covariance is not calibrated localization posterior risk.']}
    return result,points,wall


def main():
    fig,axes=plt.subplots(3,3,figsize=(15,10),constrained_layout=True);reports={}
    for index,profile in enumerate(('zero','yaw_bias','random_walk')):
        report,rows,wall=analyze(profile);reports[profile]=report
        t=[p['relative_to_min_s'] for p in rows]
        axes[index,0].plot(t,[p['gt_gap_m']*1000 for p in rows],label='GT sampled')
        axes[index,0].plot(t,[p['localized_gap_m']*1000 for p in rows],label='estimated same stamp')
        guard=[p for p in rows if p.get('predicted_gap_m') is not None]
        axes[index,0].plot([p['relative_to_min_s'] for p in guard],[p['predicted_gap_m']*1000 for p in guard],label='guard prediction',alpha=.7)
        axes[index,0].axhline(80,color='red',linestyle='--');axes[index,0].set(title=profile+' wall gap',xlabel='seconds from minimum',ylabel='gap (mm)');axes[index,0].legend(fontsize=8)
        for k,label in (('normal_position_error_m','normal position'),('yaw_support_error_m','yaw footprint'),('gap_overestimate_m','gap overestimate')):
            axes[index,1].plot(t,[p[k]*1000 for p in rows],label=label)
        axes[index,1].axhline(0,color='grey',linewidth=.5);axes[index,1].set(title=profile+' simultaneous errors',xlabel='seconds from minimum',ylabel='error (mm)');axes[index,1].legend(fontsize=8)
        ax=axes[index,2];ax.add_patch(Polygon(wall['polygon'],facecolor='grey',alpha=.4))
        ax.plot([p['gt_x'] for p in rows],[p['gt_y'] for p in rows],label='GT')
        ax.plot([p['localized_x'] for p in rows],[p['localized_y'] for p in rows],label='localized')
        m=report['minimum'];ax.add_patch(Polygon(rectangle(m['gt_x'],m['gt_y'],m['gt_yaw'],.52,.42),fill=False,edgecolor='black'))
        ax.axhline(1.8,color='green',linestyle='--',label='reference y=1.8')
        ax.set_xlim(m['gt_x']-.7,m['gt_x']+.7);ax.set_ylim(1.55,2.28);ax.set_aspect('equal');ax.set(title=profile+' closest footprint',xlabel='world x (m)',ylabel='world y (m)');ax.legend(fontsize=8)
    fig.savefig(OUT/'narrow_clearance_review_v2.png',dpi=150)
    target=OUT/'narrow_clearance_review_v2.json'
    if target.exists():raise FileExistsError(target)
    target.write_text(json.dumps(reports,indent=2)+'\n')
    print(json.dumps({p:{'gap_m':a['minimum']['gt_gap_m'],'normal_mm':a['minimum']['normal_position_error_m']*1000,
       'yaw_support_mm':a['minimum']['yaw_support_error_m']*1000,'gap_overestimate_mm':a['minimum']['gap_overestimate_m']*1000,
       'min_guard_prediction_m':a['minimum'].get('predicted_gap_m'),'window_overestimate_max_mm':a['maximum_gap_overestimate_in_window_m']*1000} for p,a in reports.items()},indent=2))


if __name__=='__main__':main()
