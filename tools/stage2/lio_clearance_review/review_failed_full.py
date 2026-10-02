#!/usr/bin/env python3
"""Read-only partial-Full geometry, LIO and guard timeline; no ROS publications."""
import argparse,collections,hashlib,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from analyze_provincial_forward_clearance import walls_from_sdf,nearest_plan,body_world
from provincial_safety_geometry import rectangle,polygon_distance

def load(p):return json.loads(p.read_text())
def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def read(p):return [json.loads(l) for l in p.open()]
def pose(row):
    p=row['data']['pose']['pose'];return [row['message_stamp_s'],p['position']['x'],p['position']['y'],yaw(p['orientation'])]

def review(directory,out):
    out.mkdir(exist_ok=False);name='lio_full_01';rows=read(directory/(name+'.native.jsonl'))
    topics=collections.defaultdict(list)
    for row in rows:topics[row['topic']].append(row)
    gt=np.asarray([pose(r) for r in topics['/gazebo/odometry']]);nav=np.asarray([pose(r) for r in topics['/localization/lio_navigation_odometry']])
    walls=walls_from_sdf(directory/'input_snapshot/src/uav_bringup/worlds/provincial_stage2_lio.sdf')
    goal_time=topics['/goal_pose'][0]['receive_sim_s']
    active=gt[gt[:,0]>=goal_time];gaps=[]
    for g in active:
        body=rectangle(g[1],g[2],g[3],.52,.42)
        gap,wall=min((polygon_distance(body,w['polygon']),w['name']) for w in walls)
        gaps.append([*g,gap,wall])
    minimum=min(gaps,key=lambda g:g[4]);t,x,y,a,gap,wall_name=minimum
    wall=next(w for w in walls if w['name']==wall_name)
    plans=read(directory/(name+'.plans.jsonl'));plan,point=nearest_plan(plans,t,x)
    nearest_nav=nav[np.argmin(abs(nav[:,0]-t))]
    assert abs(nearest_nav[0]-t)<=.025
    nav_body=rectangle(nearest_nav[1],nearest_nav[2],nearest_nav[3],.52,.42)
    nav_gap=min(polygon_distance(nav_body,w['polygon']) for w in walls)
    diag=[r for r in topics['/ground/planning/actuation_diagnostics'] if r['receive_sim_s']<=t]
    diagnostic=diag[-1];d=json.loads(diagnostic['data']['data'])
    setpoint=min(topics['/ground/planning/commanded_setpoint'],key=lambda r:abs(r['message_stamp_s']-t))
    cmd=min(topics['/model/omni_robot/cmd_vel'],key=lambda r:abs(r['receive_sim_s']-t))
    velocity=cmd['data']['linear'];world_velocity=body_world(velocity['x'],velocity['y'],a)
    pairs=[]
    for n in nav[nav[:,0]>=goal_time]:
        g=gt[np.argmin(abs(gt[:,0]-n[0]))]
        if abs(g[0]-n[0])<=.025:pairs.append([n[0],n[1]-g[1],n[2]-g[2],math.hypot(n[1]-g[1],n[2]-g[2])])
    pairs=np.asarray(pairs)
    # Reproduce the original source auditor's output-only window. Report scope,
    # never alter its FAIL or fill unpublished startup history.
    history=collections.deque();velocity_bad=[];diagnostics={}
    for row in topics['/lio/adapter_diagnostics']:
        event=json.loads(row['data']['data'])
        if event.get('event')=='accepted':diagnostics[round(event['stamp_s'],8)]=event
    for row,n in zip(topics['/localization/lio_navigation_odometry'],nav):
        history.append(n)
        while history and n[0]-history[0][0]>.2+1e-9:history.popleft()
        if n[0]-nav[0,0]>=.21 and len(history)>3:
            h=np.asarray(history);dt=h[:,0]-h[:,0].mean();den=dt@dt
            vx=dt@h[:,1]/den;vy=dt@h[:,2]/den;wz=dt@np.unwrap(h[:,3])/den
            v=row['data']['twist']['twist'];c,s=math.cos(n[3]),math.sin(n[3])
            residual=max(abs(c*vx+s*vy-v['linear']['x']),abs(-s*vx+c*vy-v['linear']['y']),abs(wz-v['angular']['z']))
            if residual>1e-7:velocity_bad.append({'stamp_s':float(n[0]),'residual':float(residual),
                'output_only_window_samples':len(history),'actual_adapter_diagnostic':diagnostics.get(round(n[0],8))})
    timeline=[]
    for g in gt[(gt[:,0]>=t-2)&(gt[:,0]<=t+2)]:
        n=nav[np.argmin(abs(nav[:,0]-g[0]))]
        accepted=[p for p in plans if p['marker_stamp_s']<=g[0]]
        pp=min(accepted[-1]['points'],key=lambda p:abs(p[0]-g[1])) if accepted else None
        timeline.append({'sim_s':float(g[0]),'GT':g[1:].tolist(),'nearest_LIO':n.tolist(),
            'association_delta_s':float(n[0]-g[0]),'GT_reference_cte_m':float(g[2]-1.8),
            'LIO_reference_cte_m':float(n[2]-1.8),'nearest_latest_plan_point':pp,
            'GT_body_wall_gap_m':min(polygon_distance(rectangle(g[1],g[2],g[3],.52,.42),w['polygon']) for w in walls)})
    old_dir=ROOT/'tools/results/stage2_step4_lio_20261001/full_01'
    old_gt=np.asarray([pose(r) for r in read(old_dir/'lio_full_01.native.jsonl') if r['topic']=='/gazebo/odometry'])
    old_band=old_gt[(old_gt[:,1]>=5)&(old_gt[:,1]<=x)&(old_gt[:,2]>=1.7)]
    result={'strict_acceptance_pass':False,'scope':'One interrupted Full, raw samples only; no synthetic post-stop evidence',
        'goal_receive_sim_s':goal_time,'minimum':dict(zip(['sim_s','x','y','yaw_rad','gap_m','nearest_wall'],minimum)),
        'threshold_m':.08,'threshold_shortfall_m':.08-gap,'wall':wall,'latest_plan_stamp_s':plan['marker_stamp_s'],
        'plan_point_near_same_x':point,'plan_reference_cte_m':point[1]-1.8,'GT_reference_cte_m':y-1.8,
        'GT_minus_plan_y_m':y-point[1],'nearest_LIO_pose':nearest_nav.tolist(),'LIO_minus_GT_y_m':nearest_nav[2]-y,
        'inferred_nearest_LIO_body_gap_m':nav_gap,'last_guard_diagnostic':d,
        'guard_receive_sim_s':diagnostic['receive_sim_s'],'guard_age_to_minimum_s':t-diagnostic['receive_sim_s'],
        'nearest_setpoint':setpoint,'nearest_final_Twist':cmd,'final_Twist_world_xy_using_GT_yaw':world_velocity,
        'task_max_LIO_GT_xy_m':float(np.max(pairs[:,3])),'task_end_LIO_GT_xy_m':float(pairs[-1,3]),
        'GT_sample_max_header_gap_s':float(np.max(np.diff(active[:,0]))),'GT_active_count':len(active),
        'velocity_auditor_output_only_mismatches':velocity_bad,'velocity_mismatches_after_goal':[r for r in velocity_bad if r['stamp_s']>=goal_time],
        'post_minimum_2s_coverage':False,'raw_record_end_sim_s':float(gt[-1,0]),
        'old_Full_common_space_band_count':len(old_band),'old_Full_common_space_band_mean_y':float(np.mean(old_band[:,2])),
        'source_audit_FAIL_preserved':True,'contact':'CONTACT DATA UNAVAILABLE',
        'limitations':['Nearest planned point at similar x is spatial diagnostic, not time-aligned tracking proof.',
            'LIO and GT use nearest raw stamp within25ms, no interpolated GT.',
            'Yaw for planned rectangle clearance is assumed fixed, local plan lacks a measured chassis yaw.',
            'Startup velocity-history mismatch is reported, not waived; unpublished adapter initialization history absent from output-only recomputation.',
            'Old and new runs differ in localization model and entering state; common-space comparison alone cannot establish causality.',
            'No post-minimum2s data after safety abort, no terminal evidence, contact or continuous-time guarantee.']}
    with (out/'review.json').open('x') as f:json.dump(result,f,indent=2)
    with (out/'minimum_window_raw_timeline.json').open('x') as f:json.dump(timeline,f,indent=2)
    fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
    for ax in axes[0]:
        for w in walls:ax.add_patch(Polygon(w['polygon'],color='gray',alpha=.5))
        ax.plot(gt[:,1],gt[:,2],label='GT raw');ax.plot(nav[:,1],nav[:,2],label='LIO raw')
        ax.plot([4.7,4.7,8.7],[.5,1.8,1.8],'k--',label='reference')
        pp=np.asarray(plan['points']);ax.plot(pp[:,0],pp[:,1],label='latest plan',alpha=.8)
        ax.add_patch(Polygon(rectangle(x,y,a,.52,.42),fill=False,color='red'));ax.scatter([x],[y],c='red');ax.legend(fontsize=8);ax.set_aspect('equal')
    axes[0,0].set_xlim(4.35,6);axes[0,0].set_ylim(.25,2.4);axes[0,0].set_title('Partial Full stopped at guard threshold')
    axes[0,1].set_xlim(x-.4,x+.6);axes[0,1].set_ylim(1.6,2.3);axes[0,1].set_title(f'{wall_name}: GT sample gap {gap:.8f}m')
    tl=np.array([[r['sim_s'],r['GT_reference_cte_m'],r['LIO_reference_cte_m'],r['GT_body_wall_gap_m']] for r in timeline])
    axes[1,0].plot(tl[:,0],tl[:,1]*100,label='GT-reference');axes[1,0].plot(tl[:,0],tl[:,2]*100,label='LIO-reference');axes[1,0].set_ylabel('cm');axes[1,0].set_xlabel('sim s');axes[1,0].legend();axes[1,0].grid()
    axes[1,1].plot(tl[:,0],tl[:,3],label='GT sampled gap');axes[1,1].axhline(.08,c='red',ls='--',label='0.08m gate');axes[1,1].set_ylabel('m');axes[1,1].set_xlabel('sim s');axes[1,1].legend();axes[1,1].grid()
    fig.savefig(out/'failed_full_clearance.png',dpi=140);plt.close(fig)
    print(json.dumps({k:result[k] for k in ['minimum','plan_reference_cte_m','GT_minus_plan_y_m','LIO_minus_GT_y_m','inferred_nearest_LIO_body_gap_m','task_max_LIO_GT_xy_m','velocity_mismatches_after_goal']},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();review(a.result_dir,a.output)
