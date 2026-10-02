"""Read-only native pose pairing, final-code static window and diagnostic plot."""
import sys,json,math,argparse,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from analyze_provincial_forward_clearance import walls_from_sdf
from provincial_safety_geometry import rectangle

def load(path):return json.loads(path.read_text())
def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))

def review(directory,phase,output):
    output.mkdir(exist_ok=False);name='lio_'+phase+'_01'
    selected={t:[] for t in ['/gazebo/odometry','/localization/lio_navigation_odometry','/lio/odometry','/goal_pose','/ground/planning/actuation_diagnostics','/rosout/relevant']}
    rates={}
    for row in (directory/(name+'.native.jsonl')).open():
        r=json.loads(row);t=r['topic']
        if t in selected:selected[t].append(r)
        if r['message_stamp_s'] is not None:
            info=rates.setdefault(t,{'count':0,'first':r['message_stamp_s'],'last':None,'max_sim_gap':0.,'max_receive_gap':0.,'duplicates':0,'out_of_order':0})
            if info['last'] is not None:
                dt=r['message_stamp_s']-info['last'];info['max_sim_gap']=max(info['max_sim_gap'],dt)
                info['duplicates']+=dt==0;info['out_of_order']+=dt<0
                info['max_receive_gap']=max(info['max_receive_gap'],(r['receive_monotonic_ns']-info['last_receive_ns'])*1e-9)
            info.update(last=r['message_stamp_s'],last_receive_ns=r['receive_monotonic_ns'],count=info['count']+1)
    for row in (directory/(name+'.sensors.jsonl')).open():
        r=json.loads(row);t=r['topic'];info=rates.setdefault(t,{'count':0,'first':r['message_stamp_s'],'last':None,'max_sim_gap':0.,'max_receive_gap':0.,'duplicates':0,'out_of_order':0})
        if info['last'] is not None:
            dt=r['message_stamp_s']-info['last'];info['max_sim_gap']=max(info['max_sim_gap'],dt)
            info['duplicates']+=dt==0;info['out_of_order']+=dt<0
            info['max_receive_gap']=max(info['max_receive_gap'],(r['receive_monotonic_ns']-info['last_receive_ns'])*1e-9)
        info.update(last=r['message_stamp_s'],last_receive_ns=r['receive_monotonic_ns'],count=info['count']+1)
    for info in rates.values():info['stamp_rate_hz']=(info['count']-1)/(info['last']-info['first']) if info['last']>info['first'] else None
    def poses(rows):
        a=[]
        for r in rows:
            p=r['data']['pose']['pose'];v=r['data']['twist']['twist']
            a.append([r['message_stamp_s'],p['position']['x'],p['position']['y'],yaw(p['orientation']),p['position']['z'],math.hypot(v['linear']['x'],v['linear']['y']),v['angular']['z']])
        return np.asarray(a)
    gt=poses(selected['/gazebo/odometry']);nav=poses(selected['/localization/lio_navigation_odometry'])
    pairs=[]
    for n in nav:
        i=np.searchsorted(gt[:,0],n[0]);indices=[j for j in (i-1,i) if 0<=j<len(gt)]
        k=min(indices,key=lambda j:abs(gt[j,0]-n[0]))
        if abs(gt[k,0]-n[0])<=.025:
            g=gt[k];pairs.append([n[0],n[1]-g[1],n[2]-g[2],math.hypot(n[1]-g[1],n[2]-g[2]),math.atan2(math.sin(n[3]-g[3]),math.cos(n[3]-g[3])),n[4],n[5],n[6]])
    a=np.asarray(pairs);goal_time=selected['/goal_pose'][0]['receive_sim_s']
    summary=load(directory/(name+'.summary.json'))
    completion_time=summary['navigation_completion']['gt_stamp_s']
    active=a[(a[:,0]>=goal_time)&(a[:,0]<=summary['terminal_snapshot']['gt_stamp_s'])]
    completion=active[np.argmin(abs(active[:,0]-completion_time))]
    checkpoints=[]
    for r in selected['/rosout/relevant']:
        message=r['data'].get('msg','')
        if 'RMUC waypoint diagnostic completed' in message:
            time_s=r['receive_sim_s'];b=active[np.argmin(abs(active[:,0]-time_s))]
            checkpoints.append({'message':message,'receive_sim_s':time_s,'nearest_pair':b.tolist()})
    # Fixed 10..15s lies before the external goal in these trials. No selected
    # adaptive quiet interval; all samples and max gaps are used.
    static=a[(a[:,0]>=10)&(a[:,0]<=15)];static_gt=gt[(gt[:,0]>=10)&(gt[:,0]<=15)]
    static_checks={'pre_goal_window':goal_time>15,'GT_covers_5s':len(static_gt)>200 and static_gt[0,0]<=10.02 and static_gt[-1,0]>=14.98,
        'LIO_covers_5s':len(static)>1000 and static[0,0]<=10.02 and static[-1,0]>=14.98,
        'finite':bool(len(static)) and np.isfinite(static).all().item(),
        'xy_error_below_2cm':bool(len(static)) and float(np.max(static[:,3]))<=.02,
        'estimated_speed_below_2cm_s':bool(len(static)) and float(np.max(static[:,6]))<=.02,
        'estimated_yaw_speed_below_003':bool(len(static)) and float(np.max(abs(static[:,7])))<=.03,
        'GT_stopped':bool(len(static_gt)) and float(np.max(static_gt[:,5]))<=.02,
        'static_sample_gap_bounded':len(static)>1 and float(np.max(np.diff(static[:,0])))<=.2}
    static_checks={key:bool(value) for key,value in static_checks.items()}
    audit=load(directory/('independent_audit_v1.json' if phase=='full' else 'independent_audit_v3.json'));minimum=audit['native']['minimum']
    walls=walls_from_sdf(directory/'input_snapshot/src/uav_bringup/worlds/provincial_stage2_lio.sdf')
    fig,axes=plt.subplots(2,2,figsize=(12,9))
    ax=axes[0,0]
    for wall in walls:ax.add_patch(Polygon(wall['polygon'],color='grey',alpha=.6))
    ax.plot(gt[:,1],gt[:,2],label='raw Gazebo GT');ax.plot(nav[:,1],nav[:,2],label='LIO navigation',alpha=.8)
    reference=np.array([[4.7,.5],[4.7,1.8],[8.7,1.8],[8.7,4.25]])
    ax.plot(reference[:,0],reference[:,1],'k--',label='reference');ax.axis('equal');ax.set_xlim(4,9.5);ax.set_ylim(0,5);ax.legend();ax.set_title('Actual poses; no interpolation')
    ax=axes[0,1]
    for wall in walls:ax.add_patch(Polygon(wall['polygon'],color='grey',alpha=.6))
    ax.plot(gt[:,1],gt[:,2]);ax.plot(nav[:,1],nav[:,2]);ax.plot(reference[:,0],reference[:,1],'k--')
    ax.add_patch(Polygon(rectangle(minimum['x'],minimum['y'],minimum['yaw_rad'],.52,.42),fill=False,color='red'))
    ax.scatter([minimum['x']],[minimum['y']],c='red');ax.set_xlim(minimum['x']-.6,minimum['x']+.9);ax.set_ylim(1.25,2.4);ax.set_aspect('equal');ax.set_title(f"Minimum GT sample: {minimum['gap_m']:.5f}m / {minimum['nearest_wall']}")
    ax=axes[1,0];ax.plot(active[:,0]-goal_time,100*active[:,1],label='LIO-GT x');ax.plot(active[:,0]-goal_time,100*active[:,2],label='LIO-GT y');ax.plot(active[:,0]-goal_time,100*active[:,3],label='xy magnitude');ax.set_ylabel('cm');ax.set_xlabel('sim s after goal');ax.legend();ax.grid();ax.set_title('Nearest raw GT within25ms')
    ax=axes[1,1];ax.plot(active[:,0]-goal_time,np.rad2deg(active[:,4]),label='yaw difference (deg)');ax.plot(active[:,0]-goal_time,100*(active[:,5]-.15),label='LIO base z offset (cm)');ax.legend();ax.grid();ax.set_xlabel('sim s after goal');ax.set_title('GT plugin is2D: z offset is not 3D GT error')
    fig.tight_layout();fig.savefig(output/'lio_forward_review.png',dpi=140);plt.close(fig)
    result={'phase':phase,'independent_audit_pass':audit['all_pass'],'rates':rates,'task_max_xy_error_m':float(np.max(active[:,3])),
        'task_max_error_pair':active[np.argmax(active[:,3])].tolist(),'completion_nearest_pose_pair':completion.tolist(),
        'pair_columns':['sim_s','LIO_minus_GT_x','LIO_minus_GT_y','xy_error_m','yaw_error_rad','LIO_base_z_m','LIO_body_speed','LIO_yaw_speed'],
        'waypoint_pairs':checkpoints,'minimum_native_sample_gap':minimum,
        'latest_code_static_10_to_15s':{'pass':all(static_checks.values()),'checks':static_checks,'samples':len(static),
            'max_xy_error_m':float(np.max(static[:,3])),'max_estimated_speed_mps':float(np.max(static[:,6]))},
        'evidence_files':str(directory),'review_tool_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'causality':'The terminal LIO/GT offset is measured; specific cause of LIO drift is not independently identified.',
        'no_continuous_time_clearance_proof':True,'CONTACT DATA':'CONTACT DATA UNAVAILABLE'}
    with (output/'review.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:result[k] for k in ['phase','independent_audit_pass','task_max_xy_error_m','completion_nearest_pose_pair','latest_code_static_10_to_15s','minimum_native_sample_gap']},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--phase',choices=['short','full'],required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();review(a.result_dir,a.phase,a.output)
