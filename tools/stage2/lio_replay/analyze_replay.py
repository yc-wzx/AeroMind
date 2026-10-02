"""Offline pose/time/point geometry diagnosis. GT is never a replay input."""
import argparse,base64,hashlib,json,math,sys
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from analyze_provincial_forward_clearance import walls_from_sdf

def load(p):return json.loads(p.read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def xyz(m):
    fields={f.name:f for f in m.fields}
    a=np.stack([np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=bytes(m.data),offset=fields[k].offset,strides=(m.row_step,m.point_step)).reshape(-1) for k in 'xyz'],axis=1)
    return a[np.isfinite(a).all(axis=1)]
def pose(row,aligned=False):
    p=row['data']['pose']['pose'];v=p['position'];q=p['orientation'];r=Rotation.from_quat([q[k] for k in 'xyzw']);position=np.array([v[k] for k in 'xyz'])
    if not aligned:
        position=Rotation.from_euler('z',math.pi/2).apply(position-r.apply([0,0,.25]))+np.array([4.7,.5,.4]);r=Rotation.from_euler('z',math.pi/2)*r
    yaw,pitch,roll=r.as_euler('ZYX')
    return [row['message_stamp_s'],*position,yaw,pitch,roll]

def monotonic(rows,aligned=False):
    selected=[];last=-float('inf');skipped=0
    for row in rows:
        t=row['message_stamp_s']
        if t>last:selected.append(pose(row,aligned));last=t
        else:skipped+=1
    return np.asarray(selected),skipped
def associate(poses,gt):
    rows=[]
    for p in poses:
        i=np.searchsorted(gt[:,0],p[0]);indices=[j for j in [i-1,i] if 0<=j<len(gt)];j=min(indices,key=lambda k:abs(gt[k,0]-p[0]))
        if abs(gt[j,0]-p[0])<=.025:
            g=gt[j];rows.append([p[0],p[1]-g[1],p[2]-g[2],math.hypot(p[1]-g[1],p[2]-g[2]),p[3]-.15,p[5],p[6]])
    return np.asarray(rows)

def analyze(source,replay,out):
    out.mkdir(parents=True,exist_ok=False);groups=defaultdict(list);physical=[]
    for line in (source/'lio_full_01.native.jsonl').open():
        row=json.loads(line)
        if row['topic'] in ['/lio/odometry','/gazebo/odometry','/localization/lio_navigation_odometry','/goal_pose']:groups[row['topic']].append(row)
        if row['topic']=='/simulation/imu_kinematics':physical.append(json.loads(row['data']['data']))
    replay_rows=[json.loads(l) for l in (replay/'output.native.jsonl').open() if '"/offline_lio/odometry"' in l]
    gt=np.asarray([pose(r,True) for r in groups['/gazebo/odometry']]);original,_=monotonic(groups['/localization/lio_navigation_odometry'],True);new,skipped=monotonic(replay_rows)
    errors=associate(original,gt);new_errors=associate(new,gt);summary=load(source/'lio_full_01.summary.json')
    start=groups['/goal_pose'][0]['receive_sim_s'];end=summary['terminal_snapshot']['gt_stamp_s'];completion=summary['navigation_completion']['gt_stamp_s']
    def statistics(a):
        b=a[(a[:,0]>=start)&(a[:,0]<=end)]
        if not len(b):return {'pairs':0,'status':'NO_TASK_WINDOW_COVERAGE'}
        return {'pairs':len(b),'max_xy_error_m':float(max(b[:,3])),
            'at_completion':b[np.argmin(abs(b[:,0]-completion))].tolist(),
            'at_terminal':b[np.argmin(abs(b[:,0]-end))].tolist(),
            'max_height_deviation_from_declared_flat_base_m':float(max(abs(b[:,4]))),
            'max_pitch_deg':float(np.rad2deg(max(abs(b[:,5])))),'max_roll_deg':float(np.rad2deg(max(abs(b[:,6]))))}
    # Inspect raw instantaneous clouds at 1Hz: classify proximity to known
    # horizontal/vertical visual surfaces. This is NOT the internal LIO H matrix.
    wall=walls_from_sdf(source/'input_snapshot/src/uav_bringup/worlds/provincial_stage2_lio.sdf');scene=[];time_spans=[];raw_cloud={};registered={}
    for line in (source/'lio_full_01.sensors.jsonl').open():
        row=json.loads(line);t=row['message_stamp_s'];key=round(t,8)
        if row['topic'] not in ['/lio/lidar','/lio/cloud_registered']:continue
        if int(round((t-.008)*10))%10:continue
        m=deserialize_message(base64.b64decode(row['cdr']),PointCloud2);a=xyz(m)
        if row['topic']=='/lio/cloud_registered':registered[key]=a;continue
        raw_cloud[key]=a
        timestamp=next(f for f in m.fields if f.name=='timestamp')
        values=np.ndarray((m.height,m.width),dtype='<f8',buffer=bytes(m.data),offset=timestamp.offset,strides=(m.row_step,m.point_step)).reshape(-1)
        time_spans.append(float(np.ptp(values)))
        index=np.argmin(abs(gt[:,0]-t));g=gt[index];world=Rotation.from_euler('z',g[4]).apply(a)+[g[1],g[2],.4]
        floor=np.minimum(abs(world[:,2]),abs(world[:,2]-.004))<.025
        vertical=np.zeros(len(a),dtype=bool)
        caps=np.zeros(len(a),dtype=bool)
        for w in wall:
            cx,cy,cz,_,_,angle=w['pose'];sx,sy,sz=w['size']
            local=Rotation.from_euler('z',-angle).apply(world-[cx,cy,cz]);x,y,z=local.T
            inx=abs(x)<=sx/2+.025;iny=abs(y)<=sy/2+.025;inz=abs(z)<=sz/2+.025
            face=((abs(abs(x)-sx/2)<.025)&iny&inz)|((abs(abs(y)-sy/2)<.025)&inx&inz)
            vertical|=face;caps|=(abs(z-sz/2)<.025)&inx&iny
        scene.append({'sim_s':t,'points':len(a),'floor_fraction':float(floor.mean()),'wall_top_fraction':float(caps.mean()),
            'wall_side_fraction':float(vertical.mean()),'other_fraction':float((~(floor|caps|vertical)).mean()),
            'model':'Flat sensor at physicalz=.4 from SDF; GT2D yaw; visual surfaces approximate, tolerance .025m.'})
    # Distinguish corrected scan states from high-rate predictions by matching
    # each published dense cloud to the raw input transformed by same-stamp
    # candidate poses. No dependence on GT in this correspondence calculation.
    candidates=defaultdict(list)
    for r in groups['/lio/odometry']:candidates[round(r['message_stamp_s'],8)].append(r)
    corrected=[]
    for t,points in raw_cloud.items():
        if t not in registered or t not in candidates:continue
        tree=cKDTree(registered[t]);distances=[];percentiles=[]
        for r in candidates[t]:
            p=r['data']['pose']['pose'];q=p['orientation'];rotation=Rotation.from_quat([q[k] for k in 'xyzw']);v=p['position']
            transformed=rotation.apply(points)+[v[k] for k in 'xyz'];d,_=tree.query(transformed,k=1)
            distances.append(float(np.max(d)))
            percentiles.append([float(np.percentile(d,50)),float(np.percentile(d,95)),int(np.sum(d>2e-5))])
        i=int(np.argmin(distances));r=candidates[t][i]
        corrected.append({'sim_s':t,'candidate_count':len(distances),'best_cloud_residual_m':distances[i],
            'other_candidate_residuals_m':distances,'corrected_pose':pose(r),'cloud_pose_match_pass':distances[i]<2e-5})
        corrected[-1]['candidate_median_p95_exceeding_2e5']=percentiles
    valid_corrected=[r['corrected_pose'] for r in corrected if r['cloud_pose_match_pass']]
    corrected_error=associate(np.asarray(valid_corrected),gt) if valid_corrected else np.empty((0,7))
    source_inputs=load(replay/'input_manifest.json');progress=load(replay/'progress.json');native_health=load(replay/'output.native.health.json');cloud_health=load(replay/'output.clouds.health.json')
    integrity={'replay_finished':progress['finished'],'raw_source_unchanged':progress['input_content_unchanged'],
        'all_expected_imu_sent':progress['sent']['imu']==22688,'all_expected_clouds_sent':progress['sent']['cloud']==908,
        'all_cloud_corrections_observed':cloud_health['received_counts'].get('/offline_lio/cloud_registered')==905,
        'native_writer_healthy':native_health['pass'],'cloud_writer_healthy':cloud_health['pass'],
        'data_hashes_match':all(sha(replay/p)==h for p,h in load(replay/'data_sha256.json').items()),
        'sensor_content_unchanged_no_GT_feedback':not source_inputs['sensor_content_changed'] and not source_inputs['GT_and_navigation_input_replayed'],
        'only_declared_parameter_overrides':source_inputs.get('parameter_overrides',{}) in [{},{'common.planar_mode':True,'common.planar_height':0.0},
            {'mapping.acc_cov':1e-6,'mapping.gyr_cov':1e-8,'mapping.b_acc_cov':0.0,'mapping.b_gyr_cov':0.0}]}
    result={'integrity':integrity,'integrity_pass':all(integrity.values()),'original':statistics(errors),'replay':statistics(new_errors),
        'parameter_overrides':source_inputs.get('parameter_overrides',{}),
        'replay_skipped_duplicate_or_older_odometry':skipped,'error_columns':['sim_s','LIO_GT_x_m','LIO_GT_y_m','xy_error_m','base_height_deviation_m','pitch_rad','roll_rad'],
        'corrected_cloud_pose_matches':corrected,'corrected_cloud_matches_pass':all(r['cloud_pose_match_pass'] for r in corrected),
        'corrected_LIO_task_error':statistics(corrected_error) if len(corrected_error) else None,
        'sampled_cloud_geometry':scene,'max_point_timestamp_span_ns':max(time_spans),
        'physical_max_vertical_velocity_mps':max(abs(r['world_velocity'][2]) for r in physical),
        'physical_max_roll_pitch_angular_speed_rad_s':max(max(abs(v) for v in r['body_angular_velocity'][:2]) for r in physical),
        'original_raw_hashes':source_inputs['raw_input_sha256'],'tool_sha256':sha(Path(__file__)),
        'limitations':['Half-speed replay changes callback scheduling; not a bit-exact reproduction.','Geometry surface counts do not measure internal LIO correspondence Hessian.','Known flat height comparison is a model-based inference, not recorded3D GT.','Offline replay never demonstrates navigation or contact success.']}
    fig,ax=plt.subplots(3,1,figsize=(11,10),sharex=True)
    ax[0].plot(errors[:,0],errors[:,3]*100,label='original navigation LIO');ax[0].plot(new_errors[:,0],new_errors[:,3]*100,label='replayed same sensor content',alpha=.7)
    if len(corrected_error):ax[0].scatter(corrected_error[:,0],corrected_error[:,3]*100,s=12,label='original corrected scan states (~1Hz)')
    ax[0].set_ylabel('xy error cm');ax[0].legend()
    ax[1].plot(errors[:,0],errors[:,4]*100,label='original');ax[1].plot(new_errors[:,0],new_errors[:,4]*100,label='replay');ax[1].set_ylabel('height vs declared flat base cm');ax[1].legend()
    ax[2].plot([r['sim_s'] for r in scene],[100*r['floor_fraction'] for r in scene],label='floor-like returns');ax[2].plot([r['sim_s'] for r in scene],[100*r['wall_top_fraction'] for r in scene],label='wall-top returns');ax[2].plot([r['sim_s'] for r in scene],[100*r['wall_side_fraction'] for r in scene],label='wall-side returns');ax[2].set_ylabel('sampled surface proximity %');ax[2].set_xlabel('original sim s');ax[2].legend()
    for a in ax:a.axvline(start,color='grey',ls='--');a.axvline(completion,color='red',ls='--');a.grid()
    fig.tight_layout();fig.savefig(out/'replay_comparison.png',dpi=140);plt.close(fig)
    with (out/'analysis.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:result[k] for k in ['integrity_pass','original','replay','corrected_cloud_matches_pass','corrected_LIO_task_error','physical_max_vertical_velocity_mps','physical_max_roll_pitch_angular_speed_rad_s']},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--replay',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();analyze(a.source,a.replay,a.output)
