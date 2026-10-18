"""Read-only gimbal turn/return analysis. No motion commands or angle truth."""
import argparse,base64,json,math,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;PHASE=HERE.parent/'mid360_lio_phased'
sys.path.insert(0,str(PHASE))
from variant import verify,MANIFEST,CONTRACT,sha
from audit import TOPICS,audit_rows
sys.path.insert(0,str(HERE.parent/'mid360_lio'))
from static_check import pose_row,stamp_ns,stream_stats
from core import IMU_SI,imu_values
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
STREAMS=dict(TOPICS,imu_si=IMU_SI)
PHASES=['baseline','turn','turned_hold','return','final_hold']

def qmean(values):
    q=np.asarray(values,dtype=float)
    eigenvalues,eigenvectors=np.linalg.eigh(q.T@q)
    result=eigenvectors[:,int(np.argmax(eigenvalues))]
    return result/np.linalg.norm(result)

def multiply(a,b):
    ax,ay,az,aw=a;bx,by,bz,bw=b
    return np.asarray([aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,
                       aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz])

def rotation_vector_degrees(first,last):
    relative=multiply(np.asarray([-first[0],-first[1],-first[2],first[3]]),last)
    relative=relative/np.linalg.norm(relative)
    if relative[3]<0:relative=-relative
    length=np.linalg.norm(relative[:3])
    if length<1e-12:return np.zeros(3)
    return relative[:3]/length*math.degrees(2*math.atan2(length,float(relative[3])))

def hold_window(rows,interval):
    start,end=interval
    selected=[r for r in rows if max(start,end-2_000_000_000)<=r['receive_monotonic_ns']<=end]
    if len(selected)<15 or (selected[-1]['receive_monotonic_ns']-selected[0]['receive_monotonic_ns'])*1e-9<1.5:
        raise ValueError('Insufficient final two-second hold window')
    return dict(position_m=np.median([r['position'] for r in selected],axis=0).tolist(),
                quaternion=qmean([r['quaternion'] for r in selected]).tolist(),samples=len(selected),
                first_receive_monotonic_ns=selected[0]['receive_monotonic_ns'],
                last_receive_monotonic_ns=selected[-1]['receive_monotonic_ns'],
                meaning='Derived median position / quaternion mean; original samples unchanged')

def gyro_integral(rows,interval,axis):
    start,end=interval;selected=[r for r in rows if start<=r['receive_monotonic_ns']<=end]
    if len(selected)<2:raise ValueError('Missing IMU rotation interval')
    times=np.asarray([r['stamp_ns'] for r in selected],dtype=np.int64)
    delta=np.diff(times)*1e-9
    if np.any(delta<=0) or np.max(delta)>.1:raise ValueError('Invalid/noncontinuous IMU integration interval')
    gyro=np.asarray([r['gyro'] for r in selected]);increments=(gyro[:-1]+gyro[1:])/2*delta[:,None]
    result=np.sum(increments,axis=0)*180/math.pi
    return dict(rotation_vector_integral_deg_xyz=result.tolist(),about_baseline_up_deg=float(result@axis),
                samples=len(selected),sensor_span_s=float((times[-1]-times[0])*1e-9),
                max_sensor_gap_s=float(max(delta)),
                integration='Trapezoids between received original IMU samples; no fabricated endpoints or bias subtraction')

def analyze_records(records,events):
    intervals={};expected=[(phase,edge) for phase in PHASES for edge in ['start','end']]
    if [(e.get('phase'),e.get('edge')) for e in events]!=expected:raise ValueError('Incomplete/out-of-order operator phase markers')
    last=0
    for event in events:
        stamp=event['receive_monotonic_ns']
        if not isinstance(stamp,int) or stamp<=last:raise ValueError('Invalid marker order')
        last=stamp
    for i,phase in enumerate(PHASES):
        start,end=events[2*i]['receive_monotonic_ns'],events[2*i+1]['receive_monotonic_ns']
        if end<=start:raise ValueError('Empty operator phase')
        intervals[phase]=(start,end)
    for phase in ['baseline','turned_hold','final_hold']:
        if (intervals[phase][1]-intervals[phase][0])*1e-9<4.9:raise ValueError('Incomplete five-second hold')
    mission_start,mission_end=intervals['baseline'][0],intervals['final_hold'][1]
    required=max(1.,(mission_end-mission_start)*1e-9-.5)
    lio=[];imu=[];corrected=[];last=0
    for i,row in enumerate(records,1):
        if row.get('sequence')!=i or row.get('stream') not in STREAMS:raise ValueError('Invalid original sequence/stream')
        now=row.get('receive_monotonic_ns')
        if not isinstance(now,int) or now<last:raise ValueError('Original receipt clock regressed')
        last=now
        if row['stream']=='imu_si':
            msg=deserialize_message(base64.b64decode(row['cdr_base64'],validate=True),Imu)
            if msg.header.frame_id!='mid360_imu':raise ValueError('Unexpected IMU frame')
            values=imu_values(msg)
            imu.append(dict(stamp_ns=stamp_ns(msg),receive_monotonic_ns=now,receive_wall_ns=row['receive_wall_ns'],error=None,**values))
        else:
            # Derived subset index only; original global sequence/file/order remain untouched.
            lio.append(dict(row,sequence=len(lio)+1))
            if row['stream']=='corrected':
                msg=deserialize_message(base64.b64decode(row['cdr_base64'],validate=True),Odometry)
                corrected.append(dict(stamp_ns=stamp_ns(msg),receive_monotonic_ns=now,**pose_row(msg)))
    lio_audit=audit_rows(lio,required)
    lio_audit.pop('corrected_drift',None)  # This is a movement trial, not an all-run stationary capture.
    imu_stats=stream_stats(imu,required)
    imu_ok=(imu_stats['pass_'] and imu_stats.get('max_receive_gap_s',math.inf)<=.1 and imu_stats.get('max_sensor_gap_s',math.inf)<=.1)
    holds={phase:hold_window(corrected,intervals[phase]) for phase in ['baseline','turned_hold','final_hold']}
    baseline_imu=[r for r in imu if intervals['baseline'][1]-2_000_000_000<=r['receive_monotonic_ns']<=intervals['baseline'][1]]
    if len(baseline_imu)<200:raise ValueError('Insufficient baseline IMU')
    force=np.mean([r['acc'] for r in baseline_imu],axis=0);norm=float(np.linalg.norm(force))
    if not math.isfinite(norm) or norm<=1e-6:raise ValueError('Undefined baseline specific-force direction')
    up=force/norm
    first,turned,final=(holds[p] for p in ['baseline','turned_hold','final_hold'])
    out=rotation_vector_degrees(first['quaternion'],turned['quaternion'])
    back=rotation_vector_degrees(turned['quaternion'],final['quaternion'])
    closure=rotation_vector_degrees(first['quaternion'],final['quaternion'])
    gyro_out=gyro_integral(imu,intervals['turn'],up);gyro_back=gyro_integral(imu,intervals['return'],up)
    outward=float(out@up);returning=float(back@up)
    exercised=abs(outward)>=5 and abs(gyro_out['about_baseline_up_deg'])>=5
    return_exercised=abs(returning)>=5 and abs(gyro_back['about_baseline_up_deg'])>=5
    direction=(outward>5 and gyro_out['about_baseline_up_deg']>5 and returning< -5 and gyro_back['about_baseline_up_deg']< -5)
    return dict(evidence_complete=bool(lio_audit['evidence_complete'] and imu_ok),
        lio_output_audit=lio_audit,imu_stream=imu_stats,hold_windows=holds,
        baseline_specific_force_m_s2=force.tolist(),estimated_up_axis_in_sensor_frame=up.tolist(),
        outward_rotation_vector_in_initial_sensor_deg=out.tolist(),return_rotation_vector_in_turned_sensor_deg=back.tolist(),
        outward_about_estimated_up_deg=outward,return_about_estimated_up_deg=returning,
        imu_outward=gyro_out,imu_return=gyro_back,
        rotation_exercised=exercised,return_exercised=return_exercised,
        direction_consistent_with_operator_declared_ccw_then_cw=direction,
        minimum_exercised_angle_deg=5.,
        final_orientation_closure_deg=float(np.linalg.norm(closure)),
        final_sensor_position_closure_m=float(np.linalg.norm(np.asarray(final['position_m'])-first['position_m'])),
        outward_sensor_displacement_m=float(np.linalg.norm(np.asarray(turned['position_m'])-first['position_m'])),
        angle_reference='NO INDEPENDENT ANGLE TRUTH; approximate target is not measured angle',
        position_meaning='Sensor origin, not chassis centre. Rotation lever-arm motion is not classified as drift',
        up_axis_assumption='Stationary support specific force estimates upward direction; not a mounting calibration',
        no_accuracy_acceptance_threshold_invented=True)

def analyze(directory):
    root=Path(directory);meta=json.loads((root/'capture_meta.json').read_text())
    for name,value in meta['data_sha256'].items():
        if sha(root/name)!=value:raise ValueError('Original data hash changed: '+name)
    if sha(root/'variant_manifest_snapshot.json')!=meta['variant_manifest_sha256']:raise ValueError('Manifest snapshot hash mismatch')
    records=[json.loads(s) for s in (root/'native.jsonl').read_text().splitlines()]
    events=[json.loads(s) for s in (root/'events.jsonl').read_text().splitlines()]
    try:result=analyze_records(records,events)
    except Exception as error:result=dict(evidence_complete=False,analysis_error=str(error),rotation_exercised=False,return_exercised=False)
    h=meta['health'];sources=meta['publishers'];p=meta['actual_parameters']
    sources_ok=all(len(sources.get(k,[]))==1 and sources[k][0]['name']=='mid360_real_lio' and sources[k][0]['namespace']=='/hardware/mid360/lio' for k in ['corrected','predicted','mixed'])
    imu_source=sources.get('imu_si',[])
    sources_ok=sources_ok and len(imu_source)==1 and imu_source[0]['name']=='mid360_imu_unit_adapter'
    gates=dict(writer_health=h['closed'] and h['error'] is None and h['overflow']==0 and h['received']==h['written']==h['actual_lines']==len(records),
        source_identity=sources_ok,parameters=p.get('common.odometry_output_contract')==CONTRACT and p.get('use_sim_time') is False and p.get('common.visualization_frame')=='imu',
        inputs_unchanged=meta['inputs_unchanged'],parameters_unchanged=meta['parameters_unchanged'],not_interrupted=not meta['interrupted'],capture_finished_without_error=meta.get('failure') is None)
    result.update(scope='REAL SENSOR-LOCAL MANUAL GIMBAL TURN/RETURN DIAGNOSTIC; NOT NAVIGATION ACCEPTANCE',
        gates=gates,health=h,publishers=sources,actual_parameters=p,original_data_sha256=meta['data_sha256'],
        limits=['Manual direction and fixed chassis are operator declarations, not independent truth',
                'No angular/position accuracy acceptance','No motor or navigation command','No known pivot offset or chassis calibration',
                'Zero upstream Twist is not a stationarity measurement'])
    result['evidence_complete']=bool(result['evidence_complete'] and all(gates.values()))
    result['motion_check_complete']=bool(result['evidence_complete'] and result.get('rotation_exercised') and result.get('return_exercised') and result.get('direction_consistent_with_operator_declared_ccw_then_cw'))
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--output-name',default='independent_review.json');args=parser.parse_args()
    if Path(args.output_name).name!=args.output_name:parser.error('Use a new filename')
    result=analyze(args.directory);target=Path(args.directory)/args.output_name
    with target.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',target)
    raise SystemExit(0 if result['evidence_complete'] else 2)
