"""Offline provisional axis-centre preview. No ROS node, TF publisher or motor output."""
import argparse,base64,json,math,sys
from pathlib import Path
import numpy as np
from check_measurements import inspect
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'mid360_gimbal_check'))
from analyze import analyze,sha,deserialize_message,Odometry,pose_row,stamp_ns,hold_window,PHASES


def rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(q@q-1.)>.01:raise ValueError('Invalid quaternion')
    q=q/np.linalg.norm(q);x,y,z,w=q
    return np.asarray([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def arm_in_imu(measurements):
    report=inspect(measurements)
    if not report['measurement_data_complete']:raise ValueError('Incomplete mount measurements: '+json.dumps(report['missing']+report['invalid']))
    # This diagnostic can avoid live joint feedback ONLY for declared concentric axes.
    if any(v!=0 for v in report['geometry_candidate']['base_to_axis_xy_m']):
        raise ValueError('Axis not declared concentric: joint association/body orientation required; no approximation applied')
    yaw=math.radians(report['geometry_candidate']['native_x_yaw_at_zero_deg']);c,s=math.cos(yaw),math.sin(yaw)
    R_G_I=np.asarray([[c,s,0.],[s,-c,0.],[0.,0.,-1.]])
    p_G_I=np.asarray(report['geometry_candidate']['axis_ground_to_imu_at_zero_m'])
    return -R_G_I.T@p_G_I,report


def preview_rows(rows,measurements):
    arm,report=arm_in_imu(measurements);out=[]
    for row in rows:
        p=np.asarray(row['position'],dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('Invalid IMU position')
        centre=p+rotation(row['quaternion'])@arm
        out.append(dict(row,position=centre.tolist(),sensor_position=list(row['position'])))
    return out,arm.tolist(),report


def preview(root,measurement_path):
    root=Path(root);measurement_path=Path(measurement_path);measurements=json.loads(measurement_path.read_text())
    audit=analyze(root)
    if not audit['evidence_complete'] or not audit['motion_check_complete']:
        raise ValueError('Require original complete turn/return check; diagnostic cannot repair its verdict')
    records=[json.loads(line) for line in (root/'native.jsonl').read_text().splitlines()]
    events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    rows=[]
    for record in records:
        if record['stream']!='corrected':continue
        msg=deserialize_message(base64.b64decode(record['cdr_base64'],validate=True),Odometry)
        rows.append(dict(sequence=record['sequence'],stamp_ns=stamp_ns(msg),receive_monotonic_ns=record['receive_monotonic_ns'],**pose_row(msg)))
    centres,arm,worksheet=preview_rows(rows,measurements)
    intervals={phase:(events[2*i]['receive_monotonic_ns'],events[2*i+1]['receive_monotonic_ns']) for i,phase in enumerate(PHASES)}
    holds={phase:hold_window(centres,intervals[phase]) for phase in ['baseline','turned_hold','final_hold']}
    base=np.asarray(holds['baseline']['position_m']);sensor_base=np.asarray(audit['hold_windows']['baseline']['position_m'])
    mission_rows=[r for r in centres if intervals['baseline'][0]<=r['receive_monotonic_ns']<=intervals['final_hold'][1]]
    if not mission_rows:raise ValueError('No original corrected samples in mission')
    stats=dict(corrected_samples=len(rows),mission_samples=len(mission_rows),
        original_sensor_turn_hold_displacement_m=audit['outward_sensor_displacement_m'],
        provisional_centre_turn_hold_displacement_m=float(np.linalg.norm(np.asarray(holds['turned_hold']['position_m'])-base)),
        original_sensor_return_closure_m=audit['final_sensor_position_closure_m'],
        provisional_centre_return_closure_m=float(np.linalg.norm(np.asarray(holds['final_hold']['position_m'])-base)),
        sensor_max_sample_distance_from_baseline_m=max(float(np.linalg.norm(np.asarray(r['sensor_position'])-sensor_base)) for r in mission_rows),
        provisional_centre_max_sample_distance_from_baseline_m=max(float(np.linalg.norm(np.asarray(r['position'])-base)) for r in mission_rows))
    result=dict(scope='PROVISIONAL OFFLINE MOUNT LEVER-ARM PREVIEW; NOT CHASSIS ODOMETRY ACCEPTANCE',
        original_evidence_complete=True,original_motion_check_complete=True,
        original_data_sha256=audit['original_data_sha256'],measurement_path=str(measurement_path.resolve()),measurement_sha256=sha(measurement_path),
        tool_sha256=sha(Path(__file__)),mount_checker_sha256=sha(Path(__file__).with_name('check_measurements.py')),analyzer_sha256=sha(Path(__file__).parent.parent/'mid360_gimbal_check/analyze.py'),
        measurement_quality=measurements.get('measurement_quality',{}),mount_worksheet=worksheet,
        imu_to_axis_ground_point_in_imu_m=arm,statistics=stats,derived_centre_hold_windows=holds,
        navigation_ready=False,body_yaw_computed=False,live_joint_feedback_used=False,published_ros_data=False,
        limitations=['User declared fixed chassis and concentric yaw axis; neither independently verified',
            'Forward offset and height are rough, no uncertainty bound or calibration-accuracy threshold',
            'Same lever arm applied to each original pose; dimensions never fitted to improve this trial',
            'Derived axis-ground positions are in original sensor-local LIO frame, not gravity/map-qualified coordinates',
            'Rotation errors couple the long vertical lever into horizontal error; no error removed or corrected',
            'Uses nominal internal IMU offset; not this sensor individual calibration',
            'No body heading, encoder zero, sensor/encoder clock alignment, motor control or navigation acceptance'])
    timeline=[dict(sequence=r['sequence'],stamp_ns=r['stamp_ns'],receive_monotonic_ns=r['receive_monotonic_ns'],
        sensor_position_m=r['sensor_position'],provisional_axis_ground_position_m=r['position'],quaternion=r['quaternion']) for r in centres]
    return result,timeline


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--measurements',default=str(Path(__file__).with_name('mount_measurements.operator_20261018_v1.json')))
    parser.add_argument('--output-prefix',default='mount_preview_v1');args=parser.parse_args()
    if Path(args.output_prefix).name!=args.output_prefix:parser.error('Use a filename prefix only')
    root=Path(args.directory);summary=root/(args.output_prefix+'.json');native=root/(args.output_prefix+'.derived.jsonl')
    if summary.exists() or native.exists():raise FileExistsError('Preserving existing preview files')
    result,timeline=preview(root,args.measurements)
    with native.open('x') as f:
        for row in timeline:f.write(json.dumps(row,allow_nan=False)+'\n')
    result['derived_timeline_sha256']=sha(native)
    with summary.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(scope=result['scope'],statistics=result['statistics'],navigation_ready=False,body_yaw_computed=False,
        measurement_quality=result['measurement_quality']),indent=2,allow_nan=False));print('PREVIEW:',summary)

if __name__=='__main__':main()
