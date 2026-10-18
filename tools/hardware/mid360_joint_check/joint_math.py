"""Provisional inverted-mount geometry; no ROS output and no fitted dimensions."""
import bisect,math,statistics,sys
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360_mount_check'))
from check_measurements import inspect

PAIR_LIMIT_NS=50_000_000

def rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(float(q@q)-1)>1e-3:
        raise ValueError('Invalid original quaternion')
    q=q/np.linalg.norm(q);x,y,z,w=q
    return np.asarray([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.asarray([[c,-s,0.],[s,c,0.],[0.,0.,1.]])

def mount(data):
    report=inspect(data)
    if not report['measurement_data_complete']:raise ValueError('Incomplete mount worksheet')
    g=report['geometry_candidate']
    if any(v!=0 for v in g['base_to_axis_xy_m']):raise ValueError('This diagnostic requires declared concentric chassis/pivot axes')
    R=rz(math.radians(g['native_x_yaw_at_zero_deg']))@np.diag([1.,-1.,-1.])
    arm=-R.T@np.asarray(g['axis_ground_to_imu_at_zero_m'])
    return R,arm,report

def phase_bounds(phases):
    expected=['baseline','turn_left','turned_hold','return','final_hold']
    if [p.get('phase') for p in phases]!=expected:raise ValueError('Incomplete phase order')
    previous=None
    for p in phases:
        a,b=p['start_monotonic_ns'],p['end_monotonic_ns']
        if type(a)!=int or type(b)!=int or b-a<4_900_000_000 or (previous is not None and a!=previous):
            raise ValueError('Noncontinuous/short original phases')
        previous=b
    return phases[0]['start_monotonic_ns'],phases[-1]['end_monotonic_ns']

def window(rows,phase):
    a,b=phase['start_monotonic_ns'],phase['end_monotonic_ns']
    selected=[r for r in rows if max(a,b-2_000_000_000)<=r['receive_monotonic_ns']<=b]
    if len(selected)<15 or selected[-1]['receive_monotonic_ns']-selected[0]['receive_monotonic_ns']<1_500_000_000:
        raise ValueError('Insufficient original corrected samples in final two-second hold')
    return selected

def mean_rotation(rows):
    q=np.asarray([r['quaternion'] for r in rows]);_,v=np.linalg.eigh(q.T@q)
    return rotation(v[:,-1])

def derive(corrected,encoder,phases,measurements):
    start,end=phase_bounds(phases);R_G_I,arm,worksheet=mount(measurements)
    if not corrected or not encoder:raise ValueError('Missing pose/encoder samples')
    for rows,key in [(corrected,'receive_monotonic_ns'),(encoder,'host_completion_receive_monotonic_ns')]:
        times=[r[key] for r in rows]
        if any(type(t)!=int for t in times) or any(b<a for a,b in zip(times,times[1:])):
            raise ValueError('Original receive times regressed or invalid')
    poses=[r for r in corrected if start<=r['receive_monotonic_ns']<=end]
    if len(poses)<3:raise ValueError('Missing mission corrected poses')
    times=[r['receive_monotonic_ns'] for r in poses]
    if times[0]-start>200_000_000 or end-times[-1]>200_000_000 or max(np.diff(times))>200_000_000:
        raise ValueError('Corrected mission receipt coverage/gap exceeds 0.2 seconds')
    valid=[];last_angle=None;unwrapped=None
    for r in encoder:
        t=r['host_completion_receive_monotonic_ns']
        if start<=t<=end and not (r['relative_yaw_valid'] and r['structural_fields_valid']):
            raise ValueError('Invalid original encoder during mission')
        if not (r['relative_yaw_valid'] and r['structural_fields_valid']):continue
        angle=r['gimbal_relative_angle_rad']
        if not math.isfinite(angle):raise ValueError('Nonfinite encoder angle')
        unwrapped=angle if last_angle is None else unwrapped+math.atan2(math.sin(angle-last_angle),math.cos(angle-last_angle))
        valid.append(dict(r,unwrapped_rad=unwrapped));last_angle=angle
    mission_enc=[r for r in valid if start<=r['host_completion_receive_monotonic_ns']<=end]
    et=[r['host_completion_receive_monotonic_ns'] for r in mission_enc]
    if len(et)<3 or et[0]-start>200_000_000 or end-et[-1]>200_000_000 or max(np.diff(et))>200_000_000:
        raise ValueError('Encoder mission receipt coverage/gap exceeds 0.2 seconds')
    base_phase=phases[0];base_enc=[r['unwrapped_rad'] for r in mission_enc if base_phase['end_monotonic_ns']-2_000_000_000<=r['host_completion_receive_monotonic_ns']<=base_phase['end_monotonic_ns']]
    if len(base_enc)<3:raise ValueError('Missing baseline encoder')
    zero=statistics.median(base_enc)
    reference=mean_rotation(window(poses,base_phase))@R_G_I.T
    receipt=[r['host_completion_receive_monotonic_ns'] for r in valid]
    timeline=[]
    for r in poses:
        t=r['receive_monotonic_ns'];i=bisect.bisect_left(receipt,t)
        choices=[j for j in [i-1,i] if 0<=j<len(valid)]
        j=min(choices,key=lambda j:(abs(receipt[j]-t),j));enc=valid[j];dt=receipt[j]-t
        if abs(dt)>PAIR_LIMIT_NS:raise ValueError('Nearest received encoder exceeds 50 ms association filter')
        p=np.asarray(r['position'],dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('Invalid original position')
        R_L_I=rotation(r['quaternion']);joint=enc['unwrapped_rad']-zero
        gimbal=reference.T@R_L_I@R_G_I.T
        body=gimbal@rz(-joint);centre=p+R_L_I@arm
        timeline.append(dict(sequence=r['sequence'],stamp_ns=r['stamp_ns'],receive_monotonic_ns=t,
            sensor_position_m=p.tolist(),sensor_quaternion=r['quaternion'],provisional_centre_in_lio_m=centre.tolist(),
            encoder_frame_offset=enc['offset'],encoder_receive_monotonic_ns=receipt[j],pair_receive_delta_ns=dt,
            encoder_angle_rad=enc['gimbal_relative_angle_rad'],encoder_change_deg=math.degrees(joint),
            gimbal_forward_heading_change_deg=math.degrees(math.atan2(gimbal[1,0],gimbal[0,0])),
            provisional_body_yaw_change_deg=math.degrees(math.atan2(body[1,0],body[0,0])),
            provisional_body_rotation_change_deg=math.degrees(math.acos(float(np.clip((np.trace(body)-1)/2,-1,1)))),
            meaning='Derived nearest-host-receipt pairing, not synchronized measurement times; original samples unchanged'))
    holds={}
    # Window() needs only receive time for derived records; no resampling is performed.
    for phase in [phases[0],phases[2],phases[4]]:
        selected=window(timeline,phase)
        holds[phase['phase']]=dict(samples=len(selected),
            centre_in_lio_m=np.median([r['provisional_centre_in_lio_m'] for r in selected],axis=0).tolist(),
            sensor_in_lio_m=np.median([r['sensor_position_m'] for r in selected],axis=0).tolist(),
            body_yaw_change_median_deg=float(np.median([r['provisional_body_yaw_change_deg'] for r in selected])),
            gimbal_heading_change_median_deg=float(np.median([r['gimbal_forward_heading_change_deg'] for r in selected])),
            encoder_change_median_deg=float(np.median([r['encoder_change_deg'] for r in selected])))
    first=holds['baseline'];origin=np.asarray(first['centre_in_lio_m']);sensor_origin=np.asarray(first['sensor_in_lio_m'])
    stats=dict(corrected_mission_samples=len(timeline),hold_windows=holds,
        max_absolute_body_yaw_change_deg=max(abs(r['provisional_body_yaw_change_deg']) for r in timeline),
        max_body_rotation_change_deg=max(r['provisional_body_rotation_change_deg'] for r in timeline),
        max_sampled_centre_distance_m=max(float(np.linalg.norm(np.asarray(r['provisional_centre_in_lio_m'])-origin)) for r in timeline),
        max_sampled_sensor_distance_m=max(float(np.linalg.norm(np.asarray(r['sensor_position_m'])-sensor_origin)) for r in timeline),
        turn_centre_displacement_m=float(np.linalg.norm(np.asarray(holds['turned_hold']['centre_in_lio_m'])-origin)),
        return_centre_closure_m=float(np.linalg.norm(np.asarray(holds['final_hold']['centre_in_lio_m'])-origin)),
        max_absolute_pair_receive_delta_s=max(abs(r['pair_receive_delta_ns'])*1e-9 for r in timeline),
        mechanical_forward_encoder_candidate_rad=math.atan2(math.sin(zero),math.cos(zero)),
        imu_to_axis_ground_point_in_imu_m=arm.tolist(),mount_worksheet=worksheet)
    return stats,timeline
