"""Reference-based translation diagnostics; original samples retained, no accuracy acceptance."""
import bisect,math,statistics,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360_joint_check'))
from joint_math import mount,rotation,rz,mean_rotation,window,PAIR_LIMIT_NS

PHASES=['baseline','outward','far_hold','return','final_hold']

def intervals(events):
    if [(e.get('phase'),e.get('edge')) for e in events]!=[(p,k) for p in PHASES for k in ['start','end']]:raise ValueError('Missing/out-of-order original operator markers')
    previous=0;phases=[]
    for i,p in enumerate(PHASES):
        a,b=events[2*i]['receive_monotonic_ns'],events[2*i+1]['receive_monotonic_ns']
        if type(a)!=int or type(b)!=int or a<previous or b<=a:raise ValueError('Invalid marker timestamps')
        if p in ['baseline','far_hold','final_hold'] and b-a<4_900_000_000:raise ValueError('Short original hold phase')
        phases.append(dict(phase=p,start_monotonic_ns=a,end_monotonic_ns=b));previous=b
    return phases

def reference(data):
    d=data.get('distance_m')
    if isinstance(d,bool) or not isinstance(d,(int,float)) or not math.isfinite(d) or not 0<d<=2:raise ValueError('Measured A-B distance must be finite, >0 and <=2m')
    for k in ['source','reference_point','direction']:
        if not isinstance(data.get(k),str) or not data[k].strip():raise ValueError('Missing reference provenance: '+k)
    return float(d)

def derive_translation(corrected,encoder,events,measurements,ref):
    distance=reference(ref);phases=intervals(events);start=phases[0]['start_monotonic_ns'];end=phases[-1]['end_monotonic_ns']
    R_G_I,arm,worksheet=mount(measurements)
    for values,key in [(corrected,'receive_monotonic_ns'),(encoder,'host_completion_receive_monotonic_ns')]:
        times=[r[key] for r in values]
        if not times or any(type(t)!=int for t in times) or any(b<a for a,b in zip(times,times[1:])):raise ValueError('Missing/regressed original times')
    poses=[r for r in corrected if start<=r['receive_monotonic_ns']<=end]
    ts=[r['receive_monotonic_ns'] for r in poses]
    if len(ts)<3 or ts[0]-start>200_000_000 or end-ts[-1]>200_000_000 or max(np.diff(ts))>200_000_000:raise ValueError('Missing corrected coverage')
    valid=[];last=None;unwrapped=None
    for r in encoder:
        t=r['host_completion_receive_monotonic_ns'];good=r['relative_yaw_valid'] and r['structural_fields_valid']
        if start<=t<=end and not good:raise ValueError('Invalid original encoder within mission')
        if not good:continue
        angle=r['gimbal_relative_angle_rad']
        if not math.isfinite(angle):raise ValueError('Nonfinite original angle')
        unwrapped=angle if last is None else unwrapped+math.atan2(math.sin(angle-last),math.cos(angle-last))
        valid.append(dict(r,unwrapped_rad=unwrapped));last=angle
    enc=[r for r in valid if start<=r['host_completion_receive_monotonic_ns']<=end];et=[r['host_completion_receive_monotonic_ns'] for r in enc]
    if len(et)<3 or et[0]-start>200_000_000 or end-et[-1]>200_000_000 or max(np.diff(et))>200_000_000:raise ValueError('Missing encoder coverage')
    b=phases[0]['end_monotonic_ns'];base_angles=[r['unwrapped_rad'] for r in enc if b-2_000_000_000<=r['host_completion_receive_monotonic_ns']<=b]
    if len(base_angles)<3:raise ValueError('Missing baseline encoder')
    zero=statistics.median(base_angles);R_L_B0=mean_rotation(window(poses,phases[0]))@R_G_I.T
    converted=[]
    for p in poses:
        position=np.asarray(p['position'],dtype=float)
        if position.shape!=(3,) or not np.isfinite(position).all():raise ValueError('Invalid original pose')
        R=rotation(p['quaternion']);converted.append(dict(p,centre=(position+R@arm).tolist()))
    initial=np.median([r['centre'] for r in window(converted,phases[0])],axis=0)
    enc_times=[r['host_completion_receive_monotonic_ns'] for r in valid];timeline=[]
    for p in converted:
        t=p['receive_monotonic_ns'];i=bisect.bisect_left(enc_times,t);choices=[j for j in [i-1,i] if 0<=j<len(valid)]
        j=min(choices,key=lambda j:(abs(enc_times[j]-t),j));angle=valid[j];delta=enc_times[j]-t
        if abs(delta)>PAIR_LIMIT_NS:raise ValueError('No original encoder within 50ms host-receipt filter')
        body=R_L_B0.T@rotation(p['quaternion'])@R_G_I.T@rz(-(angle['unwrapped_rad']-zero))
        xyz=R_L_B0.T@(np.asarray(p['centre'])-initial)
        timeline.append(dict(sequence=p['sequence'],stamp_ns=p['stamp_ns'],receive_monotonic_ns=t,
            sensor_position_lio_m=p['position'],sensor_quaternion=p['quaternion'],centre_in_lio_m=p['centre'],
            displacement_initial_chassis_m=xyz.tolist(),body_yaw_change_deg=math.degrees(math.atan2(body[1,0],body[0,0])),
            encoder_change_deg=math.degrees(angle['unwrapped_rad']-zero),encoder_frame_offset=angle['offset'],
            encoder_receive_monotonic_ns=enc_times[j],pair_receive_delta_ns=delta))
    holds={}
    for p in [phases[0],phases[2],phases[4]]:
        selected=window(timeline,p)
        holds[p['phase']]=dict(samples=len(selected),displacement_initial_chassis_m=np.median([r['displacement_initial_chassis_m'] for r in selected],axis=0).tolist(),
            body_yaw_median_deg=float(np.median([r['body_yaw_change_deg'] for r in selected])),
            encoder_change_median_deg=float(np.median([r['encoder_change_deg'] for r in selected])))
    a=np.asarray(holds['baseline']['displacement_initial_chassis_m']);b=np.asarray(holds['far_hold']['displacement_initial_chassis_m']);c=np.asarray(holds['final_hold']['displacement_initial_chassis_m'])
    outward=b-a;back=c-b;closure=c-a;moving=[r for r in timeline if phases[1]['start_monotonic_ns']<=r['receive_monotonic_ns']<=phases[1]['end_monotonic_ns']]
    stats=dict(reference_distance_m=distance,corrected_mission_samples=len(timeline),hold_windows=holds,
        outward_delta_initial_chassis_m=outward.tolist(),return_delta_initial_chassis_m=back.tolist(),closure_initial_chassis_m=closure.tolist(),
        estimated_forward_distance_m=float(outward[0]),signed_forward_distance_difference_m=float(outward[0]-distance),
        forward_scale_ratio=float(outward[0]/distance),far_point_euclidean_reference_difference_m=float(np.linalg.norm(outward-np.asarray([distance,0.,0.]))),
        far_lateral_displacement_m=float(outward[1]),far_vertical_displacement_m=float(outward[2]),return_closure_3d_m=float(np.linalg.norm(closure)),
        max_sampled_lateral_displacement_m=max(abs(r['displacement_initial_chassis_m'][1]) for r in timeline),
        max_sampled_body_yaw_change_deg=max(abs(r['body_yaw_change_deg']) for r in timeline),max_sampled_encoder_change_deg=max(abs(r['encoder_change_deg']) for r in timeline),
        max_absolute_pair_receive_delta_s=max(abs(r['pair_receive_delta_ns'])*1e-9 for r in timeline),
        mechanical_forward_encoder_candidate_rad=math.atan2(math.sin(zero),math.cos(zero)),
        R_lio_from_initial_chassis=R_L_B0.tolist(),initial_centre_in_lio_m=initial.tolist(),mount_worksheet=worksheet,
        forward_direction_consistent=bool(outward[0]>0 and back[0]<0),
        accuracy_accepted=False,navigation_ready=False,
        reference_meaning='Tape distance and fixed-mark endpoint alignment are operator-reported; no independent tracking truth or certified uncertainty')
    return stats,timeline,phases
