"""Offline timing diagnostic for captured IMU/LIO; never changes an acceptance result."""
import argparse,base64,json,math
from pathlib import Path
import numpy as np
from analyze import (analyze,sha,rotation_vector_degrees,imu_values,pose_row,
                     stamp_ns,deserialize_message,Imu,Odometry,PHASES)


def decode(records):
    imu=[];lio=[]
    for row in records:
        kind=row['stream']
        if kind not in ['imu_si','corrected']:continue
        msg=deserialize_message(base64.b64decode(row['cdr_base64'],validate=True),Imu if kind=='imu_si' else Odometry)
        common=dict(sequence=row['sequence'],receive_monotonic_ns=row['receive_monotonic_ns'],
                    receive_wall_ns=row['receive_wall_ns'],stamp_ns=stamp_ns(msg))
        if kind=='imu_si':imu.append(dict(common,**imu_values(msg)))
        else:lio.append(dict(common,**pose_row(msg)))
    return imu,lio


def integral(rows,axis):
    if len(rows)<2:return None
    delta=np.diff(np.asarray([r['stamp_ns'] for r in rows],dtype=np.int64))*1e-9
    if np.any(delta<=0) or np.max(delta)>.1:raise ValueError('Invalid IMU sample spacing')
    gyro=np.asarray([r['gyro'] for r in rows]);mid=(gyro[:-1]+gyro[1:])/2
    components=np.sum(mid*delta[:,None],axis=0)*180/math.pi
    return dict(component_integrals_deg_xyz=components.tolist(),projected_net_deg=float(components@axis),
                projected_travel_deg=float(np.sum(np.abs(mid@axis)*delta)*180/math.pi))


def segment(label,start,end,imu,lio,axis,origin):
    # Half-open selection partitions raw samples; no fabricated boundary sample.
    ir=[r for r in imu if start<=r['receive_monotonic_ns']<end]
    lr=[r for r in lio if start<=r['receive_monotonic_ns']<end]
    angle=None
    if len(lr)>=2:
        angle=float(rotation_vector_degrees(lr[0]['quaternion'],lr[-1]['quaternion'])@axis)
    def coverage(rows):
        if not rows:return None
        offsets=[(r['receive_wall_ns']-r['stamp_ns'])*1e-9 for r in rows]
        return dict(first_sequence=rows[0]['sequence'],last_sequence=rows[-1]['sequence'],
                    first_receive_relative_s=(rows[0]['receive_monotonic_ns']-origin)*1e-9,
                    last_receive_relative_s=(rows[-1]['receive_monotonic_ns']-origin)*1e-9,
                    first_device_stamp_ns=rows[0]['stamp_ns'],last_device_stamp_ns=rows[-1]['stamp_ns'],
                    wall_minus_device_median_s=float(np.median(offsets)),
                    wall_minus_device_range_s=[float(min(offsets)),float(max(offsets))])
    peak=max(ir,key=lambda r:abs(np.asarray(r['gyro'])@axis)) if ir else None
    return dict(label=label,start_relative_s=(start-origin)*1e-9,end_relative_s=(end-origin)*1e-9,
                duration_s=(end-start)*1e-9,imu_count=len(ir),lio_count=len(lr),
                imu_integral=integral(ir,axis),lio_first_to_last_projected_deg=angle,
                peak_projected_gyro_deg_s=float(np.asarray(peak['gyro'])@axis*180/math.pi) if peak else None,
                peak_gyro_relative_s=(peak['receive_monotonic_ns']-origin)*1e-9 if peak else None,
                imu_coverage=coverage(ir),lio_coverage=coverage(lr))


def timeline(records,events,axis):
    if [(e.get('phase'),e.get('edge')) for e in events]!=[(p,e) for p in PHASES for e in ['start','end']]:
        raise ValueError('Missing/out-of-order markers')
    marks=[e['receive_monotonic_ns'] for e in events]
    if any(b<=a for a,b in zip(marks,marks[1:])):raise ValueError('Nonmonotonic markers')
    axis=np.asarray(axis,dtype=float)
    if axis.shape!=(3,) or not np.all(np.isfinite(axis)) or abs(np.linalg.norm(axis)-1)>.000001:raise ValueError('Invalid upward axis')
    imu,lio=decode(records)
    if not imu or not lio:raise ValueError('Missing motion streams')
    origin=marks[0];first=records[0]['receive_monotonic_ns'];last=records[-1]['receive_monotonic_ns']+1
    if first>marks[0] or last<=marks[-1]:raise ValueError('Capture does not cover all markers')
    windows=[]
    cursor=first
    for i,phase in enumerate(PHASES):
        start,end=marks[2*i:2*i+2]
        if start>cursor:windows.append(segment('before_baseline' if i==0 else 'between_'+PHASES[i-1]+'_and_'+phase,cursor,start,imu,lio,axis,origin))
        windows.append(segment(phase,start,end,imu,lio,axis,origin));cursor=end
    if last>cursor:windows.append(segment('after_final_hold',cursor,last,imu,lio,axis,origin))
    assert sum(w['imu_count'] for w in windows)==len(imu)
    assert sum(w['lio_count'] for w in windows)==len(lio)
    bins=[]
    for start in range(first,last,1_000_000_000):
        b=segment('one_second',start,min(start+1_000_000_000,last),imu,lio,axis,origin)
        b['operator_phase']=next((w['label'] for w in windows if w['start_relative_s']<=b['start_relative_s']<w['end_relative_s']),None)
        bins.append(b)
    return dict(phase_and_gap_windows=windows,one_second_bins=bins,
                partition_counts=dict(imu=len(imu),corrected=len(lio)),
                top_gyro_bins=sorted([b for b in bins if b['imu_integral'] is not None],key=lambda b:b['imu_integral']['projected_travel_deg'],reverse=True)[:6],
                top_lio_bins=sorted([b for b in bins if b['lio_first_to_last_projected_deg'] is not None],key=lambda b:abs(b['lio_first_to_last_projected_deg']),reverse=True)[:6])


def diagnose(root):
    root=Path(root);audit=analyze(root)  # Recheck original hashes and original acceptance gates.
    if not audit['evidence_complete']:raise ValueError('Original evidence incomplete; retain original result and diagnose capture failure first')
    records=[json.loads(line) for line in (root/'native.jsonl').read_text().splitlines()]
    events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    data=timeline(records,events,audit['estimated_up_axis_in_sensor_frame'])
    return dict(scope='OFFLINE PHASE/TIMING DIAGNOSTIC ONLY; NO NEW MOTION ACCEPTANCE',
                original_evidence_complete=audit['evidence_complete'],original_motion_check_complete=audit['motion_check_complete'],
                input_dir=str(root.resolve()),original_data_sha256=audit['original_data_sha256'],
                diagnostic_sha256=sha(Path(__file__)),analyzer_sha256=sha(Path(__file__).with_name('analyze.py')),
                original_full_audit=audit,**data,
                limits=['No raw edits, reordering, interpolation, bias subtraction or acceptance-threshold change',
                        'All phases, holds and unmarked gaps are included; no marker is moved to obtain a pass',
                        'Gyro component integrals and fixed-up projections assume nearly yaw-only motion; not general 3D attitude integration',
                        'LIO segment endpoints are first/last actual samples; this differs from terminal hold means',
                        'Missing segment samples stay null, not zero; per-bin differences include unequal sample coverage',
                        'Wall-minus-device includes unknown clock offset; not a measured one-way latency',
                        'Bins use receive time; sensor and receive clocks are preserved, not silently aligned',
                        'No independent angular truth, pivot calibration or identified root cause'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--output-name',default='timing_review_v1.json');args=parser.parse_args()
    if Path(args.output_name).name!=args.output_name:parser.error('Use a new filename')
    target=Path(args.directory)/args.output_name
    if target.exists():raise FileExistsError('Preserving existing report: '+str(target))
    result=diagnose(args.directory)
    with target.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    def compact(w):
        g=w['imu_integral']
        return dict(phase=w['label'],start_s=w['start_relative_s'],end_s=w['end_relative_s'],
                    imu_net_deg=g['projected_net_deg'] if g else None,
                    imu_travel_deg=g['projected_travel_deg'] if g else None,
                    lio_delta_deg=w['lio_first_to_last_projected_deg'])
    print(json.dumps(dict(original_motion_check_complete=result['original_motion_check_complete'],
        phase_and_gap_windows=[compact(w) for w in result['phase_and_gap_windows']],
        top_gyro_bins=[compact(w) for w in result['top_gyro_bins']],
        top_lio_bins=[compact(w) for w in result['top_lio_bins']],
        root_cause='NOT YET DETERMINED; review full timing report'),indent=2,allow_nan=False))
    print('TIMING REPORT:',target)

if __name__=='__main__':main()
