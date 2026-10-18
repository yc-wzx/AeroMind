"""Independent raw-message audit of an operator-measured straight translation."""
import argparse,base64,json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360_joint_check'))
from review import sources_ok,params_ok,sha,STREAMS,pose_row,stamp_ns,imu_values,stream_stats,audit_rows,deserialize_message,Odometry,Imu,serial_audit,PROFILES
from translation_math import derive_translation,intervals,reference
sys.path.insert(0,str(HERE.parent/'mid360_lio_guard'))
from input_health import PoseHealth

def native_audit(records,events):
    phases=intervals(events);start=phases[0]['start_monotonic_ns'];end=phases[-1]['end_monotonic_ns'];groups={k:[] for k in STREAMS}
    corrected=[];imu=[];lio=[];previous=0
    for i,r in enumerate(records,1):
        kind=r.get('stream');t=r.get('receive_monotonic_ns')
        if r.get('sequence')!=i or kind not in groups or type(t)!=int or t<previous:raise ValueError('Invalid original native order/stream')
        previous=t;groups[kind].append(t);payload=base64.b64decode(r['cdr_base64'],validate=True)
        if kind=='imu_si':
            msg=deserialize_message(payload,Imu)
            if msg.header.frame_id!='mid360_imu':raise ValueError('Wrong IMU frame')
            imu.append(dict(stamp_ns=stamp_ns(msg),receive_monotonic_ns=t,receive_wall_ns=r['receive_wall_ns'],error=None,**imu_values(msg)))
        else:
            lio.append(dict(r,sequence=len(lio)+1))
            if kind=='corrected':
                msg=deserialize_message(payload,Odometry);corrected.append(dict(sequence=r['sequence'],stamp_ns=stamp_ns(msg),receive_monotonic_ns=t,**pose_row(msg)))
    span=max(1.,(end-start)*1e-9-.5);audit=audit_rows(lio,span);audit.pop('corrected_drift',None);imu_stats=stream_stats(imu,span);coverage={}
    for k,times in groups.items():
        times=[t for t in times if start<=t<=end];limit=.1 if k=='imu_si' else .2
        coverage[k]=dict(samples=len(times),receive_gap_limit_s=limit,pass_=bool(len(times)>=3 and times[0]-start<=limit*1e9 and end-times[-1]<=limit*1e9 and max((b-a for a,b in zip(times,times[1:])),default=10**12)<=limit*1e9))
    complete=bool(audit['evidence_complete'] and imu_stats['pass_'] and imu_stats.get('max_receive_gap_s',math.inf)<=.1 and imu_stats.get('max_sensor_gap_s',math.inf)<=.1 and all(r['pass_'] for r in coverage.values()))
    return dict(evidence_complete=complete,lio_output_audit=audit,imu_stream=imu_stats,mission_coverage=coverage),corrected

def review_translation(root):
    root=Path(root).resolve();meta=json.loads((root/'capture_meta.json').read_text());hashes=meta['data_sha256']
    required={'native.jsonl','events.jsonl','reference.json','mount_snapshot.json','local_inputs.json','variant_manifest_snapshot.json','encoder/raw.bin','encoder/reads.jsonl','encoder/capture_meta.json'}
    if not required<=set(hashes):raise ValueError('Missing mandatory raw hashes')
    for name,value in hashes.items():
        p=(root/name).resolve()
        if not p.is_relative_to(root) or sha(p)!=value:raise ValueError('Original file hash mismatch: '+name)
    inputs=json.loads((root/'local_inputs.json').read_text())
    if not inputs:raise ValueError('Missing input snapshots')
    for r in inputs:
        p=(root/r['snapshot']).resolve()
        if not p.is_relative_to(root) or sha(p)!=r['sha256']:raise ValueError('Input snapshot mismatch')
    records=[json.loads(s) for s in (root/'native.jsonl').read_text().splitlines()];events=[json.loads(s) for s in (root/'events.jsonl').read_text().splitlines()]
    h=meta['health'];gates=dict(writer_health=h['closed'] and h['error'] is None and h['overflow']==0 and h['received']==h['written']==h['actual_lines']==len(records),
        sources=sources_ok(meta['publishers_before']) and sources_ok(meta['publishers_after']),parameters=params_ok(meta['parameters_before']) and params_ok(meta['parameters_after']) and meta['parameters_before']==meta['parameters_after'],
        sealed_inputs_verified=meta['sealed_inputs_before'] and meta['sealed_inputs_after'],inputs_unchanged=meta['inputs_unchanged'],capture_finished_without_error=meta['failure'] is None and not meta['interrupted'])
    result=dict(scope='REAL SENSOR / ENCODER MANUAL REFERENCE TRANSLATION; DIAGNOSTIC ONLY',evidence_complete=False,diagnostic_complete=False,
        accuracy_accepted=False,navigation_ready=False,published_ros_data=False,gates=gates,capture_failure=meta['failure'],health=h,
        original_data_sha256=hashes,capture_meta_sha256=sha(root/'capture_meta.json'),reviewer_sha256=sha(Path(__file__)),math_sha256=sha(HERE/'translation_math.py'),
        limitations=['Ground distance and same chassis mark alignment are operator reports, without independent endpoint tracking or certified uncertainty',
            'Initial chassis axes use declared mechanical forward and nominal inverted mount; not official map/gravity alignment',
            'No dimensions or scale fitted; encoder and LIO paired by host receipt only, not device sampling time',
            'Rough mount and long vertical lever can couple orientation error into position',
            'No motor/navigation/TF publication, no reset/set_pose service monitoring, no invented accuracy acceptance threshold'])
    timeline=[]
    try:
        ref=json.loads((root/'reference.json').read_text());reference(ref)
        serial,frames=serial_audit(root/'encoder');result['encoder']=serial
        if serial['expected_crc_profile']!='0x8408' or not serial['receive_profile_matches'] or not serial['capture_complete']:raise ValueError('Encoder capture/explicit CRC8408 incomplete')
        native,corrected=native_audit(records,events);result['native']=native
        frames=sorted((r for r in frames if r['crc_algorithm']==PROFILES['0x8408']),key=lambda r:r['offset'])
        stats,timeline,phases=derive_translation(corrected,frames,events,json.loads((root/'mount_snapshot.json').read_text()),ref)
        result['reference']=ref;result['statistics']=stats;result['phase_intervals']=phases
        result['evidence_complete']=bool(all(gates.values()) and native['evidence_complete'])
        pose_health=PoseHealth()
        for row in corrected:
            pose_health.accept(row['stamp_ns'],row['receive_monotonic_ns'],row['position'],row['quaternion'])
        holds=[]
        for phase in [phases[0],phases[2],phases[4]]:
            h=PoseHealth()
            for row in corrected:
                if phase['start_monotonic_ns']<=row['receive_monotonic_ns']<=phase['end_monotonic_ns']:
                    h.accept(row['stamp_ns'],row['receive_monotonic_ns'],row['position'],row['quaternion'])
            holds.append(dict(phase=phase['phase'],**h.static_report(start_ns=phase['start_monotonic_ns'])))
        result['localization_sanity']=dict(pass_=pose_health.failure is None and all(h['pass_'] for h in holds),failure=pose_health.failure,holds=holds,accuracy_accepted=False)
        result['diagnostic_complete']=bool(result['evidence_complete'] and result['localization_sanity']['pass_'])
        result['status']='REFERENCE_TRANSLATION_DIAGNOSTIC_COMPLETE_NO_ACCURACY_ACCEPTANCE' if result['diagnostic_complete'] else 'LOCALIZATION_SANITY_REJECTED' if result['evidence_complete'] else 'EVIDENCE_INCOMPLETE'
    except Exception as error:result['analysis_error']=str(error);result['status']='EVIDENCE_INCOMPLETE'
    return result,timeline

def save_review(root,name):
    root=Path(root)
    if Path(name).name!=name:raise ValueError('Use a filename only')
    output=root/name;derived=root/(Path(name).stem+'.derived.jsonl')
    if output.exists() or derived.exists():raise FileExistsError('Preserving existing report')
    result,rows=review_translation(root)
    with derived.open('x') as f:
        for r in rows:f.write(json.dumps(r,allow_nan=False)+'\n')
    result['derived_timeline_sha256']=sha(derived)
    with output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--output-name',default='independent_translation_review_v1.json');a=p.parse_args()
    result=save_review(a.directory,a.output_name);print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',Path(a.directory)/a.output_name)
    return 0 if result['diagnostic_complete'] else 2
if __name__=='__main__':raise SystemExit(main())
