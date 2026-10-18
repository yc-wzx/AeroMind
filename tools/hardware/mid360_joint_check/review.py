"""Independent simultaneous LIO/encoder evidence review. Never publishes anything."""
import argparse,base64,json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360_gimbal_check'))
from analyze import STREAMS,CONTRACT,sha,pose_row,stamp_ns,imu_values,stream_stats
from audit import audit_rows
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
sys.path.insert(0,str(HERE.parent/'xuc_feedback_check'))
from inspect_wire import audit as serial_audit,PROFILES
from joint_math import derive,phase_bounds

def sources_ok(sources):
    return all(len(sources.get(k,[]))==1 and sources[k][0]['name']=='mid360_real_lio' and sources[k][0]['namespace']=='/hardware/mid360/lio' for k in ['corrected','predicted','mixed','tf']) and len(sources.get('imu_si',[]))==1 and sources['imu_si'][0]['name']=='mid360_imu_unit_adapter' and sources['imu_si'][0]['namespace']=='/'

def params_ok(p):
    import numpy as np
    expected={'use_sim_time':False,'common.odometry_output_contract':CONTRACT,'common.visualization_frame':'imu',
        'common.imu_frame':'mid360_imu','common.map_frame':'mid360_local_odom','common.time_sync_en':False}
    if not all(p.get(k)==v for k,v in expected.items()):return False
    try:
        return bool(np.allclose(p['mapping.extrinsic_T'],[-.011,-.02329,.04412],atol=1e-12,rtol=0) and
            np.allclose(p['mapping.extrinsic_R'],[1.,0.,0.,0.,1.,0.,0.,0.,1.],atol=1e-12,rtol=0))
    except (KeyError,ValueError,TypeError):return False

def inspect_native(records,phases):
    start,end=phase_bounds(phases);groups={k:[] for k in STREAMS};corrected=[];imu=[];lio=[];previous=0
    for i,r in enumerate(records,1):
        t=r.get('receive_monotonic_ns');kind=r.get('stream')
        if r.get('sequence')!=i or kind not in groups or type(t)!=int or t<previous:raise ValueError('Invalid native sequence/time/stream')
        previous=t;groups[kind].append(t)
        payload=base64.b64decode(r['cdr_base64'],validate=True)
        if kind=='imu_si':
            msg=deserialize_message(payload,Imu)
            if msg.header.frame_id!='mid360_imu':raise ValueError('Wrong IMU frame')
            values=imu_values(msg);imu.append(dict(stamp_ns=stamp_ns(msg),receive_monotonic_ns=t,receive_wall_ns=r['receive_wall_ns'],error=None,**values))
        else:
            lio.append(dict(r,sequence=len(lio)+1))
            if kind=='corrected':
                msg=deserialize_message(payload,Odometry)
                corrected.append(dict(sequence=r['sequence'],stamp_ns=stamp_ns(msg),receive_monotonic_ns=t,**pose_row(msg)))
    required=max(1.,(end-start)*1e-9-.5)
    audit=audit_rows(lio,required);audit.pop('corrected_drift',None)
    imu_stats=stream_stats(imu,required)
    coverage={}
    for kind,times in groups.items():
        times=[t for t in times if start<=t<=end];limit=.1 if kind=='imu_si' else .2
        coverage[kind]=dict(samples=len(times),limit_s=limit,pass_=bool(len(times)>=3 and times[0]-start<=limit*1e9 and end-times[-1]<=limit*1e9 and max((b-a for a,b in zip(times,times[1:])),default=10**12)<=limit*1e9))
    complete=audit['evidence_complete'] and imu_stats['pass_'] and imu_stats.get('max_receive_gap_s',math.inf)<=.1 and imu_stats.get('max_sensor_gap_s',math.inf)<=.1 and all(v['pass_'] for v in coverage.values())
    return dict(evidence_complete=bool(complete),lio_output_audit=audit,imu_stream=imu_stats,mission_coverage=coverage),corrected

def review(root):
    root=Path(root);meta=json.loads((root/'capture_meta.json').read_text())
    hashes=meta['data_sha256']
    required={'native.jsonl','mount_snapshot.json','variant_manifest_snapshot.json','local_inputs.json'}
    if not required<=set(hashes):raise ValueError('Missing mandatory data hashes')
    for name,value in hashes.items():
        target=(root/name).resolve()
        if not target.is_relative_to(root.resolve()) or sha(target)!=value:raise ValueError('Original hash mismatch: '+name)
    inputs=json.loads((root/'local_inputs.json').read_text())
    if not inputs:raise ValueError('Missing local input provenance')
    for item in inputs:
        if sha(root/item['snapshot'])!=item['sha256']:raise ValueError('Tool snapshot mismatch')
    records=[json.loads(s) for s in (root/'native.jsonl').read_text().splitlines()]
    h=meta['health'];gates=dict(writer_health=h['closed'] and h['error'] is None and h['overflow']==0 and h['received']==h['written']==h['actual_lines']==len(records),
        inputs_unchanged=meta['inputs_unchanged'],sealed_inputs_verified=meta['sealed_inputs_verified_before'] and meta['sealed_inputs_verified_after'],
        source_identity=sources_ok(meta['publishers_before']) and sources_ok(meta['publishers_after']),
        parameters=params_ok(meta['parameters_before']) and params_ok(meta['parameters_after']) and meta['parameters_before']==meta['parameters_after'],
        capture_finished_without_error=meta['failure'] is None and not meta['interrupted'],serial_process_exit=meta['serial_returncode']==0)
    result=dict(scope='SIMULTANEOUS REAL LIO / ENCODER RECEIVE-ONLY PROVISIONAL BODY PREVIEW',
        evidence_complete=False,motion_check_complete=False,compensation_accuracy_accepted=False,navigation_ready=False,
        published_ros_data=False,body_orientation_meaning='Change relative to operator-declared initial mechanical-forward frame in original LIO coordinates',
        gates=gates,capture_failure=meta['failure'],health=h,original_data_sha256=hashes,
        capture_meta_sha256=sha(root/'capture_meta.json'),reviewer_sha256=sha(Path(__file__)),math_sha256=sha(HERE/'joint_math.py'),
        timestamp_association='Nearest original host monotonic receive times, <=50ms filter; NO device-clock synchronization or delay correction',
        limits=['Fixed chassis and mechanical forward are operator declarations; no independent position/angle truth',
            'Mount dimensions are rough; no fitting or automatic encoder-zero configuration',
            'Positions are 3D in sensor-local LIO coordinates, not gravity/map-qualified',
            'Centre uses concentric fixed lever arm; live encoder is used for orientation, not to force centre stationary',
            'LIO output/transport delay and encoder measurement time are unknown; motion residual is not isolated calibration error',
            'No invented accuracy threshold, no motor/navigation/TF publication, no reset service monitoring'])
    timeline=[]
    try:
        encoder,frames=serial_audit(root/'encoder')
        if encoder['expected_crc_profile']!='0x8408':raise ValueError('Expected explicit CRC8408')
        result['encoder']=encoder
        serial_meta=json.loads((root/'encoder/capture_meta.json').read_text());phases=serial_meta['guided_phases']
        native,corrected=inspect_native(records,phases);result['native']=native
        expected=[f for f in sorted(frames,key=lambda f:f['offset']) if f['crc_algorithm']==PROFILES['0x8408']]
        stats,timeline=derive(corrected,expected,phases,json.loads((root/'mount_snapshot.json').read_text()))
        result['statistics']=stats
        holds=stats['hold_windows'];left=holds['turned_hold']['gimbal_heading_change_median_deg']-holds['baseline']['gimbal_heading_change_median_deg']
        back=holds['final_hold']['gimbal_heading_change_median_deg']-holds['turned_hold']['gimbal_heading_change_median_deg']
        result['lio_left_change_deg']=left;result['lio_return_change_deg']=back
        result['evidence_complete']=bool(all(gates.values()) and native['evidence_complete'] and encoder['capture_complete'] and encoder['receive_profile_matches'] and encoder['guided_turn_check']['evidence_complete'])
        result['motion_check_complete']=bool(result['evidence_complete'] and encoder['guided_turn_check']['motion_check_complete'] and left>=5 and back<=-5)
        result['status']='DIAGNOSTIC_COMPLETE_NO_ACCURACY_ACCEPTANCE' if result['motion_check_complete'] else 'INCOMPLETE_OR_MOTION_NOT_EXERCISED'
    except Exception as error:result['analysis_error']=str(error);result['status']='EVIDENCE_INCOMPLETE'
    return result,timeline

def save_review(root,name):
    root=Path(root)
    if Path(name).name!=name:raise ValueError('Use filename only')
    report=root/name;derived=root/(Path(name).stem+'.derived.jsonl')
    if report.exists() or derived.exists():raise FileExistsError('Preserving existing review')
    result,rows=review(root)
    with derived.open('x') as f:
        for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
    result['derived_timeline_sha256']=sha(derived)
    with report.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--output-name',default='independent_joint_review_v1.json');a=p.parse_args()
    result=save_review(a.directory,a.output_name);print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',Path(a.directory)/a.output_name)
    return 0 if result['motion_check_complete'] else 2
if __name__=='__main__':raise SystemExit(main())
