"""Capture original split LIO/IMU messages; independent static gross-sanity review."""
import argparse,base64,hashlib,json,math,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360_translation_check'))
from translation_review import sources_ok,params_ok,sha,audit_rows,stream_stats,imu_values,stamp_ns,pose_row,deserialize_message,Odometry,Imu
from healthy_recorder import HealthyRecorder
from input_health import PoseHealth,STATIC_SPAN_M,STATIC_ANGLE_DEG
sys.path.insert(0,str(HERE.parent/'mid360_lio_phased'))
from variant import verify,MANIFEST

def audit(root,output_name='independent_static_review_v1.json'):
    root=Path(root);out=root/output_name
    if out.exists():raise FileExistsError(out)
    meta=json.loads((root/'capture_meta.json').read_text())
    for name,value in meta['data_sha256'].items():
        p=(root/name).resolve()
        if not p.is_relative_to(root.resolve()) or sha(p)!=value:raise ValueError('Raw hash mismatch: '+name)
    rows=[json.loads(x) for x in (root/'native.jsonl').read_text().splitlines()]
    lio=[];imus=[];corrected=[];last=0
    for i,r in enumerate(rows,1):
        if r['sequence']!=i or r['receive_monotonic_ns']<last:raise ValueError('Original sequence/time invalid')
        last=r['receive_monotonic_ns']
        if r['stream']=='imu_si':
            m=deserialize_message(base64.b64decode(r['cdr_base64']),Imu)
            imus.append(dict(stamp_ns=stamp_ns(m),receive_monotonic_ns=last,error=None,**imu_values(m)))
        else:
            lio.append(dict(r,sequence=len(lio)+1))
            if r['stream']=='corrected':
                m=deserialize_message(base64.b64decode(r['cdr_base64']),Odometry)
                corrected.append(dict(stamp_ns=stamp_ns(m),receive_monotonic_ns=last,**pose_row(m)))
    native=audit_rows(lio,max(1.,meta['seconds']-.5))
    imustats=stream_stats(imus,max(1.,meta['seconds']-.5))
    health=PoseHealth()
    for r in corrected:health.accept(r['stamp_ns'],r['receive_monotonic_ns'],r['position'],r['quaternion'])
    distance=max((math.dist(r['position'],corrected[0]['position']) for r in corrected),default=None)
    angle=max((math.degrees(2*math.acos(min(1.,abs(sum(a*b for a,b in zip(r['quaternion'],corrected[0]['quaternion'])))))) for r in corrected),default=None)
    w=meta['writer_health']
    gates=dict(writer=w['closed'] and w['overflow']==0 and w['error'] is None and w['received']==w['written']==w['actual_lines']==len(rows),
        raw_lio_audit=native['evidence_complete'],imu=imustats['pass_'] and imustats.get('max_receive_gap_s',math.inf)<=.1 and imustats.get('max_sensor_gap_s',math.inf)<=.1,
        sources=sources_ok(meta['sources_before']) and sources_ok(meta['sources_after']),
        parameters=params_ok(meta['parameters_before']) and meta['parameters_before']==meta['parameters_after'],
        inputs=meta['inputs_unchanged'] and meta['sealed_before'] and meta['sealed_after'],
        no_capture_failure=meta['failure'] is None)
    sanity=bool(corrected and health.failure is None and health.static_report()['pass_'] and distance<=STATIC_SPAN_M and angle<=STATIC_ANGLE_DEG)
    result=dict(scope='REAL STATIONARY LIO RECOVERY DIAGNOSTIC; NOT ACCURACY/NAVIGATION ACCEPTANCE',
        evidence_complete=all(gates.values()),static_recovery_check_pass=bool(all(gates.values()) and sanity),
        accuracy_accepted=False,navigation_ready=False,gates=gates,capture_failure=meta['failure'],
        native=native,imu=imustats,pose_sanity_failure=health.failure,last_five_seconds=health.static_report(),
        full_capture_max_displacement_from_first_m=distance,full_capture_max_rotation_from_first_deg=angle,
        endpoint_delta_m=[corrected[-1]['position'][i]-corrected[0]['position'][i] for i in range(3)] if corrected else None,
        raw_samples=len(rows),original_data_sha256=meta['data_sha256'],capture_meta_sha256=sha(root/'capture_meta.json'),
        reviewer_sha256=sha(Path(__file__)),engineering_limits=dict(position_span_m=STATIC_SPAN_M,angle_deg=STATIC_ANGLE_DEG),
        limitations=['Stationarity operator-declared; no position truth','Gross-sanity gates are not accuracy thresholds',
                     'No navigation/motor integration or statistical repeatability qualification'])
    out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');return result

def capture(root,seconds):
    root=Path(root);root.mkdir(parents=True,exist_ok=False)
    verify();before_manifest=sha(MANIFEST);sealed_before=True
    files=[f for f in HERE.glob('*') if f.is_file() and f.suffix in ['.py','.md','.sh']]
    snapshot=root/'input_snapshot';snapshot.mkdir()
    for f in files:(snapshot/f.name).write_bytes(f.read_bytes())
    inputs={str(f):sha(f) for f in files}
    (root/'local_inputs.json').write_text(json.dumps(inputs,indent=2)+'\n')
    ros=HealthyRecorder(root/'native.jsonl');failure=None;before={};after={};sources_before={};sources_after={};health={}
    try:
        deadline=time.monotonic()+10
        while not ros.fresh():
            if ros.error or ros.pose_health.failure:raise RuntimeError(ros.error or ros.pose_health.failure)
            if time.monotonic()>deadline:raise RuntimeError('LIO inputs not ready')
            time.sleep(.02)
        sources_before=ros.publishers();before=ros.parameters()
        if not sources_ok(sources_before) or not params_ok(before):raise RuntimeError('Source/parameter precheck failed')
        print('Stationary capture: '+str(seconds)+' seconds; no commands.',flush=True)
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if not ros.fresh():raise RuntimeError(ros.pose_health.failure or 'Input became stale')
            time.sleep(.02)
        sources_after=ros.publishers();after=ros.parameters()
    except KeyboardInterrupt:failure='Operator interrupted'
    except Exception as error:failure=str(error)
    finally:health=ros.close()
    try:verify();sealed_after=before_manifest==sha(MANIFEST)
    except Exception:sealed_after=False
    meta=dict(seconds=seconds,failure=failure,writer_health=health,sources_before=sources_before,sources_after=sources_after,
        parameters_before=before,parameters_after=after,sealed_before=sealed_before,sealed_after=sealed_after,
        inputs_unchanged=all(sha(f)==inputs[str(f)] for f in files),
        operator_declaration='User confirmed chassis/gimbal stationary and released before restart and capture',
        data_sha256={n:sha(root/n) for n in ['native.jsonl','local_inputs.json']},
        no_motion_commands=True,sealed_variant_manifest_sha256=before_manifest)
    (root/'capture_meta.json').write_text(json.dumps(meta,indent=2)+'\n')
    try:result=audit(root)
    except Exception as error:
        result=dict(evidence_complete=False,static_recovery_check_pass=False,accuracy_accepted=False,navigation_ready=False,
                    capture_failure=failure,analysis_error=str(error),original_data_sha256=meta['data_sha256'])
        with (root/'independent_static_review_v1.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2));print('SUMMARY:',root/'independent_static_review_v1.json',flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=30);p.add_argument('--output-dir');p.add_argument('--review-dir');p.add_argument('--output-name',default='independent_static_review_v2.json');a=p.parse_args()
    if a.review_dir:r=audit(a.review_dir,a.output_name)
    else:
        if not 10<=a.seconds<=60:raise ValueError('Static capture must be 10..60 seconds')
        path=a.output_dir or str(Path.home()/'AeroMind_mid360_guard_checks'/('static_'+time.strftime('%Y%m%d_%H%M%S')))
        r=capture(path,a.seconds)
    return 0 if r['static_recovery_check_pass'] else 2
if __name__=='__main__':raise SystemExit(main())
