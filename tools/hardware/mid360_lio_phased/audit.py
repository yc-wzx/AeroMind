"""Independent native-message audit for corrected / predicted / mixed output."""
import argparse,base64,collections,json,math,sys
from pathlib import Path
from variant import HERE,ROOT,CONTRACT,sha
sys.path.insert(0,str(HERE.parent/'mid360_lio'))
from static_check import pose_row,drift_stats,stream_stats,stamp_ns
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
TOPICS={'corrected':'/hardware/mid360/lio/odometry',
        'predicted':'/hardware/mid360/lio/odometry_predicted',
        'mixed':'/hardware/mid360/lio/odometry_mixed_legacy','tf':'/tf'}

def identity(row,pose_only=False):
    keys=['stamp_ns','position','quaternion']
    if not pose_only:keys+=['velocity','pose_covariance','twist_covariance']
    return json.dumps({k:row[k] for k in keys},sort_keys=True,allow_nan=False,separators=(',',':'))

def audit_rows(records,required_span_s):
    groups={kind:[] for kind in TOPICS};errors=[];last_receive=0
    for i,raw in enumerate(records,1):
        kind=raw.get('stream')
        if raw.get('sequence')!=i or kind not in groups:raise ValueError('Bad native sequence/stream')
        receive=raw.get('receive_monotonic_ns')
        if not isinstance(receive,int) or receive<last_receive:raise ValueError('Native receipt order invalid')
        last_receive=receive
        decoded=dict(receive_monotonic_ns=receive,receive_wall_ns=raw['receive_wall_ns'],stamp_ns=None,error=None)
        try:
            payload=base64.b64decode(raw['cdr_base64'],validate=True)
            if kind=='tf':
                message=deserialize_message(payload,TFMessage)
                selected=[t for t in message.transforms if t.header.frame_id=='mid360_local_odom' and t.child_frame_id=='mid360_imu']
                if not selected:continue
                if len(selected)!=1:raise ValueError('Duplicate target-frame transforms in one TF message')
                transform=selected[0];p=transform.transform.translation;q=transform.transform.rotation
                decoded.update(stamp_ns=stamp_ns(transform),position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w])
                if not all(math.isfinite(v) for v in decoded['position']+decoded['quaternion']) or abs(sum(v*v for v in decoded['quaternion'])-1)>1e-3:
                    raise ValueError('Invalid TF pose')
            else:
                message=deserialize_message(payload,Odometry)
                decoded['stamp_ns']=stamp_ns(message);decoded.update(pose_row(message))
        except Exception as error:
            decoded['error']=str(error);errors.append(dict(sequence=i,stream=kind,error=str(error)))
        groups[kind].append(decoded)
    stats={k:stream_stats(v,required_span_s) for k,v in groups.items()}
    ordered_ok=all(stats[k]['pass_'] and stats[k].get('max_receive_gap_s',math.inf)<=.2 and
                   stats[k].get('max_sensor_gap_s',math.inf)<=.2 for k in ['corrected','predicted','tf'])
    mixed=stats['mixed'];mixed_integrity=(not errors and mixed.get('errors')==0 and
        mixed.get('observed_span_s',0)>=required_span_s and mixed.get('max_receive_gap_s',math.inf)<=.2)
    coverage=dict(pass_=False,reason='Missing/invalid streams')
    if not errors and all(groups.values()):
        low=max(min(r['stamp_ns'] for r in v) for v in groups.values())
        high=min(max(r['stamp_ns'] for r in v) for v in groups.values())
        interior={k:[r for r in v if low<=r['stamp_ns']<=high] for k,v in groups.items()}
        counter=lambda name,pose=False:collections.Counter(identity(r,pose) for r in interior[name])
        phase_union=counter('corrected')+counter('predicted');legacy=counter('mixed')
        tf_match=counter('corrected',True)==counter('tf',True)
        complete_span=(high-low)*1e-9>=required_span_s
        coverage=dict(pass_=complete_span and all(interior.values()) and phase_union==legacy and tf_match,
            common_sensor_start_ns=low,common_sensor_end_ns=high,common_sensor_span_s=(high-low)*1e-9,
            interior_counts={k:len(v) for k,v in interior.items()},
            boundary_records_retained_but_excluded_from_identity_comparison={k:len(groups[k])-len(v) for k,v in interior.items()},
            missing_from_mixed=sum((phase_union-legacy).values()),missing_from_split=sum((legacy-phase_union).values()),
            tf_matches_corrected_pose=tf_match,
            rule='Exact decoded message identity multisets within common device-time coverage; all boundary messages remain recorded')
    good=[r for r in groups['corrected'] if r['error'] is None]
    return dict(evidence_complete=bool(ordered_ok and mixed_integrity and coverage['pass_'] and not errors),
                streams=stats,phase_identity=coverage,parse_errors=errors,
                corrected_drift=drift_stats(good),
                zero_twist_is_not_a_stationarity_measurement=True,
                legacy_mixed_order_gate_is_diagnostic_only=True)

def audit(directory):
    root=Path(directory);meta=json.loads((root/'capture_meta.json').read_text())
    raw=root/'native.jsonl'
    if sha(raw)!=meta['data_sha256']['native.jsonl']:raise ValueError('Native hash mismatch')
    snapshot=root/'variant_manifest_snapshot.json'
    if sha(snapshot)!=meta['variant_manifest_sha256']:raise ValueError('Input manifest snapshot hash mismatch')
    rows=[json.loads(line) for line in raw.read_text().splitlines()]
    result=audit_rows(rows,max(1.,meta['seconds']-.5))
    h=meta['health']
    health_ok=h['closed'] and h['error'] is None and h['overflow']==0 and h['received']==h['written']==h['actual_lines']==len(rows)
    sources=meta['publishers']
    source_ok=all(len(sources.get(k,[]))==1 and sources[k][0]['name']=='mid360_real_lio' and sources[k][0]['namespace']=='/hardware/mid360/lio' for k in ['corrected','predicted','mixed'])
    p=meta['actual_parameters']
    params_ok=(p.get('common.odometry_output_contract')==CONTRACT and p.get('common.visualization_frame')=='imu' and
               p.get('common.imu_frame')=='mid360_imu' and p.get('use_sim_time') is False)
    result.update(scope='REAL SENSOR-LOCAL PHASED LIO STATIC CHECK; NOT NAVIGATION ACCEPTANCE',
        original_data_sha256=meta['data_sha256'],health=h,publishers=sources,actual_parameters=p,
        input_manifest_sha256=meta['variant_manifest_sha256'],inputs_unchanged=meta['inputs_unchanged'],
        gates=dict(writer_health=health_ok,source_identity=source_ok,parameters=params_ok,
                   inputs_unchanged=meta['inputs_unchanged'],not_interrupted=not meta['interrupted']),
        limits=['Stationarity is operator-declared; no ground truth','No chassis-centre/gravity alignment qualification',
                'No invented drift pass threshold','No motors or navigation output','No claim of publisher-side losslessness'])
    result['evidence_complete']=bool(result['evidence_complete'] and all(result['gates'].values()))
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('result_dir');parser.add_argument('--output-name',default='reaudit.json');args=parser.parse_args()
    if Path(args.output_name).name!=args.output_name:parser.error('output-name must be a filename')
    out=Path(args.result_dir)/args.output_name
    result=audit(args.result_dir)
    with out.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',out)
    raise SystemExit(0 if result['evidence_complete'] else 2)
