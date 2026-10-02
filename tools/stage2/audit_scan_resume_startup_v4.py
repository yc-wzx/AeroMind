#!/usr/bin/env python3
"""Sensor-first trial: require complete recorded startup and unchanged strict gates."""
import argparse
import json
import math
from pathlib import Path
import sys
import audit_scan_resume_v3 as v3
from audit_localized import load_jsonl, read, sha, finite_tree
from scan_capture_readiness import CaptureReadiness


def fully_timed_loss(rows):
    # This orchestration has /clock before localization is created. Therefore
    # it needs no untimed final-command/actuation startup exception at all.
    for r in rows:
        if r['topic'] not in ('/model/omni_robot/cmd_vel','/ground/planning/actuation_diagnostics'):continue
        t=r['message_stamp_s'] if r['message_stamp_s'] is not None else r['receive_sim_s']
        d=json.loads(r['data']['data']) if r['topic'].endswith('diagnostics') else r['data']
        if t is None or not isinstance(t,(int,float)) or not math.isfinite(t) or not finite_tree(d):
            raise ValueError('Sensor-first trial has missing/nonfinite final-output observation')
    result=v3.ORIGINAL_LOSS_AUDIT(rows)
    result['startup_clock_association']=dict(omitted_from_loss_window_only=0,original_native_rows_preserved=True,
        scope='Every final output and actuation diagnostic has real received clock association; no startup exception needed.')
    return result


def startup_audit(directory,rows,summary):
    files={n:read(directory/(n+'.json')) for n in ('recorder_constructed','world_start','sensors_capture_ready',
        'navigation_start','navigation_capture_ready','allow_goal','goal_release_capture_ready','before_publish_capture_ready')}
    checks={}
    sequence=[files[n]['monotonic_ns'] for n in ('recorder_constructed','world_start','sensors_capture_ready',
        'navigation_start','navigation_capture_ready','allow_goal','goal_release_capture_ready','before_publish_capture_ready')]
    checks['startup_sequence_before_unique_goal']=all(b>a for a,b in zip(sequence,sequence[1:])) and sequence[-1]<summary['goal_delivery']['publish_monotonic_ns']
    diagnostics=[r for r in rows if r['topic']=='/localization/diagnostics']
    checks['no_localization_diagnostic_before_sensor_readiness']=bool(diagnostics) and diagnostics[0]['receive_monotonic_ns']>files['navigation_start']['monotonic_ns']
    gate=CaptureReadiness();results={}
    for phase,label in [('sensors','sensors_capture_ready'),('goal','before_publish_capture_ready')]:
        mark=files[label];gate=CaptureReadiness()
        for r in rows:
            if r['receive_monotonic_ns']>=mark['monotonic_ns']:break
            gate.observe(r['topic'],r['data'],r['receive_monotonic_ns'],r['message_stamp_s'])
            gate.check(phase,r['receive_monotonic_ns'],mark['graph'])
        result=gate.check(phase,mark['monotonic_ns'],mark['graph'])
        # Raw receive replay proves observations; publisher snapshot duration is
        # attested by the sealed production gate and its recorded stable_s.
        checks[label+'_raw_data_ready']=all(result['checks'].values())
        checks[label+'_stable_gate_report']=mark.get('ready') is True and mark.get('stable_s',0)>=.5 and all(mark['checks'].values())
        results[label]=result
    return dict(pass_=all(checks.values()),checks=checks,markers=files,replayed_data_gates=results,
        scope='Raw message readiness plus recorded endpoint/GID/QoS snapshots and sealed gate implementation; not continuous packet-level graph monitoring or all-service reset monitoring.')


def audit(directory):
    previous=v3.clock_associated_loss;v3.clock_associated_loss=fully_timed_loss
    try:result=v3.audit(directory)
    finally:v3.clock_associated_loss=previous
    name=read(directory/'progress.json')['name'];rows=load_jsonl(directory/(name+'.native.jsonl'))
    result['startup_capture']=startup_audit(directory,rows,result['summary'])
    result['checks']['sensor_first_complete_capture']=result['startup_capture']['pass_']
    result['all_pass']=all(result['checks'].values());result['strict_trial_verified']=result['all_pass']
    result['live_scan_loss_guard_verified']=result['all_pass']
    result['evaluator_sha256']=sha(Path(__file__))
    result['evaluator_dependency_sha256']['tools/stage2/audit_scan_resume_v3.py']=sha(Path(v3.__file__))
    result['evaluator_dependency_sha256']['tools/stage2/scan_capture_readiness.py']=sha(Path(__file__).with_name('scan_capture_readiness.py'))
    result['command']=sys.argv
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    try:r=audit(a.result_dir)
    except (OSError,ValueError,KeyError,TypeError,IndexError) as error:r=dict(all_pass=False,status='INCOMPLETE_EVIDENCE',error=repr(error))
    a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:r.get(k) for k in ('all_pass','checks','loss_resume','error')},ensure_ascii=False))
    raise SystemExit(0 if r['all_pass'] else 2)
