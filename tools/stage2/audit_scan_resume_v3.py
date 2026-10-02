#!/usr/bin/env python3
"""Read-only loss-window clock association and actual task-clock evidence.

The full native audit always receives every original row. Only the loss-window
subcheck omits untimed Twist receipts before the recorder's first /clock. No
field is filled, and any untimed receipt after that first clock is an error.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import audit_scan_resume as loss_v1
from audit_scan_resume_v2 import audit as audit_v2
from audit_localized import read, sha, load_jsonl, finite_tree, ROOT

ORIGINAL_LOSS_AUDIT = loss_v1.loss_audit
FINAL_TOPIC = '/model/omni_robot/cmd_vel'


def clock_associated_loss(rows):
    clocks = [r for r in rows if r['topic'] == '/clock']
    if not clocks:
        raise ValueError('No original /clock receipt')
    first_clock = clocks[0]['receive_monotonic_ns']
    commands = [r for r in rows if r['topic'] == FINAL_TOPIC]
    skipped = []
    seen_timed = False
    for r in commands:
        t = r['message_stamp_s'] if r['message_stamp_s'] is not None else r['receive_sim_s']
        if not finite_tree(r['data']):
            raise ValueError('Nonfinite original final command')
        if t is None:
            if seen_timed or r['receive_monotonic_ns'] >= first_clock:
                raise ValueError('Untimed final command after first /clock')
            skipped.append(r)
        else:
            if not isinstance(t, (int, float)) or not math.isfinite(t):
                raise ValueError('Nonfinite final command time')
            seen_timed = True
    first_timed = next((r for r in commands if r['receive_sim_s'] is not None), None)
    if first_timed is None or not 0 <= (first_timed['receive_monotonic_ns'] - first_clock)*1e-9 <= .2:
        raise ValueError('No fresh timed final command after first /clock')
    # String actuation diagnostics also have no message header. Their initial
    # pre-clock receipts must not be mistaken for loss-window observations.
    untimed_diagnostics = []
    timed_diagnostic_seen = False
    for r in rows:
        if r['topic'] != '/ground/planning/actuation_diagnostics':continue
        if not finite_tree(json.loads(r['data']['data'])):
            raise ValueError('Nonfinite original actuation diagnostic')
        t = r['message_stamp_s'] if r['message_stamp_s'] is not None else r['receive_sim_s']
        if t is None:
            if timed_diagnostic_seen or r['receive_monotonic_ns'] >= first_clock:
                raise ValueError('Untimed actuation diagnostic after first /clock')
            untimed_diagnostics.append(r)
        else:
            if not isinstance(t,(int,float)) or not math.isfinite(t):
                raise ValueError('Nonfinite actuation diagnostic time')
            timed_diagnostic_seen = True
    omitted = {(r['topic'],r['receive_monotonic_ns']) for r in skipped+untimed_diagnostics}
    scoped = [r for r in rows if (r['topic'],r['receive_monotonic_ns']) not in omitted]
    result = ORIGINAL_LOSS_AUDIT(scoped)
    result['startup_clock_association'] = dict(
        omitted_from_loss_window_only=len(skipped), first_clock_receive_monotonic_ns=first_clock,
        pre_clock_actuation_diagnostics=len(untimed_diagnostics),
        last_untimed_receive_monotonic_ns=skipped[-1]['receive_monotonic_ns'] if skipped else None,
        first_timed_command_receive_monotonic_ns=first_timed['receive_monotonic_ns'],
        original_native_rows_preserved=True,
        scope='No message values missing: recorder had not yet received /clock. Full native audit retains all original rows; no timestamps fabricated.')
    return result


def task_clock_audit(rows, summary, loss):
    ds = [(r, json.loads(r['data']['data'])) for r in rows if r['topic']=='/localization/diagnostics']
    events = [(r,d) for r,d in ds if d.get('event')=='localization_task_clock']
    phases = [d.get('phase') for _,d in events]
    final = summary['final_waypoint_id']
    route = final.split(':')[0]
    checks = {'one_pause_then_resume': phases == ['paused','resumed']}
    if not checks['one_pause_then_resume']:
        return dict(pass_=False, checks=checks, events=[d for _,d in events])
    pr,p = events[0]; rr,u = events[1]
    checks.update(
        same_actual_route_waypoint=all(d.get('route')==route and d.get('waypoint')==final for _,d in events),
        diagnostic_not_accepted_scan=all('accepted' not in d and 'stamp_s' not in d for _,d in events),
        finite_diagnostic_times=all(finite_tree(d) and all(isinstance(d.get(k),(int,float)) and math.isfinite(d[k])
            for k in ('monotonic_s','sim_s','hold_duration_s','stage_sent_at_before_hold','progress_at_before_hold')) for _,d in events),
        pause_after_timeout_before_resume=loss['last_accepted_scan_sim_s']+.5 <= p['sim_s'] <= loss['resume_sim_s'],
        resume_after_new_scan=u['sim_s'] >= loss['first_resumed_scan_sim_s'] and u['sim_s'] > p['sim_s'],
        receive_order=pr['receive_monotonic_ns'] < rr['receive_monotonic_ns'],
        hold_duration_recomputes=p['hold_duration_s']==0 and u['hold_duration_s']>0 and
            abs(u['hold_duration_s']-(u['monotonic_s']-p['monotonic_s']))<1e-8,
        pre_hold_clocks_consistent=all(p[k]==u[k] and p[k]<=p['monotonic_s'] for k in
            ('stage_sent_at_before_hold','progress_at_before_hold')),
        no_retry_stuck_false_success=summary['retry_count']==summary['stuck_count']==0 and summary['false_success'] is False,
    )
    goals = [r for r in rows if r['topic']=='/ground/planning/goal']
    logs = [r for r in rows if r['topic']=='/rosout/relevant']
    checks['one_original_internal_goal_no_resend'] = len(goals)==1 and goals[0]['receive_monotonic_ns']<pr['receive_monotonic_ns'] and not any(
        'Retrying RMUC stage' in r['data'].get('msg','') for r in logs)
    completed = [r for r in logs if 'RMUC waypoint diagnostic completed' in r['data'].get('msg','')]
    checks['matching_completion_after_resume'] = len(completed)==1 and completed[0]['receive_monotonic_ns']>rr['receive_monotonic_ns'] and (
        'route='+route+' waypoint='+final) in completed[0]['data']['msg']
    return dict(pass_=all(checks.values()),checks=checks,events=[d for _,d in events],
        scope='Actual raw diagnostic transitions and unchanged task evidence; private timer values after resume are supported by sealed runtime source and isolated production tests, not independently subscribed state.')


def audit(directory):
    previous = loss_v1.loss_audit
    loss_v1.loss_audit = clock_associated_loss
    try:
        result = audit_v2(directory)
    finally:
        loss_v1.loss_audit = previous
    name = read(directory/'progress.json')['name']
    rows = load_jsonl(directory/(name+'.native.jsonl'))
    result['task_clock'] = task_clock_audit(rows,result['summary'],result['loss_resume'])
    result['checks']['actual_localization_task_clock'] = result['task_clock']['pass_']
    diagnostics = [json.loads(r['data']['data']) for r in rows if r['topic']=='/localization/diagnostics']
    scans = [d for d in diagnostics if isinstance(d.get('accepted'),bool)]
    result['localization']['rejected_scans'] = sum(d['accepted'] is False for d in scans)
    result['localization']['non_scan_diagnostics'] = len(diagnostics)-len(scans)
    result['all_pass'] = all(result['checks'].values())
    result['strict_trial_verified'] = result['all_pass']
    result['evaluator_sha256'] = sha(Path(__file__))
    result['evaluator_dependency_sha256'].update({str(Path(p).relative_to(ROOT)):sha(Path(p)) for p in
        (loss_v1.__file__,str(Path(__file__).with_name('audit_scan_resume_v2.py')))})
    result['correction_note_v3'] = 'Only loss-window clock association and task-clock evidence added; full raw/native/terminal/retry/geometry thresholds retained. v1/v2 error reports and original evidence unchanged.'
    runtime_file = directory/'input_snapshot/tools/stage2/audit_scan_resume_v2.py'
    result['current_evaluator_not_runtime_input'] = dict(
        v2_current_matches_runtime_snapshot=runtime_file.exists() and sha(runtime_file)==sha(Path(__file__).with_name('audit_scan_resume_v2.py')),
        v3='Post-run independent evaluator; never claimed as runtime input.')
    result['command'] = sys.argv
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    try:r=audit(a.result_dir)
    except (OSError,ValueError,KeyError,TypeError,IndexError) as error:r={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':repr(error)}
    a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:r.get(k) for k in ('all_pass','checks','loss_resume','task_clock','error')},ensure_ascii=False))
    sys.exit(0 if r['all_pass'] else 2)
