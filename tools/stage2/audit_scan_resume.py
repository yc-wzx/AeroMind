#!/usr/bin/env python3
"""Independent short loss/resume evidence audit; no navigation publishing."""
import argparse
import bisect
import json
import math
from pathlib import Path
import sys
import audit_localized as common
from audit_localized import ROOT, read, sha, load_jsonl, yaw, finite_tree
from audit_localized_replay import replay
from stage2_wall_localization import Correction, validate_scan_metadata, verify_lidar_contract


def held_transform_audit(directory, name):
    """A correction remains unchanged between accepted scans, even during loss.

    Retain every old check except its +/- .22s nearest-scan assumption. Replace
    that assumption with direct reconstruction using the preceding accepted
    transform or a same-stamp neighbouring update (DDS delivery ordering).
    No missing/nonfinite corrected row is exempted during intentional loss.
    """
    result=ORIGINAL_LOCALIZATION_AUDIT(directory,name)
    rows=load_jsonl(directory/(name+'.native.jsonl'))
    raw={round(r['message_stamp_s'],8):r for r in rows if r['topic']=='/simulation/navigation_odometry'}
    accepted=[json.loads(r['data']['data']) for r in rows if r['topic']=='/localization/diagnostics'
              and json.loads(r['data']['data']).get('accepted') is True]
    stamps=[d['stamp_s'] for d in accepted]
    start=read(directory/(name+'.summary.json'))['goal_delivery']['publish_monotonic_ns']
    tested=[];held=0
    for row in rows:
        if row['topic']!='/localization/navigation_odometry' or row['receive_monotonic_ns']<start:continue
        t=row['message_stamp_s'];source=raw.get(round(t,8));i=bisect.bisect_right(stamps,t)
        if source is None or i==0:
            tested.append(False);continue
        candidates=[accepted[i-1]]
        # A new scan update can precede the odometry whose stamp is <=25ms older.
        if i<len(accepted) and stamps[i]-t<=.025:candidates.append(accepted[i])
        if i>=2 and t-stamps[i-1]<=.025:candidates.append(accepted[i-2])
        a=source['data']['pose']['pose'];p=row['data']['pose']['pose']
        observed=(p['position']['x'],p['position']['y'],yaw(p['orientation']))
        predictions=[]
        for d in candidates:
            c=Correction();c.offset=d['correction_transform']
            predictions.append(c.apply((a['position']['x'],a['position']['y'],yaw(a['orientation']))))
        tested.append(finite_tree(row['data']) and row['data']['twist']==source['data']['twist'] and
                      any(math.dist(observed,e)<1e-8 for e in predictions))
        held+=t-stamps[i-1]>.22
    result['checks']['corrected_pose_and_twist_reconstructable_from_raw']=bool(tested) and all(tested)
    result['pass']=all(result['checks'].values())
    result['held_transform_samples']=held
    result['association_limit']='Held preceding accepted transform reconstructed for each raw sample; neighbouring +/-25ms updates allow DDS delivery order. Not a generation-time proof.'
    return result


ORIGINAL_LOCALIZATION_AUDIT=common.localization_audit

def norm_command(r):
    d=r['data'];return math.hypot(d['linear']['x'],d['linear']['y']),abs(d['angular']['z'])

def gt_values(r):
    d=r['data'];p=d['pose']['pose'];v=d['twist']['twist']
    return [p['position']['x'],p['position']['y']],math.hypot(v['linear']['x'],v['linear']['y']),abs(v['angular']['z'])

def loss_audit(rows):
    topics={}
    for r in rows:topics.setdefault(r['topic'],[]).append(r)
    status=[(r,json.loads(r['data']['data'])) for r in topics.get('/scan_relay/status',[])]
    hold=[(r,d) for r,d in status if d['event']=='hold']
    resume=[(r,d) for r,d in status if d['event']=='resume']
    if len(hold)!=1 or len(resume)!=1:
        return {'pass':False,'status':'NOT_EXERCISED','reason':'Need exactly one raw relay hold and resume event'}
    hr,h=hold[0];rr,u=resume[0]
    hns,rns=h['monotonic_ns'],u['monotonic_ns'];ht,rt=h['sim_s'],u['sim_s']
    if not hns<rns or not ht<rt:raise ValueError('Hold/resume order invalid')
    scans=topics.get('/scan',[]);guarded=topics.get('/guarded_scan',[])
    original={round(r['message_stamp_s'],8):r for r in scans}
    diagnostics=[(r,json.loads(r['data']['data'])) for r in topics.get('/localization/diagnostics',[])]
    accepted=[(r,d) for r,d in diagnostics if d.get('accepted') is True]
    before=[(r,d) for r,d in accepted if d['stamp_s']<=h['last_forwarded_stamp_s']+1e-8]
    if not before:raise ValueError('Last accepted scan before hold missing')
    last_r,last=before[-1];last_stamp=last['stamp_s'];last_mono=last_r['receive_monotonic_ns']*1e-9
    after=[(r,d) for r,d in accepted if d['stamp_s']>h['last_forwarded_stamp_s']+1e-8 and r['receive_monotonic_ns']>=rns]
    if not after:raise ValueError('No new accepted scan after resume')
    first_r,first=after[0]
    commands=topics['/model/omni_robot/cmd_vel'];gt=topics['/gazebo/odometry']
    def time_of(r):return r['message_stamp_s'] if r['message_stamp_s'] is not None else r['receive_sim_s']
    def previous(stream,ns):
        candidates=[r for r in stream if r['receive_monotonic_ns']<=ns]
        if not candidates:raise ValueError('Pre-hold motion evidence missing')
        row=candidates[-1]
        if ns-row['receive_monotonic_ns']>200_000_000:raise ValueError('Stale pre-hold evidence')
        return row
    moving_gt=previous(gt,hns);moving_cmd=previous(commands,hns)
    hold_commands=[r for r in commands if hns<=r['receive_monotonic_ns']<=rns]
    last_nonzero=max((r['receive_monotonic_ns'] for r in hold_commands if norm_command(r)!=(0.,0.)),default=hns-1)
    candidates=[r for r in hold_commands if r['receive_monotonic_ns']>last_nonzero and norm_command(r)==(0.,0.)]
    if not candidates:raise ValueError('No exact zero final command after hold')
    stopped_cmd=candidates[0];ct=time_of(stopped_cmd)
    stopped_gt=[r for r in gt if time_of(r)>=ct and gt_values(r)[1]<=.02 and gt_values(r)[2]<=.03]
    if not stopped_gt:raise ValueError('GT did not stop')
    sg=stopped_gt[0];st=time_of(sg)
    hold_gt=[r for r in gt if st<=time_of(r)<=rt]
    hold_cmd=[r for r in commands if ct<=time_of(r)<=rt]
    anchor=gt_values(sg)[0]
    motion=[r for r in gt if ht<=time_of(r)<=st]
    stop_displacement=max((math.dist(gt_values(r)[0],gt_values(moving_gt)[0]) for r in motion),default=math.inf)
    drift=max((math.dist(gt_values(r)[0],anchor) for r in hold_gt),default=math.inf)
    # Control timer: 20ms. Observed command/clock delivery permits a two-period
    # association budget; report latency separately in sim and receive clocks.
    period=.02
    cutoff=last_stamp+.5+2*period
    late_commands=[r for r in commands if cutoff<=time_of(r)<first['stamp_s']]
    pre_resume_move=[r for r in commands if rt<=time_of(r)<first['stamp_s'] and norm_command(r)!=(0.,0.)]
    resumed_motion=[r for r in commands if r['receive_monotonic_ns']>=first_r['receive_monotonic_ns'] and norm_command(r)[0]>.03]
    guard_index={round(r['message_stamp_s'],8):r for r in guarded}
    scan_valid=[]
    from types import SimpleNamespace
    for r in guarded:
        source=original.get(round(r['message_stamp_s'],8))
        validate_scan_metadata(SimpleNamespace(**r['data']))
        scan_valid.append(source is not None and source['data']==r['data'] and
                          r['data']['header']['frame_id']=='omni_robot/base_link/lidar')
    in_hold=[r for r in guarded if r['message_stamp_s']>h['last_forwarded_stamp_s']+1e-8 and r['message_stamp_s']<rt-.02]
    guarded_stamps=[r['message_stamp_s'] for r in guarded]
    completed=[r for r in topics.get('/rosout/relevant',[]) if 'RMUC waypoint diagnostic completed' in r['data'].get('msg','') and hns<=r['receive_monotonic_ns']<=rns]
    stale_diags=[r for r in topics.get('/ground/planning/actuation_diagnostics',[]) if ht+.55<=time_of(r)<=rt and
                 json.loads(r['data']['data']).get('action')=='stale_stop']
    hold_original=[r for r in scans if ht+.1<=r['message_stamp_s']<=rt-.1]
    def continuous(stream,low,high,maxgap,strict):
        selected=[r for r in stream if low<=time_of(r)<=high]
        if len(selected)<2:return False
        stamps=[time_of(r) for r in selected];receives=[r['receive_monotonic_ns']*1e-9 for r in selected]
        return (stamps[0]<=low+maxgap and stamps[-1]>=high-maxgap and
                all((0< b-a if strict else 0<=b-a) and b-a<=maxgap for a,b in zip(stamps,stamps[1:])) and
                all(0<b-a<=maxgap for a,b in zip(receives,receives[1:])))
    checks={
      'single_hold_resume_in_order':h['held'] is True and u['held'] is False,
      'moving_before_hold':gt_values(moving_gt)[1]>.03 and norm_command(moving_cmd)[0]>.03 and
           math.dist(gt_values(moving_gt)[0],[4.7,1.15])>=.35,
      'relay_forwarded_unchanged_no_replay':bool(scan_valid) and all(scan_valid) and
           len(set(guarded_stamps))==len(guarded_stamps) and all(b>a for a,b in zip(guarded_stamps,guarded_stamps[1:])) and not in_hold,
      'original_sensor_kept_publishing':len(hold_original)>=10 and u['dropped']>h['dropped'],
      'relay_counter_invariants':all(d['received']==d['forwarded']+d['dropped'] for _,d in status),
      'every_accepted_scan_was_forwarded':all(round(d['stamp_s'],8) in guard_index for _,d in accepted),
      'stop_within_timeout_plus_two_control_periods':ct<=cutoff and (stopped_cmd['receive_monotonic_ns']*1e-9-last_mono)<=.54,
      'exact_zero_until_new_scan':bool(late_commands) and all(norm_command(r)==(0.,0.) for r in late_commands) and not pre_resume_move,
      'hold_gt_stable_at_least_2s':rt-st>=2 and len(hold_gt)>50 and
           max(gt_values(r)[1] for r in hold_gt)<=.02 and max(gt_values(r)[2] for r in hold_gt)<=.03 and drift<=.03,
      'hold_final_commands_stopped':bool(hold_cmd) and all(norm_command(r)==(0.,0.) for r in hold_cmd),
      'hold_gt_command_continuity':continuous(gt,st,rt,.2,True) and continuous(commands,st,rt,.2,False),
      'same_task_not_completed_during_hold':not completed and bool(stale_diags),
      'fresh_scan_and_motion_after_resume':first['stamp_s']>=rt-.02 and bool(resumed_motion),
    }
    return {'pass':all(checks.values()),'checks':checks,'hold_sim_s':ht,'resume_sim_s':rt,
      'last_accepted_scan_sim_s':last_stamp,'final_zero_sim_s':ct,'gt_stop_sim_s':st,
      'scan_to_zero_sim_s':ct-last_stamp,'scan_to_zero_receive_s':stopped_cmd['receive_monotonic_ns']*1e-9-last_mono,
      'scan_to_gt_stop_sim_s':st-last_stamp,'hold_to_gt_stop_sim_s':st-ht,
      'stop_displacement_m':stop_displacement,'stable_hold_duration_sim_s':rt-st,'hold_max_drift_m':drift,
      'first_resumed_scan_sim_s':first['stamp_s'],'new_motion_sim_s':time_of(resumed_motion[0]) if resumed_motion else None,
      'relay_hold':h,'relay_resume':u,'stale_stop_diagnostics':len(stale_diags),
      'time_limit':'0.5s existing match timeout plus two 20ms control periods for independent receipt association; both clocks reported. Not a generation-time latency proof.'}


def audit(directory):
    # Scoped replacement only for the new loss test's held-transform semantics.
    original=common.localization_audit
    common.localization_audit=held_transform_audit
    try:result=common.audit(directory)
    finally:common.localization_audit=original
    name=read(directory/'progress.json')['name']
    rows=load_jsonl(directory/(name+'.native.jsonl'))
    result['loss_resume']=loss_audit(rows)
    result['replay']=replay(directory,name)
    graph=read(directory/'scan_graph.json')
    result['checks']['loss_resume_raw_evidence']=result['loss_resume']['pass']
    result['checks']['accepted_scans_replayed']=result['replay']['pass']
    ep=graph['endpoints'];names=lambda t,k:[e['name'] for e in ep[t+':'+k]]
    result['checks']['scan_graph_isolated']=(graph.get('stable_s',0)>=1 and
       names('/scan','pub')==['gazebo_bridge'] and 'stage2_scan_relay' in names('/scan','sub') and
       'gazebo_navigation_interface' not in names('/scan','sub') and
       names('/guarded_scan','pub')==['stage2_scan_relay'] and
       'gazebo_navigation_interface' in names('/guarded_scan','sub') and
       all(any(e['gid']) and e['qos'] for v in ep.values() for e in v))
    result['checks']['short_goal_only']=result['summary']['goal_xy']==[4.7,1.15]
    result['all_pass']=all(result['checks'].values())
    result['live_scan_loss_guard_verified']=result['all_pass']
    result['hard_invalid_frame_live_tested']=False
    result['evaluator_sha256']=sha(Path(__file__))
    result['evaluator_dependency_sha256']={str(p.relative_to(ROOT)):sha(p) for p in
         (Path(common.__file__),Path(__file__).with_name('audit_localized_replay.py'),
          Path(__file__).with_name('stage2_terminal_evidence.py'),ROOT/'src/uav_planning/scripts/stage2_wall_localization.py')}
    result['command']=sys.argv
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    try:r=audit(a.result_dir)
    except (OSError,ValueError,KeyError,TypeError,IndexError) as error:
        r={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':repr(error)}
    a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'all_pass':r['all_pass'],'checks':r.get('checks'),'loss_resume':r.get('loss_resume'),'error':r.get('error')},ensure_ascii=False))
    sys.exit(0 if r['all_pass'] else 2)
