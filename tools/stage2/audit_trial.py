#!/usr/bin/env python3
"""Read-only Stage 2 evidence audit; completion is not a process return code."""
import argparse
import hashlib
import json
import math
import random
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from stage2_terminal_evidence import stage2_native_terminal_audit
from summarize_provincial_stage15_terminal_v3 import audit_run, sdf_walls, load_jsonl
from summarize_provincial_forward_integration import raw_waypoint_sequence
from summarize_provincial_full_observed import native_audit, native_terminal_audit, yaw, finite_tree
from analyze_provincial_forward_clearance import walls_from_sdf


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p): return json.loads(p.read_text())

def stage2_waypoint_sequence(directory, name, summary, preflight):
    """Keep original route/order rules; associate logged pose with actual odom.

    Enrichment exists only in memory. Original raw GT remains untouched. This
    calls the frozen checker with its pose observation supplied from nav odom,
    since that is the coordinate source of the production completion log.
    """
    import copy
    events=copy.deepcopy(load_jsonl(directory/(name+'.events.jsonl')))
    associations={};latest_nav=None;latest_gt=None
    for row in load_jsonl(directory/(name+'.native.jsonl')):
        if row['topic']=='/ground/odometry':latest_nav=row
        elif row['topic']=='/gazebo/odometry':latest_gt=row
        elif row['topic']=='/rosout/relevant':
            msg=row['data'].get('msg','')
            if 'RMUC waypoint diagnostic completed' not in msg:continue
            valid=(latest_nav is not None and latest_gt is not None and
                0 <= (row['receive_monotonic_ns']-latest_nav['receive_monotonic_ns'])*1e-9<=.5 and
                0 <= (row['receive_monotonic_ns']-latest_gt['receive_monotonic_ns'])*1e-9<=.2 and
                abs(latest_nav['message_stamp_s']-latest_gt['message_stamp_s'])<=.2 and
                finite_tree(latest_nav['data']))
            if not valid:return {'pass':False,'reason':'missing/stale native pose at waypoint event'}
            p=latest_nav['data']['pose']['pose']
            associations.setdefault(msg,[]).append([p['position']['x'],p['position']['y'],yaw(p['orientation'])])
    for event in events:
        if 'RMUC waypoint diagnostic completed' not in event.get('message',''):continue
        found=associations.get(event['message'])
        if not found:return {'pass':False,'reason':'CSV event not found in native waypoint evidence'}
        event['gt_pose']=found.pop(0)  # compatibility field ONLY in in-memory copy
    result=raw_waypoint_sequence(events,summary,preflight)
    result['completion_pose_source']='raw /ground/odometry, not ground truth; original events unchanged'
    return result


def model_steps(pairs, diagnostics, parameters):
    """Independent recurrence from raw truth/nav pairs and recorded step count.

Same seeded Gaussian sequence, no interpolation or hidden truth reanchoring.
Unobserved source prefix is skipped by the reported accepted counter, not
invented as raw samples. At least two recorded adjacent pairs are required.
"""
    by_stamp={round(d['stamp_s'],8):d for d in diagnostics if d.get('accepted') is True}
    rng=random.Random(parameters['seed']);draws=0;checks=[];max_residual=0.0
    zero=(parameters['scale_x']==parameters['scale_y']==1 and
          parameters['yaw_bias_rad_s']==parameters['position_walk_m_sqrt_s']==parameters['yaw_walk_rad_sqrt_s']==0)
    for (ta,ga,na),(tb,gb,nb) in zip(pairs,pairs[1:]):
        dt=tb-ta;diag=by_stamp.get(round(tb,8))
        prior=by_stamp.get(round(ta,8))
        if diag is None or prior is None or diag['accepted_count']!=prior['accepted_count']+1:
            checks.append(False);continue
        if not 0<dt<=parameters['max_source_gap_s']+1e-9:
            checks.append(False);continue
        g0,g1,n0,n1=[d['pose']['pose'] for d in (ga,gb,na,nb)]
        a0,a1,nangle=map(yaw,(g0['orientation'],g1['orientation'],n0['orientation']))
        dx=g1['position']['x']-g0['position']['x'];dy=g1['position']['y']-g0['position']['y']
        c,s=math.cos(a0),math.sin(a0)
        bx=(c*dx+s*dy)*parameters['scale_x'];by=(-s*dx+c*dy)*parameters['scale_y']
        desired_draws=3*(diag['accepted_count']-2)
        while draws<desired_draws:rng.gauss(0,1);draws+=1
        ny=nx=nr=0.0
        if not zero:
            if draws!=desired_draws:checks.append(False);continue
            nr=rng.gauss(0,parameters['yaw_walk_rad_sqrt_s']*math.sqrt(dt))
            nx=rng.gauss(0,parameters['position_walk_m_sqrt_s']*math.sqrt(dt))
            ny=rng.gauss(0,parameters['position_walk_m_sqrt_s']*math.sqrt(dt));draws+=3
        c,s=math.cos(nangle),math.sin(nangle)
        expected=(n0['position']['x']+c*bx-s*by+nx,
                  n0['position']['y']+s*bx+c*by+ny,
                  nangle+math.atan2(math.sin(a1-a0),math.cos(a1-a0))+
                  parameters['yaw_bias_rad_s']*dt+nr)
        delta=math.dist(expected[:2],(n1['position']['x'],n1['position']['y']))
        angular=abs(math.atan2(math.sin(yaw(n1['orientation'])-expected[2]),
                               math.cos(yaw(n1['orientation'])-expected[2])))
        max_residual=max(max_residual,delta,angular)
        checks.append(delta<1e-9 and angular<1e-9)
    return {'pass':bool(checks) and all(checks),'compared_steps':len(checks),
            'maximum_algebraic_residual':max_residual,'failed_steps':sum(not c for c in checks),
            'prefix_assumption':'Recorded accepted counters account for RNG draws before recorder discovery; no prefix samples are fabricated.'}

def odometry_audit(directory, name, zero):
    topics = {}
    rows = load_jsonl(directory/(name+'.native.jsonl'))
    for row in rows:
        topics.setdefault(row['topic'], []).append(row)
    def indexed(topic):
        return {round(row['message_stamp_s'], 8):row for row in topics.get(topic, [])
                if row['message_stamp_s'] is not None}
    gt, injected, ground = [indexed(t) for t in ('/gazebo/odometry',
                               '/simulation/navigation_odometry', '/ground/odometry')]
    start = read(directory/(name+'.summary.json'))['goal_delivery'].get('publish_monotonic_ns', 0)
    nav_rows = [r for r in topics.get('/simulation/navigation_odometry', [])
                if r['receive_monotonic_ns'] >= start]
    errors, yaw_errors, zero_differences, pairs = [], [], [], []
    missing = 0
    for row in nav_rows:
        key = round(row['message_stamp_s'], 8)
        original = gt.get(key)
        if original is None:
            missing += 1
            continue
        a, b = row['data']['pose']['pose'], original['data']['pose']['pose']
        errors.append(math.dist([a['position']['x'],a['position']['y']],
                                [b['position']['x'],b['position']['y']]))
        yaw_errors.append(math.atan2(math.sin(yaw(a['orientation'])-yaw(b['orientation'])),
                                    math.cos(yaw(a['orientation'])-yaw(b['orientation']))))
        zero_differences.append(row['data']['pose']==original['data']['pose'] and
                                row['data']['twist']==original['data']['twist'])
        pairs.append((row['message_stamp_s'],original['data'],row['data']))
    conversion = []
    for key, row in ground.items():
        if row['receive_monotonic_ns'] < start: continue
        src = injected.get(key)
        if src is None:
            conversion.append(False)
            continue
        a, b = row['data'], src['data']
        c, s = math.cos(yaw(b['pose']['pose']['orientation'])), math.sin(yaw(b['pose']['pose']['orientation']))
        v = b['twist']['twist']['linear']
        w = a['twist']['twist']['linear']
        conversion.append(math.dist([a['pose']['pose']['position']['x'],a['pose']['pose']['position']['y']],
                                    [b['pose']['pose']['position']['x'],b['pose']['pose']['position']['y']])<1e-10 and
                          abs(w['x']-(c*v['x']-s*v['y']))<1e-10 and
                          abs(w['y']-(s*v['x']+c*v['y']))<1e-10)
    diagnostics = [json.loads(r['data']['data']) for r in topics.get('/simulation/odometry_diagnostics', [])]
    rejected = [d for d in diagnostics if d.get('accepted') is not True or d.get('rejected_count',0)>0]
    parameters=next(iter(yaml.safe_load((directory/'runtime_parameters/stage2_simulated_odometry.yaml').read_text()).values()))['ros__parameters']
    recurrence=model_steps(pairs,diagnostics,parameters)
    receive = [r['receive_monotonic_ns'] for r in nav_rows]
    stamps = [r['message_stamp_s'] for r in nav_rows]
    mono_gaps = [(b-a)*1e-9 for a,b in zip(receive,receive[1:])]
    sim_gaps = [b-a for a,b in zip(stamps,stamps[1:])]
    checks = {'injected_topic_observed':len(nav_rows)>10,
              'paired_with_raw_truth_at_same_stamp':missing==0 and bool(errors),
              'no_model_rejections_observed':bool(diagnostics) and not rejected,
              'injected_raw_fields_finite_and_frames_valid':bool(nav_rows) and all(
                  finite_tree(r['data']) and r['data']['header']['frame_id']=='odom' and
                  r['data']['child_frame_id']=='base_link' for r in nav_rows) and all(finite_tree(d) for d in diagnostics),
              'injected_increment_recurrence_matches_profile':recurrence['pass'],
              'ground_uses_injected_pose_and_body_to_world_twist':bool(conversion) and all(conversion),
              'injected_receive_and_stamp_continuity':bool(mono_gaps) and
                    all(0<d<=.2 for d in mono_gaps) and all(0<d<=.2 for d in sim_gaps),
              'zero_profile_exact_identity':not zero or bool(zero_differences) and all(zero_differences)}
    return {'pass':all(checks.values()),'checks':checks,'recurrence':recurrence,'paired_samples':len(errors),
            'max_position_error_m':max(errors,default=None),'last_position_error_m':errors[-1] if errors else None,
            'max_abs_yaw_error_rad':max(map(abs,yaw_errors),default=None),
            'last_yaw_error_rad':yaw_errors[-1] if yaw_errors else None,
            'max_receive_gap_s':max(mono_gaps,default=None),'max_stamp_gap_s':max(sim_gaps,default=None),
            'prefix_limit':'Full injector lifetime before recorder discovery is not recorded; comparisons use raw same-stamp pairs during the recorded task.'}


def audit(directory):
    progress = read(directory/'progress.json')
    name, profile = progress['name'], progress['profile']
    manifest, after = read(directory/'input_manifest.json'), read(directory/'input_sha256_after.json')
    hashes = read(directory/'raw_data_sha256.json')
    summary = read(directory/(name+'.summary.json'))
    preflight = read(directory/(name+'.offline_preflight.json'))
    def evidence(function,*args):
        try:return function(*args)
        except (OSError,ValueError,KeyError,TypeError,IndexError) as error:
            return {'pass':False,'status':'INCOMPLETE_OR_INVALID_EVIDENCE','error':type(error).__name__+': '+str(error)}
    waypoint = evidence(stage2_waypoint_sequence,directory,name,summary,preflight)
    terminal = evidence(audit_run,directory,name,sdf_walls())
    native = evidence(native_audit,directory,summary,walls_from_sdf(directory/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf'),name)
    native.pop('gt_trace',None)
    nt = evidence(stage2_native_terminal_audit,directory,summary,waypoint,name)
    odometry = evidence(odometry_audit,directory,name,profile=='zero')
    graph = read(directory/'odometry_graph.json')
    params = {p.stem:next(iter(yaml.safe_load(p.read_text()).values()))['ros__parameters']
              for p in (directory/'runtime_parameters').glob('*.yaml')}
    model_params = params['stage2_simulated_odometry']
    expected = next(iter(yaml.safe_load((directory/'input_snapshot'/progress['profile_path']).read_text()).values()))['ros__parameters']
    input_checks = {p:sha(directory/'input_snapshot'/p)==v['sha256']==after[p]['before']==after[p]['after']
                          and v['resolved_path']==after[p]['resolved_path_before']==after[p]['resolved_path_after']
                    for p,v in manifest['inputs'].items()}
    parameter_hashes=read(directory/'runtime_parameter_sha256.json')
    nav, planner, executor = (params[n] for n in ('gazebo_navigation_interface','ego_planner_node','ego_trajectory_executor'))
    checks = {'one_world_one_goal':progress.get('launches')==1 and progress.get('goals_requested')==1 and
                              summary['goal_delivery'].get('publish_count')==1,
              'truth_decoupled_graph': 'gazebo_navigation_interface' not in graph['truth_subscriber_names'] and
                         'gazebo_navigation_interface' in graph['injected_subscriber_names'] and
                         graph['injected_publisher_names']==['stage2_simulated_odometry'],
              'inputs_snapshotted_unchanged':bool(input_checks) and all(input_checks.values()),
              'raw_hashes_match':bool(hashes) and all(sha(directory/p)==h for p,h in hashes.items()),
              'runtime_parameter_hashes_match':bool(parameter_hashes) and all(
                    sha(directory/'runtime_parameters'/p)==h for p,h in parameter_hashes.items()),
              'profile_runtime_matches':all(model_params.get(k)==v for k,v in expected.items()),
              'frozen_safety_and_control_parameters':nav['imperfect_sensors'] is False and
                     nav['grid_route_clearance']==.4 and nav['provincial_min_body_gap_m']==.08 and
                     planner['grid_map/obstacles_inflation']==.22 and planner['manager/max_vel']==.25 and
                     planner['manager/max_acc']==.35 and executor['max_vel_x_mps']==.25 and
                     executor['max_vel_y_mps']==.25 and executor['max_yaw_rate_rad_s']==.3 and
                     executor['position_kp']==1.2 and executor['yaw_kp']==2.0 and executor['trajectory_start_clock']=='ros' and
                     all(p.get('use_sim_time') is True for p in params.values()),
              'odometry_separation_and_recording':odometry['pass'],
              'navigation_completed':summary['result']=='terminal_pass',
              'goal_start_final_yaw_within_existing_0.05_rad':
                    abs(summary['sent_goal_yaw_rad']-math.pi/2)<=.05 and
                    abs(summary['final_gt_yaw_rad']-math.pi/2)<=.05 and
                    math.dist(summary['initial_gt_pose'][:2],preflight['measured_start_gt'][:2])<=.03,
              'all_waypoints':waypoint['pass'],'full_2_plus_5_csv':terminal['pass'],
              'full_2_plus_5_native':nt['pass'],'native_raw_evidence':native['pass'],
              'no_retry_stuck_false_success':summary['retry_count']==summary['stuck_count']==0 and not summary['false_success'],
              'world_closed':progress.get('remaining_gazebo_servers')==[]}
    return {'all_pass':all(checks.values()),'checks':checks,'profile':profile,'summary':summary,
            'waypoints':waypoint,'terminal':terminal,'native_terminal':nt,'native':native,
            'odometry':odometry,'input_checks':input_checks,'classification':'TRAINING-ONLY / PROVISIONAL',
            'competition_arena_verified':False,'contact':'CONTACT DATA UNAVAILABLE',
            'continuous_time_safety_proven':False,'hardware_noise_calibrated':False,
            'evaluator_sha256':sha(Path(__file__)),'command':sys.argv}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',required=True,type=Path)
    p.add_argument('--output',type=Path);a=p.parse_args()
    if a.output and a.output.exists():raise FileExistsError(a.output)
    result=audit(a.result_dir)
    if a.output:a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'all_pass':result['all_pass'],'checks':result['checks'],'odometry':result['odometry']},ensure_ascii=False))
    sys.exit(0 if result['all_pass'] else 2)
