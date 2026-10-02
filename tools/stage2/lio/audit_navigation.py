#!/usr/bin/env python3
"""Independent actual route/terminal plus sensor/LIO source checks."""
import sys,json,hashlib,math,argparse
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tools/stage2'))
from audit_localized import stage2_waypoint_sequence
from stage2_terminal_evidence import stage2_native_terminal_audit
from summarize_provincial_stage15_terminal_v3 import audit_run,sdf_walls
from summarize_provincial_full_observed import native_audit
from analyze_provincial_forward_clearance import walls_from_sdf
from audit_sensors import audit_sensor_file
from audit_lio_source import audit_lio_source

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())

def audit(out):
    phase=load(out/'experiment_metadata.json')['stage4_phase'];name='lio_'+phase+'_01'
    summary=load(out/(name+'.summary.json'));pre=load(out/(name+'.offline_preflight.json'))
    progress=load(out/'progress.json');manifest=load(out/'input_manifest.json');after=load(out/'input_sha256_after.json')
    hashes=load(out/'raw_data_sha256.json')
    expected_goal={'short':[4.7,1.15],'full':[8.7,4.25]}[phase]
    def evidence(fn,*args,**kw):
        try:return fn(*args,**kw)
        except Exception as e:return {'pass':False,'error':repr(e)}
    waypoint=evidence(stage2_waypoint_sequence,out,name,summary,pre)
    csv=evidence(audit_run,out,name,sdf_walls())
    terminal=evidence(stage2_native_terminal_audit,out,summary,waypoint,name)
    native=evidence(native_audit,out,summary,walls_from_sdf(out/'input_snapshot/src/uav_bringup/worlds/provincial_stage2_lio.sdf'),name,expected_goal=expected_goal)
    native.pop('gt_trace',None)
    sensor={'pass':False,'error':'raw unique external goal required to delimit the task'}
    # Use raw goal's real clock association for sensor continuity boundaries.
    rows=(json.loads(l) for l in (out/(name+'.native.jsonl')).open())
    goals=[];gt={};lios=[];inputs=[]
    for row in rows:
        if row['topic']=='/goal_pose':goals.append(row)
        if row['topic']=='/gazebo/odometry':gt[round(row['message_stamp_s'],8)]=row['data']
        if row['topic']=='/localization/lio_navigation_odometry':lios.append(row)
    if len(goals)==1:sensor=evidence(audit_sensor_file,out/(name+'.sensors.jsonl'),goals[0]['receive_sim_s'],summary['terminal_snapshot']['gt_stamp_s'])
    graph=load(out/'lio_goal_preflight.json')['graph']
    actual_params={p.stem:next(iter(yaml.safe_load(p.read_text()).values()))['ros__parameters'] for p in (out/'runtime_parameters').glob('*.yaml')}
    nav=actual_params['gazebo_navigation_interface'];planner=actual_params['ego_planner_node'];ex=actual_params['ego_trajectory_executor'];lio=actual_params['lio_mapping']
    errors=[]
    for row in lios:
        if goals and row['receive_monotonic_ns']<goals[0]['receive_monotonic_ns']:continue
        nearest=min(gt,key=lambda t:abs(t-row['message_stamp_s'])) if gt else None
        if nearest is None or abs(nearest-row['message_stamp_s'])>.025:continue
        g=gt[nearest]['pose']['pose']['position'];p=row['data']['pose']['pose']['position']
        errors.append(math.hypot(p['x']-g['x'],p['y']-g['y']))
    checks={'one_world_one_external_goal':progress['launches']==1 and progress['goals_requested']==1 and len(goals)==1,
        'prescribed_phase_goal':summary['goal_xy']==expected_goal,
        'navigation_completed':summary['result']=='terminal_pass','all_waypoints':waypoint['pass'],
        'full_2_plus_5_csv':csv['pass'],'full_2_plus_5_native':terminal['pass'],'native_ground_evidence':native['pass'],
        'three_dimensional_sensor_evidence':sensor['pass'],
        'inputs_snapshotted_unchanged':all(sha(out/'input_snapshot'/p)==v['sha256']==after[p]['before']==after[p]['after'] and v['resolved_path']==after[p]['resolved_path_after'] for p,v in manifest['inputs'].items()),
        'original_raw_hashes_match':all(sha(out/p)==h for p,h in hashes.items()),
        'GT_not_localization_or_navigation_input':not any(e['name'] in ['lio_mapping','lio_sensor_adapter','lio_odometry_adapter','gazebo_navigation_interface'] for e in graph['/gazebo/odometry']['sub']),
        'LIO_only_navigation_pose_publisher':[e['name'] for e in graph['/localization/lio_navigation_odometry']['pub']]==['lio_odometry_adapter'],
        'navigation_subscribes_LIO':[e['name'] for e in graph['/localization/lio_navigation_odometry']['sub']].count('gazebo_navigation_interface')==1,
        'existing_thresholds':nav['imperfect_sensors'] is False and nav['grid_route_clearance']==.4 and nav['provincial_min_body_gap_m']==.08 and planner['grid_map/obstacles_inflation']==.22 and planner['manager/max_vel']==.25 and planner['manager/max_acc']==.35 and ex['max_vel_x_mps']==ex['max_vel_y_mps']==.25 and ex['max_yaw_rate_rad_s']==.3 and ex['position_kp']==1.2 and ex['yaw_kp']==2. and ex['trajectory_start_clock']=='ros',
        'LIO_full_3D_no_online_extrinsic_or_time_sync':lio['common.planar_mode'] is False and lio['mapping.extrinsic_est_en'] is False and lio['common.time_sync_en'] is False,
        'no_retry_stuck_false_success':summary['retry_count']==summary['stuck_count']==0 and not summary['false_success'],
        'yaw90_start_and_finish':abs(summary['initial_gt_pose'][2]-math.pi/2)<.05 and abs(summary['final_gt_yaw_rad']-math.pi/2)<.05,
        'localization_pairs_present':len(errors)>100,
        'world_closed':not progress['remaining_gazebo_servers']}
    source=evidence(audit_lio_source,out,name)
    checks['actual_LIO_and_physical_IMU_source_recomputed']=source['pass']
    return {'all_pass':all(checks.values()),'checks':checks,'waypoint':waypoint,'csv_terminal':csv,'native_terminal':terminal,
        'native':native,'sensor':sensor,'LIO_source':source,'summary':summary,'LIO_max_task_xy_error_m':max(errors) if errors else None,
        'LIO_pair_association':'nearest raw GT within25ms, no interpolation; not used by navigation',
        'evaluator_sha256':sha(Path(__file__)),'command':sys.argv,'classification':'TRAINING-ONLY / PROVISIONAL',
        'contact':'CONTACT DATA UNAVAILABLE','competition_arena_verified':False}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    try:r=audit(a.result_dir)
    except Exception as error:r={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':repr(error)}
    with a.output.open('x') as f:json.dump(r,f,indent=2)
    print(json.dumps({k:r.get(k) for k in ['all_pass','checks','error']},indent=2));raise SystemExit(0 if r['all_pass'] else 2)
