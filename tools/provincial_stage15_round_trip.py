#!/usr/bin/env python3
"""Bounded forward/return evidence. No pose reset, no navigation modification."""
import argparse, hashlib, json, math, os, signal, subprocess, sys
from pathlib import Path
import rclpy
import run_provincial_full_observed as observed
from run_provincial_forward_integration import ROOT, MAP, FIELD, PoseObserver, offline_leg, server_processes
from run_provincial_safety_fix_matrix import existing_servers, wait_for_launch
from summarize_provincial_full_observed import native_audit, native_terminal_audit
from summarize_provincial_forward_integration import raw_waypoint_sequence
from summarize_provincial_stage15_terminal_v3 import audit_run, load_jsonl, sdf_walls
from analyze_provincial_forward_clearance import walls_from_sdf
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from grid_route import GridRoute

SIDES={'forward':((4.7,.5),(8.7,4.25)), 'return':((8.7,4.25),(4.7,.5))}

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def write(p,data): Path(p).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')

def audit_leg(directory, side):
    name=directory.name; summary=read(directory/(name+'.summary.json'))
    preflight=read(directory/(name+'.offline_preflight.json'))
    sequence=raw_waypoint_sequence(load_jsonl(directory/(name+'.events.jsonl')),summary,preflight)
    terminal=audit_run(directory,name,sdf_walls())
    native=native_audit(directory,summary,walls_from_sdf(directory.parent/'input_snapshot'/'src/uav_bringup/worlds/provincial_2025_training.sdf'),name,expected_goal=SIDES[side][1])
    native.pop('gt_trace',None)
    nt=native_terminal_audit(directory,summary,sequence,name)
    hashes=read(directory/'raw_data_sha256.json')
    required={name+x for x in ('.summary.json','.csv','.events.jsonl','.plans.jsonl','.actuation.jsonl','.published_commands.jsonl','.native.jsonl','.native.health.json','.native.timing.jsonl','.runner.log')}
    delivery=summary.get('goal_delivery') or {}
    checks={'navigation_completed':summary.get('result')=='terminal_pass',
        'runner_exit_zero':read(directory/'execution.json')['returncode']==0,
        'raw_hashes':required<=hashes.keys() and all(sha(directory/p)==h for p,h in hashes.items()),
        'terminal_csv':terminal['pass'],'all_waypoints':sequence['pass'],
        'native_messages':native['pass'],'native_terminal':nt['pass'],
        'matched_receiver':delivery.get('publish_count')==1 and delivery.get('state')=='accepted' and (delivery.get('readiness') or {}).get('ready') is True and (delivery.get('acceptance') or {}).get('route_id')==sequence.get('route_id'),
        'expected_goal_yaw_start':summary['goal_xy']==list(SIDES[side][1]) and abs(summary['sent_goal_yaw_rad']-math.pi/2)<.05 and abs(summary['final_gt_yaw_rad']-math.pi/2)<.05 and math.dist(summary['initial_gt_pose'][:2],preflight['measured_start_gt'][:2])<=.03,
        'no_retry_stuck_false_success':summary['retry_count']==0 and summary['stuck_count']==0 and not summary['false_success'] and summary['geometric_overlap_samples']==0}
    return {'all_pass':all(checks.values()),'checks':checks,'side':side,'name':name,'waypoint':sequence,'terminal':terminal,'native_terminal':nt,'native':native,'summary':summary,'audit_sha256':sha(__file__)}

def audit_session(out):
    progress=read(out/'progress.json'); manifest=read(out/'input_manifest.json'); after=read(out/'input_sha256_after.json')
    input_checks={p:sha(out/'input_snapshot'/p)==v['sha256']==after[p]['before']==after[p]['after'] and v['resolved_path']==after[p]['resolved_path_before']==after[p]['resolved_path_after'] for p,v in manifest['inputs'].items()}
    ph=read(out/'runtime_parameter_sha256.json'); rh=read(out/'raw_data_sha256.json')
    import yaml
    parameters={p.stem:next(iter(yaml.safe_load(p.read_text()).values()))['ros__parameters'] for p in (out/'runtime_parameters').glob('*.yaml')}
    nav=parameters['gazebo_navigation_interface'];ex=parameters['ego_trajectory_executor'];plan=parameters['ego_planner_node']
    fixed=(nav.get('imperfect_sensors') is False and nav.get('grid_route_clearance')==.4 and nav.get('provincial_min_body_gap_m')==.08 and plan.get('grid_map/obstacles_inflation')==.22 and plan.get('manager/max_vel')==.25 and plan.get('manager/max_acc')==.35 and ex.get('max_vel_x_mps')==.25 and ex.get('max_vel_y_mps')==.25 and ex.get('max_yaw_rate_rad_s')==.3 and ex.get('position_kp')==1.2 and ex.get('yaw_kp')==2.0 and ex.get('trajectory_start_clock')=='ros' and all(v.get('use_sim_time') is True for v in parameters.values()))
    legs=[]; errors=[]
    for entry in progress['legs']:
        try: legs.append(audit_leg(out/entry['name'],entry['side']))
        except (OSError,ValueError,KeyError,TypeError,IndexError) as e: errors.append(str(e))
    continuity=False
    if len(legs)==2:
        a,b=legs; sa,sb=a['summary'],b['summary']
        continuation=read(out/'continuation.json')
        accepted=b['native']['raw_route_receipts'][0]['receive_monotonic_ns']
        completed=sa['goal_delivery']['publish_monotonic_ns']+round(sa['terminal_snapshot']['wall_elapsed_s']*1e9)
        continuity=(a['waypoint']['route_id']!=b['waypoint']['route_id'] and sb['terminal_snapshot']['gt_stamp_s']-sb['duration_sim_s']>=sa['terminal_snapshot']['gt_stamp_s'] and accepted>completed and math.dist(sb['initial_gt_pose'][:2],sa['terminal_snapshot']['gt_xy'])<=.03 and continuation['servers_before_return']==progress['gazebo_server_processes'] and continuation['full_live_audit_pass'])
    checks={'fixed_runtime_parameters':fixed,'inputs_unchanged':all(input_checks.values()),'runtime_params_hashed':bool(ph) and all(sha(out/'runtime_parameters'/p)==h for p,h in ph.items()),'parent_raw_hashed':bool(rh) and all(sha(out/p)==h for p,h in rh.items()),'all_legs_pass':len(legs)==2 and not errors and all(x['all_pass'] for x in legs),'same_world_no_reset_evidence':continuity,'one_world_closed':progress.get('launches')==1 and len(progress.get('gazebo_server_processes',[]))==1 and progress.get('remaining_gazebo_servers')==[] and not progress.get('failure')}
    return {'all_pass':all(checks.values()),'checks':checks,'legs':legs,'errors':errors,'input_checks':input_checks,'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,'contact_data':'CONTACT DATA UNAVAILABLE','reset_observation_limit':'No reset/set_pose calls in runner; pose/time/PID continuity recorded. All third-party service calls not independently observed.','audit_sha256':sha(__file__)}

def run(out):
    if out.exists():raise FileExistsError(out)
    if existing_servers():raise RuntimeError('Existing Gazebo: refuse')
    grid=GridRoute(MAP,clearance=.40); walls=sdf_walls(); reference=read(FIELD)['reference_route']
    nominal={side:offline_leg(a,b,grid,walls,reference) for side,(a,b) in SIDES.items()}
    out.mkdir(parents=True); write(out/'nominal_preflight.json',nominal)
    observed.OUT=out; observed.NAME=out.name
    observed.INPUTS=observed.INPUTS+('tools/provincial_stage15_round_trip.py',)+('src/third_party/ego-planner-swarm/src/planner/plan_manage/include/ego_planner/planner_manager.h', 'src/third_party/ego-planner-swarm/src/planner/plan_manage/src/planner_manager.cpp', 'src/third_party/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp', 'src/third_party/ego-planner-swarm/src/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h', 'src/third_party/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp', 'src/third_party/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp')
    manifest=observed.capture_inputs()
    # Replace only the recorded runner plan before any process/goal is started.
    manifest['mission_plan']={'legs':['forward','return'],'reset':False,'retry':False}
    manifest['runner_argv']=None
    write(out/'input_manifest.json',manifest)
    progress={'launches':0,'legs':[],'goals_requested':0,'status':'PREFLIGHT'}; write(out/'progress.json',progress)
    launch=None; node=None; logfile=None
    try:
        logfile=(out/'launch.log').open('x'); launch=subprocess.Popen(observed.LAUNCH,stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
        progress.update(launches=1,launch_pid=launch.pid); write(out/'progress.json',progress)
        wait_for_launch(); progress['gazebo_server_processes']=server_processes()
        if len(progress['gazebo_server_processes'])!=1:raise RuntimeError('Not one Gazebo server')
        observed.capture_runtime_params()
        import yaml
        live=yaml.safe_load((out/'runtime_parameters/ego_trajectory_executor.yaml').read_text())
        if live['/ego_trajectory_executor']['ros__parameters'].get('trajectory_start_clock')!='ros':
            raise RuntimeError('Live trajectory start clock is not explicitly ROS')
        rclpy.init(); node=PoseObserver()
        for side in ('forward','return'):
            actual=node.fresh_pose(); a,b=SIDES[side]
            if math.dist(actual[:2],a)>.20 or abs(actual[2]-math.pi/2)>.05:raise RuntimeError('Unexpected initial pose '+str(actual))
            preflight=offline_leg(actual[:2],b,grid,walls,reference)
            name=out.name+'_'+side; leg=out/name; leg.mkdir()
            write(leg/(name+'.offline_preflight.json'),dict(measured_start_gt=actual,**preflight))
            # Recorder-health version is captured with the whole shared runtime snapshot.
            (leg/'input_snapshot').symlink_to(out/'input_snapshot',target_is_directory=True)
            command=[sys.executable,'-B',str(ROOT/'tools/run_provincial_low_speed_validation.py'),'--route','full' if side=='forward' else 'return','--name',name,'--yaw-mode','hold-start','--terminal-validation','--raw-every-message','--timeout','240','--output-dir',str(leg)]
            progress['legs'].append({'name':name,'side':side,'command':command}); progress['goals_requested']+=1; progress['status']='RUNNING_'+side.upper();write(out/'progress.json',progress)
            with (leg/(name+'.runner.log')).open('x') as log:
                result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=270)
            write(leg/'execution.json',{'returncode':result.returncode,'command':command})
            write(leg/'raw_data_sha256.json',{p.name:sha(p) for p in leg.iterdir() if p.is_file() and p.name!='raw_data_sha256.json'})
            try:
                report=audit_leg(leg,side)
            except (OSError,ValueError,KeyError,TypeError,IndexError) as error:
                write(leg/'live_evidence_audit.json',{'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':str(error)})
                raise RuntimeError(side+' runner/evidence failed; original result='+str(read(leg/(name+'.summary.json')).get('result'))) from error
            write(leg/'live_evidence_audit.json',report)
            print(json.dumps({'side':side,'all_pass':report['all_pass'],'checks':report['checks'],'minimum':report['native']['minimum']},ensure_ascii=False),flush=True)
            if not report['all_pass']:raise RuntimeError(side+' audit failed: '+str([k for k,v in report['checks'].items() if not v]))
            if side=='forward':
                write(out/'continuation.json',{'full_live_audit_pass':True,'servers_before_return':server_processes(),'actual_pose_after_full':node.fresh_pose(after_sim=report['summary']['terminal_snapshot']['gt_stamp_s'])})
        progress['status']='RUN_FINISHED'
    except Exception as e:
        progress.update(status='FAILED_PRESERVED',failure=type(e).__name__+': '+str(e)); print(progress['failure'],flush=True)
    finally:
        if node is not None:node.destroy_node();rclpy.shutdown()
        if launch:
            try:os.killpg(launch.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:launch.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=8)
        if logfile:logfile.close()
        progress['remaining_gazebo_servers']=server_processes();write(out/'progress.json',progress);observed.final_hashes(manifest)
    report=audit_session(out);write(out/'mission_verification.json',report)
    print(json.dumps({'all_pass':report['all_pass'],'checks':report['checks'],'errors':report['errors']},ensure_ascii=False),flush=True)
    return report['all_pass']

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',required=True,type=Path);parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
    if args.verify_only:
        result=audit_session(args.output_dir);print(json.dumps({'all_pass':result['all_pass'],'checks':result['checks'],'errors':result['errors']},ensure_ascii=False));sys.exit(0 if result['all_pass'] else 2)
    sys.exit(0 if run(args.output_dir.resolve()) else 2)
