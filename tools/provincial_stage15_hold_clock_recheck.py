#!/usr/bin/env python3
"""One native-recorded current-build A hold/release; independent evidence audit."""
import argparse,json,math,sys
from pathlib import Path
import run_provincial_full_observed as observed
import run_provincial_hold_resume as hold
from provincial_stage15_round_trip import sha,read,write
from summarize_provincial_full_observed import native_audit,native_terminal_audit
from summarize_provincial_forward_integration import raw_waypoint_sequence
from summarize_provincial_stage15_terminal_v3 import audit_run,load_jsonl,sdf_walls
from analyze_provincial_forward_clearance import walls_from_sdf
from provincial_safety_geometry import rectangle,polygon_distance,body_wall_gap
CLOCK_FILES=('src/third_party/ego-planner-swarm/src/planner/plan_manage/include/ego_planner/planner_manager.h','src/third_party/ego-planner-swarm/src/planner/plan_manage/src/planner_manager.cpp','src/third_party/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp','src/third_party/ego-planner-swarm/src/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h','src/third_party/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp','src/third_party/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp')
def audit(out):
 leg=out/'trial_hold_clock_01';name=leg.name;s=read(leg/(name+'.summary.json'));h=read(leg/'hold_trial_summary.json');seq=raw_waypoint_sequence(load_jsonl(leg/(name+'.events.jsonl')),s,{'reference_stages':[[4.7,1.15]]});term=audit_run(leg,name,sdf_walls());native=native_audit(leg,s,walls_from_sdf(out/'input_snapshot/src/uav_bringup/worlds/provincial_2025_training.sdf'),name,expected_goal=(4.7,1.15));native.pop('gt_trace',None);nt=native_terminal_audit(leg,s,seq,name,expected_stages=[[4.7,1.15]])
 rows=load_jsonl(leg/(name+'.native.jsonl'));begin=h['hold_event']['wall_monotonic_s']*1e9;end=h['obstacle_remove_wall_s']*1e9
 final=[x for x in rows if x['topic']=='/model/omni_robot/cmd_vel' and begin<=x['receive_monotonic_ns']<=end]
 gt=[x for x in rows if x['topic']=='/gazebo/odometry' and h['hold_event']['latest_gt_sim_s']-.021<=x['message_stamp_s']<=h['hold_end_gt_sim_s']+.021]
 raw=[x for x in rows if x['topic']=='/cmd_vel' and begin<=x['receive_monotonic_ns']<=end]
 diag=[x for x in rows if x['topic']=='/ground/planning/actuation_diagnostics' and begin<=x['receive_monotonic_ns']<=end];payloads=[json.loads(x['data']['data']) for x in diag]
 def speed(row):return math.hypot(row['data']['linear']['x'],row['data']['linear']['y'])
 def point(row):p=row['data']['pose']['pose']['position'];return (p['x'],p['y'])
 def gt_speed(row):t=row['data']['twist']['twist'];return math.hypot(t['linear']['x'],t['linear']['y'])
 stopped=next((x for x in gt if gt_speed(x)<=.02),None)
 settled=[x for x in gt if stopped and x['message_stamp_s']>=stopped['message_stamp_s']+.1]
 obstacle=rectangle(4.7,1.15,0,.1,.1)
 gap=min(polygon_distance(rectangle(*point(x),math.pi/2,.52,.42),obstacle) for x in gt)
 all_diag=[(x,json.loads(x['data']['data'])) for x in rows if x['topic']=='/ground/planning/actuation_diagnostics']
 first_wait=next(x for x,d in all_diag if d.get('navigation_state')=='waiting_for_clear_waypoint')
 state_window=[(x,d) for x,d in all_diag if x['receive_monotonic_ns']>=first_wait['receive_monotonic_ns'] and x['receive_sim_s']<=h['hold_end_gt_sim_s']]
 hashes=read(leg/'raw_data_sha256.json');manifest=read(out/'input_manifest.json');after=read(out/'input_sha256_after.json');ph=read(out/'runtime_parameter_sha256.json')
 import yaml
 params={p.stem:next(iter(yaml.safe_load(p.read_text()).values()))['ros__parameters'] for p in (out/'runtime_parameters').glob('*.yaml')};nav=params['gazebo_navigation_interface'];ex=params['ego_trajectory_executor'];plan=params['ego_planner_node']
 checks={'terminal_csv':term['pass'],'native_terminal':nt['pass'],'native_messages':native['pass'],'waypoints':seq['pass'],'single_goal_accepted':s['goal_delivery']['publish_count']==1 and s['goal_delivery']['state']=='accepted','inputs_unchanged':all(sha(out/'input_snapshot'/p)==v['sha256']==after[p]['before']==after[p]['after'] for p,v in manifest['inputs'].items()),'raw_hashes':bool(hashes) and all(sha(leg/p)==v for p,v in hashes.items()),'runtime_hashes':bool(ph) and all(sha(out/'runtime_parameters'/p)==v for p,v in ph.items()),'fixed_parameters':nav['imperfect_sensors'] is False and nav['grid_route_clearance']==.4 and nav['provincial_min_body_gap_m']==.08 and plan['grid_map/obstacles_inflation']==.22 and ex['trajectory_start_clock']=='ros','waiting_exercised':h['hold_state_exercised'] and any(x.get('navigation_state')=='waiting_for_clear_waypoint' and x.get('pending_waypoints',0)>0 and x.get('waypoint') is None for x in payloads),'waiting_three_sim_seconds':bool(gt) and h['hold_end_gt_sim_s']-h['hold_event']['latest_gt_sim_s']>=3 and gt[0]['message_stamp_s']<=h['hold_event']['latest_gt_sim_s'] and gt[-1]['message_stamp_s']>=h['hold_end_gt_sim_s'],'waiting_state_continuous':len(state_window)>=2 and h['hold_end_gt_sim_s']-first_wait['receive_sim_s']>=3 and h['hold_end_gt_sim_s']-state_window[-1][0]['receive_sim_s']<=.2 and all(d.get('navigation_state')=='waiting_for_clear_waypoint' and d.get('pending_waypoints',0)>0 and d.get('waypoint') is None and d['vx']==d['vy']==d['wz']==0 for x,d in state_window) and all(0<(b[0]['receive_monotonic_ns']-a[0]['receive_monotonic_ns'])*1e-9<=.2 for a,b in zip(state_window,state_window[1:])), 'waiting_final_exact_zero':bool(final) and all(speed(x)==0 and x['data']['angular']['z']==0 for x in final),'waiting_stable_after_stop':bool(settled) and all(gt_speed(x)<=.02 for x in settled) and max(math.dist(point(x),point(settled[0])) for x in settled)<=.03,'no_false_completion_while_waiting':not any(x['topic']=='/rosout/relevant' and begin<=x['receive_monotonic_ns']<=end and 'completed' in x['data'].get('msg','') for x in rows),'obstacle_geometry_no_overlap':gap>=.02,'original_task_resumed':h['obstacle_removed'] and h['status']=='RECOVERY_COMPLETED' and s['result']=='terminal_pass' and s['retry_count']==0 and s['stuck_count']==0,'world_closed':not h['gazebo_server_survived_cleanup']}
 return {'all_pass':all(checks.values()),'checks':checks,'summary':s,'terminal':term,'native_terminal':nt,'native':native,'waypoints':seq,'hold_evidence':{'waiting_state_samples':len(state_window),'waiting_state_sim_s':[first_wait['receive_sim_s'],h['hold_end_gt_sim_s']],'waiting_state_duration_sim_s':h['hold_end_gt_sim_s']-first_wait['receive_sim_s'],'final_count':len(final),'gt_count':len(gt),'wait_sim_seconds':gt[-1]['message_stamp_s']-gt[0]['message_stamp_s'],'stop_latency_sim_s':None if stopped is None else stopped['message_stamp_s']-gt[0]['message_stamp_s'],'max_displacement_m':max(math.dist(point(x),point(gt[0])) for x in gt),'minimum_robot_obstacle_gap_m':gap,'upstream_nonzero_observed':any(speed(x)>0 for x in raw),'upstream_stale_nonzero_coverage':'Only isolated production update tests establish stale prior-route command injection.'},'audit_sha256':sha(__file__),'contact':'CONTACT DATA UNAVAILABLE'}
def run(out):
 if out.exists():raise FileExistsError(out)
 if hold.existing_servers():raise RuntimeError('Gazebo already active')
 out.mkdir(parents=True);observed.OUT=out;observed.NAME='trial_hold_clock_01';observed.INPUTS+=('tools/run_provincial_hold_resume.py','tools/provincial_stage15_hold_clock_recheck.py')+CLOCK_FILES
 manifest=observed.capture_inputs();manifest['runner_argv']=None;manifest['mission_plan']={'mode':'one_A_hold_release','goal':[4.7,1.15],'external_goal_count':1,'reset':False};write(out/'input_manifest.json',manifest);hold.OUT=out/'trial_hold_clock_01';hold.TRIAL=hold.OUT.name
 code=hold.main(on_ready=observed.capture_runtime_params,raw_every_message=True)
 (hold.OUT/'input_snapshot').symlink_to(out/'input_snapshot',target_is_directory=True)
 observed.final_hashes(manifest)
 write(hold.OUT/'raw_data_sha256.json',{p.name:sha(p) for p in hold.OUT.iterdir() if p.is_file() and p.name!='raw_data_sha256.json'})
 try:report=audit(out)
 except Exception as e:report={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':str(e),'runner_returncode':code}
 write(out/'independent_hold_audit.json',report);print(json.dumps({'all_pass':report['all_pass'],'checks':report.get('checks'),'hold':report.get('hold_evidence'),'error':report.get('error')}));return report['all_pass']
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--verify-only',action='store_true');a=p.parse_args();result=audit(a.output_dir) if a.verify_only else run(a.output_dir);print(json.dumps(result) if a.verify_only else '');sys.exit(0 if (result['all_pass'] if isinstance(result,dict) else result) else 2)
