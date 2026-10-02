"""Read-only clock/delayed spline diagnosis from one failed evidence set."""
import argparse,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from ego_trajectory_executor import Spline
p=argparse.ArgumentParser();p.add_argument('--trial-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
if args.output.exists():raise FileExistsError(args.output)
name=args.trial_dir.name;summary=json.loads((args.trial_dir/(name+'.summary.json')).read_text());delivery=summary['goal_delivery'];offset=delivery['publish_wall_ns']-delivery['publish_monotonic_ns'];gt=None;items=[]
for line in (args.trial_dir/(name+'.native.jsonl')).read_text().splitlines():
 row=json.loads(line)
 if row['topic']=='/gazebo/odometry':gt=row
 if row['topic']!='/planning/bspline':continue
 d=row['data'];start=d['start_time']['sec']*1e9+d['start_time']['nanosec'];receive_system=row['receive_wall_ns'];sim=row['receive_sim_s'];s=Spline([(q['x'],q['y']) for q in d['pos_pts']],d['order'],d['knots']);age=(receive_system-start)*1e-9;duration=s.end-s.start
 old=s.evaluate(s.start);comp=s.evaluate(s.start+max(0,min(duration,age)));pt=gt['data']['pose']['pose']['position'] if gt else None
 items.append({'trajectory_id':d['traj_id'],'receive_sim_s':sim,'start_system_s':start*1e-9,'direct_receive_system_s':receive_system*1e-9,'system_vs_monotonic_offset_change_s':(row['receive_wall_ns']-(row['receive_monotonic_ns']+offset))*1e-9,'estimated_transport_or_planning_age_s':age,'old_elapsed_at_receive_s':max(0,min(duration,sim-start*1e-9)),'old_start_setpoint_xy':old,'compensated_candidate_setpoint_xy':comp,'gt_xy_at_receive':[pt['x'],pt['y']] if pt else None,'duration_s':duration,'gt_header_sim_s':gt['message_stamp_s'] if gt else None})
report={'classification':'READ-ONLY DIAGNOSIS','time_basis':'Direct original receive_wall_ns; offset change compares recorded wall and monotonic clocks. Earlier fixed-offset analysis is superseded.','all_native_spline_start_stamps_in_system_epoch':all(x['start_system_s']>1e9 for x in items),'all_executor_old_elapsed_zero':all(x['old_elapsed_at_receive_s']==0 for x in items),'max_estimated_plan_age_s':max(x['estimated_transport_or_planning_age_s'] for x in items),'relevant_delayed_trajectory':[x for x in items if x['trajectory_id']==8],'items':items,'not_unique_retry_cause_proven':True}
args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='items'},ensure_ascii=False))
