#!/usr/bin/env python3
"""Read-only final provisional-training closure, including every failed attempt."""
import argparse,json,math,sys
from pathlib import Path
from provincial_stage15_round_trip import ROOT,sha,read,write,audit_session
from provincial_stage15_hold_clock_recheck import audit as audit_hold
BASE=ROOT/'tools/results/provincial_stage15_completion_20261001'
FINAL=('round_trip_08','round_trip_09','round_trip_10')
def historical_hashes():
 dirs=('provincial_stage15_forward_integration_20260928','provincial_stage15_hold_resume_20260928','provincial_stage15_full_capture_20260929','provincial_stage15_full_delivery_20260929')
 checked={};missing=[];mismatch=[];manifests=[]
 for directory in dirs:
  for p in (ROOT/'tools/results'/directory).rglob('raw_data_sha256.json'):
   manifests.append(str(p));m=read(p)
   for rel,digest in m.items():
    path=p.parent/rel
    if not path.is_file():missing.append(str(path));continue
    checked[str(path)]={'expected':digest,'actual':sha(path)}
    if checked[str(path)]['actual']!=digest:mismatch.append(str(path))
 ledger=ROOT/'tools/results/provincial_stage15_native_terminal_fix_20260930/original_files_sha256.json'
 for rel,digest in read(ledger).items():
  path=ROOT/rel
  if not path.is_file():missing.append(str(path));continue
  checked[str(path)]={'expected':digest,'actual':sha(path)}
  if checked[str(path)]['actual']!=digest:mismatch.append(str(path))
 manifests.append(str(ledger))
 return {'pass':bool(checked) and not missing and not mismatch,'registered_files':len(checked),'manifests':manifests,'missing':missing,'mismatch':mismatch,'scope':'Registered files in named historical result directories; pre-existing unregistered files are outside this hash proof.'}
def summarize():
 sessions=[audit_session(BASE/name) for name in FINAL]
 manifests=[read(BASE/name/'input_manifest.json') for name in FINAL]
 sets=[{p:v['sha256'] for p,v in m['inputs'].items()} for m in manifests]
 same=all(s==sets[0] for s in sets)
 current={p:sha(ROOT/p)==v for p,v in sets[0].items()}
 local_elf=all(m.get('local_linked_elf_closure_captured') and m.get('all_local_installed_shared_libraries_snapshot') for m in manifests)
 hold=audit_hold(BASE/'hold_clock_recheck');hold_manifest=read(BASE/'hold_clock_recheck/input_manifest.json');hold_match={}
 for p,v in hold_manifest['inputs'].items():
  if not p.startswith(('install/','src/')):continue
  now=ROOT/p
  if p.endswith('provincial_safety_geometry.py'):
   snap=(BASE/'hold_clock_recheck/input_snapshot'/p).read_text();cur=now.read_text()
   hold_match[p]=(cur.replace("    if not walls:\n        raise ValueError('min() arg is an empty sequence')\n",'')==snap)
  else:hold_match[p]=sha(now)==v['sha256']
 history=historical_hashes();arena=read(BASE/'arena_drill_v3/offline_audit.json')
 arena_hashes={p:sha(BASE/'arena_drill_v3'/p)==v for p,v in arena['files'].items()}
 prior=[]
 for p in sorted(BASE.glob('round_trip_*')):
  if not p.is_dir() or p.name in FINAL:continue
  m=read(p/'mission_verification.json');progress=read(p/'progress.json')
  prior.append({'name':p.name,'all_pass':m['all_pass'],'failure':progress.get('failure'),'world_closed':progress.get('remaining_gazebo_servers')==[],'legs':[{'side':l['side'],'retry_count':l['summary']['retry_count'],'checks':l['checks'],'minimum_gap_m':l['native']['minimum']['gap_m']} for l in m['legs']]})
 rows=[]
 for name,session in zip(FINAL,sessions):
  for leg in session['legs']:
   path=BASE/name/leg['name']/(leg['name']+'.native.jsonl');first=last=None;count=0
   for line in path.open():
    if '/gazebo/odometry' not in line:continue
    x=json.loads(line)
    if x['topic']!='/gazebo/odometry' or x['message_stamp_s']>leg['summary']['terminal_snapshot']['gt_stamp_s']:continue
    if x['message_stamp_s']<leg['summary']['terminal_snapshot']['gt_stamp_s']-leg['summary']['duration_sim_s']:continue
    if first is None:first=x
    last=x;count+=1
   rows.append({'session':name,'side':leg['side'],'pass':leg['all_pass'],'gt_error_m':leg['terminal']['gt_error_m'],'odom_error_m':leg['terminal']['odom_error_m'],'yaw_rad':leg['terminal']['final_gt_yaw_rad'],'duration_sim_s':leg['summary']['duration_sim_s'],'goal_acceptance_delay_s':leg['summary']['goal_delivery']['acceptance']['wall_elapsed_s'],'route_id':leg['waypoint']['route_id'],'waypoints':leg['waypoint']['completed_ids'],'minimum':leg['native']['minimum'],'gt_sim_hz':leg['native']['active_gt_receive_rate_hz'],'gt_monotonic_receive_hz':(count-1)*1e9/(last['receive_monotonic_ns']-first['receive_monotonic_ns']),'max_gt_sim_gap_s':leg['native']['active_gt_max_gap_s'],'max_gt_receive_gap_s':leg['native']['topics']['/gazebo/odometry']['active_max_receive_gap_s'],'max_final_receive_gap_s':leg['native']['topics']['/model/omni_robot/cmd_vel']['active_max_receive_gap_s'],'max_odom_receive_gap_s':leg['native']['topics']['/ground/odometry']['active_max_receive_gap_s'],'native_terminal':leg['native_terminal']['full_7s'],'retry_count':leg['summary']['retry_count'],'stuck_count':leg['summary']['stuck_count'],'geometric_overlap_samples':leg['summary']['geometric_overlap_samples']})
 tests=(BASE/'tests_final.log').read_text();extra=(BASE/'geometry_extra_tests.log').read_text();cpp=read(BASE/'isolated_cpp_clock_verification.json')
 checks={'three_current_version_sessions_pass':all(s['all_pass'] for s in sessions),'same_92_item_local_runtime_inputs':same and len(sets[0])==92,'current_workspace_matches_frozen_inputs':all(current.values()),'local_elf_and_installed_plugin_snapshots':local_elf,'current_navigation_hold_resume_pass':hold['all_pass'] and all(hold_match.values()),'historical_registered_data_preserved':history['pass'],'offline_arena_bundle_pass':arena['all_pass'] and all(arena_hashes.values()),'119_tool_tests_pass':'Ran 119 tests' in tests and tests.rstrip().endswith('OK'),'missing_geometry_rejection_test_pass':extra.rstrip().endswith('OK'),'actual_cpp_ros_clock_contract_pass':cpp['pass']}
 return {'all_pass':all(checks.values()),'status':'STAGE 1.5 PROVISIONAL TRAINING VALIDATION COMPLETE' if all(checks.values()) else 'STAGE 1.5 VALIDATION INCOMPLETE','checks':checks,'rows':rows,'hold_evidence':hold['hold_evidence'],'hold_gt_error_m':hold['terminal']['gt_error_m'],'hold_runtime_comparison':hold_match,'hold_comparison_note':'Same navigation/config/binary hashes; geometry differs only by restoring original empty-wall ValueError. The actual nonempty arena is unchanged. Hold audit v1 file naming error is superseded by v2; raw evidence unchanged.','history':history,'prior_attempts':prior,'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,'contact':'CONTACT DATA UNAVAILABLE','continuous_time_safety_proven':False,'statistical_reliability_proven':False,'stage2_entered':False,'system_image_complete':False,'reset_monitoring_complete':False,'runtime_input_sha256':sets[0],'input_local_item_count':len(sets[0]),'human_30min_modelling_drill_complete':False,'arena_machine_seconds':arena['machine_seconds'],'assessment_script_sha256':sha(__file__),'command':sys.argv}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--verify-only',action='store_true');p.add_argument('--output',type=Path);a=p.parse_args()
 if not a.verify_only and (a.output is None or a.output.exists()):raise ValueError('Supply a new --output path; overwrite refused')
 report=summarize()
 if not a.verify_only:write(a.output,report)
 print(json.dumps({'all_pass':report['all_pass'],'status':report['status'],'checks':report['checks'],'output':str(a.output) if a.output else None},ensure_ascii=False));sys.exit(0 if report['all_pass'] else 2)
