#!/usr/bin/env python3
"""Repeatable offline verification of this isolated fix, never a live verdict."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'tools/results/stage2_localization_hold_retry_fix_20261001'
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def verify():
    before=read(OUT/'counterexample_before.json');after=read(OUT/'counterexample_after_final.json')
    history=read(OUT/'historical_reaudit_v3.json');plan=read(OUT/'unsafe_plan_review_v1.json')
    inputs=read(OUT/'test_input_sha256_before.json')
    source=ROOT/'src/uav_planning/scripts/stage2_localized_interface.py'
    installed=ROOT/'install/uav_planning/lib/uav_planning/stage2_localized_interface.py'
    archived=ROOT/'tools/results/stage2_scan_resume_20261001'
    raw_hashes=read(archived/'raw_data_sha256.json')
    prior=read(ROOT/'tools/results/stage2_scan_resume_tools_20261001/final_evidence_seal.json')
    frozen=read(ROOT/'tools/results/provincial_stage15_completion_20261001/provisional_stage15_completion_audit_v3.json')['runtime_input_sha256']
    frozen_check={p:sha(ROOT/p)==v for p,v in frozen.items()}
    suite=(OUT/'tests_combined_final.log').read_text()
    match=re.search(r'Ran (\d+) tests',suite)
    ps=subprocess.check_output(['ps','-eo','pid,args'],text=True)
    processes=[line for line in ps.splitlines() if ('ign gazebo -s -r' in line or
        '/lib/uav_planning/stage2_' in line or '/lib/ego_planner/ego_planner_node' in line) and
        not any(skip in line for skip in ('bash -lc','grep','python3 -c'))]
    checks={
      'before_real_internal_retry_reproduced':before['retry_count']==1 and len(before['internal_publications'])==1,
      'after_no_retry_or_internal_publish':after['retry_count']==0 and after['internal_publications']==[],
      'nonzero_upstream_stops_final_output':after['final_output']==[{'vx':0.,'vy':0.,'wz':0.}],
      'odom_and_active_task_kept':after['odom_publications']==1 and after['waypoint_id']==before['waypoint_id'] and after['pending']==before['pending'],
      'all_77_combined_tests_pass':match is not None and int(match.group(1))==77 and suite.rstrip().endswith('OK'),
      'test_input_41_item_closure_unchanged':len(inputs)==41 and all(sha(ROOT/p)==v and sha(OUT/'test_input_snapshot'/p)==v for p,v in inputs.items()),
      'stage15_frozen_92_unchanged':len(frozen_check)==92 and all(frozen_check.values()),
      'source_install_identical':sha(source)==sha(installed),
      'old_raw_hashes_unchanged':all(sha(archived/p)==v for p,v in raw_hashes.items()),
      'all_previous_sealed_evidence_unchanged':all(sha(ROOT/p)==v for p,v in prior['evidence_hashes'].items()),
      'historical_strict_fail_retained':history['all_pass'] is False and history['summary']['retry_count']==1 and history['loss_resume']['pass'] is True,
      'unsafe_plan_shape_and_rejection_retained':plan['all_marker_shapes_match_unique_raw_spline'] and
          plan['plans'][-1]['matching_raw_splines'][0]['traj_id']==6 and
          plan['plans'][-1]['production_callback_min_gap_m']==0 and
          plan['plans'][-1]['production_callback_safe'] is False and
          all(d['action']=='unsafe_plan' and d['vx']==d['vy']==d['wz']==0 for d in plan['plans'][-1]['following_raw_guard']),
      'no_remaining_simulation_processes':not processes,
    }
    manifest=read(archived/'input_manifest.json')
    return dict(scope_complete=all(checks.values()),checks=checks,
       status='STAGE2 LOCALIZATION-HOLD RETRY ISOLATED FIX VERIFIED' if all(checks.values()) else 'ISOLATED FIX INCOMPLETE',
       new_gazebo_launches=0,new_external_goals=0,live_new_fix_verified=False,
       frozen92=frozen_check,source_sha256=sha(source),installed_sha256=sha(installed),installed_resolved_path=str(installed.resolve()),
       current_workspace_differs_from_old_runtime=[p for p,v in manifest['inputs'].items() if sha(ROOT/p)!=v['sha256']],
       remaining_simulation_processes=processes,evaluator_sha256=sha(Path(__file__)),
       classification='TRAINING-ONLY / PROVISIONAL',competition_arena_verified=False,contact='CONTACT DATA UNAVAILABLE')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    r=verify();a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'scope_complete':r['scope_complete'],'status':r['status'],'checks':r['checks'],'old_runtime_differences':r['current_workspace_differs_from_old_runtime']},ensure_ascii=False))
    raise SystemExit(0 if r['scope_complete'] else 2)
