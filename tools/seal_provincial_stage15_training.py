#!/usr/bin/env python3
"""Seal assessed evidence and documentary artifacts; never launches ROS."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/results/provincial_stage15_completion_20261001'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text())
target=BASE/'final_training_seal.json'
if target.exists():raise FileExistsError(target)
a=read(BASE/'provisional_stage15_completion_audit_v3.json');b=read(BASE/'final_repeated_verify.log')
logs=['tests_final.log','geometry_extra_tests.log','hold_independent_evidence_tests.log']
checks={'independent_audit_pass':a['all_pass'],'repeated_readonly_judgment_same':b['all_pass'] and b['checks']==a['checks'],'all_additional_tool_tests_pass':all((BASE/p).read_text().rstrip().endswith('OK') for p in logs),'frozen_runtime_files_still_match':all(sha(ROOT/p)==v for p,v in a['runtime_input_sha256'].items())}
paths=[BASE/'provisional_stage15_completion_audit_v3.json',BASE/'final_repeated_verify.log',BASE/'hold_clock_recheck/independent_hold_audit_v3.json',ROOT/'tools/config/provincial_stage15_training_frozen_20261001.yaml',ROOT/'docs/provincial_stage15_training_completion_20261001.md',ROOT/'docs/provincial_stage15_race_day_30min_20261001.md',ROOT/'docs/provincial_stage15_frozen_next_prompt_20261001.md',BASE/'incremental_runtime_and_tools.patch',BASE/'final_plots/forward_return_native_gt.png',ROOT/'tools/summarize_provincial_stage15_completion.py',ROOT/'tools/provincial_stage15_hold_clock_recheck.py',Path(__file__)]+[BASE/p for p in logs]
report={'all_pass':all(checks.values()),'status':a['status'] if all(checks.values()) else 'STAGE 1.5 VALIDATION INCOMPLETE','checks':checks,'artifact_sha256':{str(p.relative_to(ROOT)):sha(p) for p in paths},'competition_arena_verified':False,'stage2_entered':False,'contact':'CONTACT DATA UNAVAILABLE','new_simulation_matrix_frozen':True}
target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'all_pass':report['all_pass'],'status':report['status'],'checks':checks},ensure_ascii=False));raise SystemExit(0 if report['all_pass'] else 2)
