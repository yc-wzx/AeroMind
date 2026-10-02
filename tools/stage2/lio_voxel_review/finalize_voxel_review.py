"""Seal the accepted LIO voxel optimization without rewriting older evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/stage2/lio_optimization'))
from finalize_optimization import sha, errors, process_evidence
OUT=ROOT/'tools/results/stage2_step4_lio_voxel_review_20261002'
PRIOR=ROOT/'tools/results/stage2_step4_lio_optimization_20261002/final_evidence_seal.json'
REPORT=ROOT/'docs/stage2_step4_lio_fine_map_optimization_20261002.md'


def read(p):return json.loads(p.read_text())


def save(p,r):
    with p.open('x') as f:json.dump(r,f,indent=2,ensure_ascii=False,allow_nan=False)


def main():
    p=argparse.ArgumentParser();p.add_argument('--verify-existing',action='store_true');a=p.parse_args()
    sealpath=OUT/'final_evidence_seal.json'
    if a.verify_existing:
        s=read(sealpath);bad=errors(s['source_hashes'])+errors(s['evidence_hashes'])
        targets=[n for n,t in s['source_resolved_paths'].items() if str((ROOT/n).resolve())!=t]
        print(json.dumps({'pass':not bad and not targets,'source_files':len(s['source_hashes']),
            'evidence_files':len(s['evidence_hashes']),'mismatches':bad,'changed_symlink_targets':targets},indent=2))
        raise SystemExit(2 if bad or targets else 0)
    assert not sealpath.exists(), 'Never overwrite a seal.'
    previous=read(PRIOR)
    assert len(previous['source_hashes'])==317 and len(previous['evidence_hashes'])==2184
    assert not errors(previous['source_hashes']) and not errors(previous['evidence_hashes'])
    assert all(str((ROOT/n).resolve())==t for n,t in previous['source_resolved_paths'].items())
    gate=read(OUT/'offline_candidate_gate.json')
    assert gate['offline_candidate_accepted'] and all(all(v.values()) for v in gate['checks'].values())
    assert gate['evaluator_sha256']==sha(Path(__file__).with_name('compare_and_gate.py'))
    assert gate['predeclared_plan_sha256']==sha(OUT/'experiment_plan.json')
    for values in gate['comparisons'].values():assert not errors(values['report_sha256'])
    run=OUT/'full_fine_map_01';audit=read(run/'independent_audit_v1.json')
    assert audit['all_pass'] and len(audit['checks'])==36 and all(audit['checks'].values())
    assert audit['fine_map_evaluator_sha256']==sha(Path(__file__).with_name('audit_fine_map_navigation.py'))
    assert audit['post_trial_evaluator_change']['sha256']==sha(ROOT/'tools/stage2/lio_optimization/audit_instant_cloud_navigation_v2.py')
    assert audit['point_set_evaluator_sha256']==sha(ROOT/'tools/stage2/lio_optimization/audit_all_point_frames.py')
    inputs=read(run/'input_manifest.json')
    gravity_key='tools/stage2/lio_optimization/audit_instant_cloud_navigation_v2.py'
    assert inputs['captured_before_goal'] and inputs['inputs'][gravity_key]['sha256']==sha(ROOT/gravity_key)
    progress=read(run/'progress.json')
    assert progress['launches']==progress['goals_requested']==1 and progress['remaining_gazebo_servers']==[]
    log=(OUT/'comparison_tests_v2.log').read_text();assert 'Ran 8 tests' in log and log.rstrip().endswith('OK')
    review=read(OUT/'fine_map_full_geometry_review.json')
    assert review['below_0_08m_sample_count']==0 and review['stationary_10_to_15']['pass']
    assert REPORT.is_file()
    remaining=process_evidence();assert not remaining,remaining
    diff=subprocess.check_output(['git','diff','--binary'],cwd=ROOT)
    assert diff==(OUT/'git_diff_before.patch').read_bytes(), 'Existing tracked dirty work changed.'
    with (OUT/'git_diff_final.patch').open('xb') as f:f.write(diff)
    with (OUT/'git_status_final.txt').open('x') as f:
        f.write(subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True))
    save(OUT/'optimization_completion_audit.json',{'all_pass':True,
        'status':'STAGE 2 STEP 4 LIO FINE MAP OPTIMIZATION VERIFIED',
        'classification':'TRAINING-ONLY / PROVISIONAL',
        'competition_arena':'COMPETITION ARENA NOT VERIFIED','contact':'CONTACT DATA UNAVAILABLE',
        'stage5_started':False,'parameter_change':audit['LIO_parameter_changes'],
        'old_source_files_unchanged':317,'old_evidence_files_unchanged':2184,
        'existing_stationary_gravity_evaluator_captured_before_this_trial':True,
        'existing_tracked_dirty_work_unchanged':True,'mission_check_count':36,
        'offline_gate':gate,'single_full_terminal':audit['csv_terminal'],
        'native_geometry':review,'corrected_output_cloud_frames':audit['point_set_evidence']['frames'],
        'tests':{'comparison_and_parameter_serialization':8},
        'new_optional_simulation_entry':'provincial_stage2_lio_fine_map.launch.py',
        'remaining_processes':remaining,
        'limitations':['Two recorded datasets, one new closed-loop Full; no statistical repeatability or hardware claim.',
            'Only LIO internal voxel size changed; global map, geometry, planner, controller and thresholds unchanged.',
            'GT used for offline evidence/abort, never LIO or navigation localization feedback.',
            '4cm reserve remains an engineering budget, not a calibrated error bound.',
            'Sampled separation does not prove continuous-time or contact safety.',
            'Whole replay job resource counters are not isolated LIO cost or a hardware benchmark.',
            'Initialization evidence and early tool diagnostics retained; no complete OS image or complete reset service trace.']})
    sources=set(previous['source_hashes'])
    sources.update(str(f.relative_to(ROOT)) for f in (ROOT/'tools/stage2/lio_voxel_review').rglob('*')
                   if f.is_file() and '__pycache__' not in f.parts)
    sources.update(['src/uav_bringup/launch/provincial_stage2_lio_fine_map.launch.py',
        'install/uav_bringup/share/uav_bringup/launch/provincial_stage2_lio_fine_map.launch.py',str(REPORT.relative_to(ROOT))])
    evidence=set(previous['evidence_hashes']);evidence.add(str(PRIOR.relative_to(ROOT)))
    evidence.update(str(f.relative_to(ROOT)) for f in OUT.rglob('*')
                    if f.is_file() and f!=sealpath and '__pycache__' not in f.parts)
    save(sealpath,{'source_hashes':{n:sha(ROOT/n) for n in sorted(sources)},
        'source_resolved_paths':{n:str((ROOT/n).resolve()) for n in sorted(sources)},
        'evidence_hashes':{n:sha(ROOT/n) for n in sorted(evidence)},'previous_seal':str(PRIOR.relative_to(ROOT)),
        'status':'STAGE 2 STEP 4 LIO FINE MAP OPTIMIZATION VERIFIED',
        'classification':'TRAINING-ONLY / PROVISIONAL','stage5_started':False})
    print(json.dumps({'pass':True,'source_files':len(sources),'evidence_files':len(evidence)},indent=2))


if __name__=='__main__':main()
