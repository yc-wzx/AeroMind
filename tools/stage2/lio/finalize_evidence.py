"""Seal both success and failure; never promotes runner success to acceptance."""
import json,hashlib,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'tools/results/stage2_step4_lio_20261001'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def load(p):return json.loads(p.read_text())
def write(p,r):
    with p.open('x') as f:json.dump(r,f,indent=2)

def main():
    prior=load(ROOT/'tools/results/stage2_scan_capture_startup_fix_20261001/final_evidence_seal.json')
    frozen=load(ROOT/'tools/results/provincial_stage15_completion_20261001/provisional_stage15_completion_audit_v3.json')['runtime_input_sha256']
    preservation={k:{'count':len(prior[k]),'unchanged':all(sha(ROOT/p)==h for p,h in prior[k].items())} for k in ['source_hashes','evidence_hashes']}
    preservation['stage15_inputs']={'count':len(frozen),'unchanged':all(sha(ROOT/p)==h for p,h in frozen.items())}
    full=load(OUT/'full_01/independent_audit_v1.json');short=load(OUT/'short_05/independent_audit_v3.json')
    static=load(OUT/'short_review_v1/review.json')['latest_code_static_10_to_15s'];tests=load(OUT/'mutation_tests_v1.json')
    manifest=load(OUT/'full_01/input_manifest.json')
    current_mismatch=[p for p,v in manifest['inputs'].items() if sha(ROOT/p)!=v['sha256'] or str((ROOT/p).resolve())!=v['resolved_path']]
    production_pairs={}
    for p in (ROOT/'src/stage2_lio_sim/scripts').glob('*.py'):
        installed=ROOT/'install/stage2_lio_sim/lib/stage2_lio_sim'/p.name
        production_pairs[str(p.relative_to(ROOT))]={'installed':str(installed.relative_to(ROOT)),
            'installed_resolved':str(installed.resolve()),'sha256':sha(p),'match':sha(p)==sha(installed)}
    for path in ['launch/provincial_stage2_lio.launch.py','worlds/provincial_stage2_lio.sdf','config/spark_provincial_sim.yaml']:
        p=ROOT/'src/uav_bringup'/path;i=ROOT/'install/uav_bringup/share/uav_bringup'/path
        production_pairs[str(p.relative_to(ROOT))]={'installed':str(i.relative_to(ROOT)),
            'installed_resolved':str(i.resolve()),'sha256':sha(p),'match':sha(p)==sha(i)}
    processes=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
    leftovers=[line for line in processes if any(t in line for t in ['ign gazebo','ignition-gazebo-server','spark_lio_mapping','lio_sensor_adapter.py','lio_odometry_adapter.py','lio_navigation_interface.py','run_navigation_trial.py']) and 'finalize_evidence.py' not in line and 'bash -lc' not in line]
    evidence_checks={'preserved_frozen_and_prior':all(v['unchanged'] for v in preservation.values()),
        'Full_runtime_inputs_still_match':not current_mismatch,'source_install_match':all(v['match'] for v in production_pairs.values()),
        'static_latest_code_pass':static['pass'],'short_strict_pass':short['all_pass'],
        'counterexamples_rejected':tests['pass'],'no_experiment_processes':not leftovers}
    review={'status':'LIO INTEGRATION READY / FULL ACCEPTANCE NOT PASSED',
        'implementation_ready':True,'specified_integration_and_validation_executed':True,
        'strict_step4_acceptance_complete':False,'checks_except_Full_acceptance':evidence_checks,
        'Full_strict_pass':full['all_pass'],'Full_failed_checks':[k for k,v in full['checks'].items() if not v],
        'Full_Gazebo_count':1,'Full_external_goal_count':1,'Full_automatic_retries':0,
        'Full_completion_GT_error_m':full['summary']['navigation_completion']['gt_goal_error_m'],
        'Full_stop_snapshot_GT_error_m':full['summary']['gt_goal_error_m'],
        'Full_native_sample_minimum':full['native']['minimum'],'full_2_plus_5_stop_windows':full['native_terminal']['full_7s']['pass'],
        'preservation':preservation,'Full_input_count':len(manifest['inputs']),
        'current_input_mismatch':current_mismatch,'source_install':production_pairs,'remaining_processes':leftovers,
        'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,
        'contact':'CONTACT DATA UNAVAILABLE','continuous_time_safety_proven':False,'statistical_reliability_proven':False,
        'step5_entered':False,'next_minimal_step':'Offline sealed-sensor replay and LIO timing/height/pose diagnosis; no GT feedback, no planner or threshold changes.',
        'report':'docs/stage2_step4_lio_simulation_20261001.md'}
    write(OUT/'final_review.json',review)
    sources={p:sha(ROOT/p) for p in manifest['inputs']}
    for folder in ['src/stage2_lio_sim','tools/stage2/lio']:
        for p in (ROOT/folder).rglob('*'):
            if p.is_file() and '__pycache__' not in str(p):sources[str(p.relative_to(ROOT))]=sha(p)
    for name in ['docs/stage2_step4_lio_simulation_20261001.md','tools/config/stage2_step4_lio_20261001.yaml']:
        sources[name]=sha(ROOT/name)
    evidence={str(p.relative_to(ROOT)):sha(p) for p in OUT.rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    write(OUT/'final_evidence_seal.json',{'source_hashes':sources,'evidence_hashes':evidence,
        'preservation':preservation,'strict_step4_acceptance_complete':False,'Full_strict_pass':False,
        'no_full_retries':True,'classification':'TRAINING-ONLY / PROVISIONAL'})
    print(json.dumps({'checks_except_Full_acceptance':evidence_checks,'Full_strict_pass':False,
        'source_count':len(sources),'evidence_count':len(evidence),'status':review['status']},indent=2))
    if not all(evidence_checks.values()):raise SystemExit(2)

if __name__=='__main__':main()
