"""Seal one completed optimization experiment, preserving the prior baseline."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT/'tools/results/stage2_step4_lio_optimization_20261002'
PRIOR = ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002/final_evidence_seal.json'
REPORT = ROOT/'docs/stage2_step4_instantaneous_cloud_optimization_20261002.md'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): digest.update(block)
    return digest.hexdigest()


def read(path): return json.loads(path.read_text())


def save(path, value):
    with path.open('x') as stream: json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)


def errors(mapping):
    return [name for name, value in mapping.items()
            if not (ROOT/name).is_file() or sha(ROOT/name) != value]


def process_evidence():
    ps = subprocess.check_output(['ps','-eo','pid=,comm=,args='], text=True)
    found = []
    for line in ps.splitlines():
        parts = line.split(None,2)
        if len(parts) < 3: continue
        comm, command = parts[1:]
        if comm in ['ruby','ign','gz','rviz2','spark_lio_mappi','spark_fastlio_m','ego_planner_nod','ego_planner_node']:
            found.append(line)
        elif comm in ['python','python3'] and (
                '/install/stage2_lio_sim/' in command or
                command.startswith('/opt/ros/humble/bin/ros2 launch')):
            found.append(line)
    return found


def main():
    p = argparse.ArgumentParser(); p.add_argument('--verify-existing',action='store_true'); a = p.parse_args()
    seal_path = OUT/'final_evidence_seal.json'
    if a.verify_existing:
        seal = read(seal_path)
        mismatches = errors(seal['source_hashes']) + errors(seal['evidence_hashes'])
        changed_targets = [name for name, target in seal['source_resolved_paths'].items()
                           if str((ROOT/name).resolve()) != target]
        print(json.dumps({'pass': not mismatches and not changed_targets,
            'source_files':len(seal['source_hashes']), 'evidence_files':len(seal['evidence_hashes']),
            'mismatches':mismatches, 'changed_symlink_targets':changed_targets},indent=2))
        raise SystemExit(2 if mismatches or changed_targets else 0)
    assert not seal_path.exists(), 'Existing evidence seal must never be overwritten.'
    previous = read(PRIOR)
    assert not errors(previous['source_hashes']) and not errors(previous['evidence_hashes'])
    assert len(previous['source_hashes']) == 233 and len(previous['evidence_hashes']) == 1338
    audit_path = OUT/'full_instant_cloud_01/independent_audit_v2.json'
    audit = read(audit_path)
    assert audit['all_pass'] and len(audit['checks']) == 34 and all(audit['checks'].values())
    versions = audit['post_trial_evaluator_change']
    assert versions['sha256'] == sha(ROOT/'tools/stage2/lio_optimization/audit_instant_cloud_navigation_v2.py')
    assert versions['gravity_module_sha256'] == sha(ROOT/'tools/stage2/lio_optimization/audit_stationary_gravity.py')
    assert audit['instant_cloud_evaluator_sha256'] == sha(ROOT/'tools/stage2/lio_optimization/audit_instant_cloud_navigation.py')
    assert audit['point_set_evaluator_sha256'] == sha(ROOT/'tools/stage2/lio_optimization/audit_all_point_frames.py')
    assert audit['safe_evaluator_sha256'] == sha(ROOT/'tools/stage2/lio_safe_audit/audit_safe_navigation.py')
    progress = read(OUT/'full_instant_cloud_01/progress.json')
    assert progress['launches'] == progress['goals_requested'] == 1 and progress['remaining_gazebo_servers'] == []
    assert read(OUT/'full_instant_cloud_01/independent_audit_v1.json')['all_pass'] is False
    replays = {}
    for name in ['baseline_replay_analysis_final','candidate_replay_analysis_final',
                 'point_boundary_replay_analysis_final','flat_prior_replay_analysis_final','instant_cloud_replay_analysis']:
        r = read(OUT/(name+'.json'))
        assert r['evidence_pass'] and r['evaluator_sha256'] == sha(ROOT/'tools/stage2/lio_optimization/analyze_variance.py')
        assert sha(OUT/(name+'.tool.py')) == r['evaluator_sha256']
        replays[name] = r['replayed']
    point = read(OUT/'instant_cloud_all_points_audit.json')
    assert point['all_points_rigid_consistent'] and point['all_recorded_outputs_checked'] and point['frames'] == 1004
    tests = {}
    for name,count in [('point_evidence_tests.log',6),('variance_parameter_tests_v3.log',5),('stationary_gravity_tests.log',9)]:
        log = (OUT/name).read_text()
        assert 'Ran '+str(count)+' tests' in log and log.rstrip().endswith('OK')
        tests[name] = count
    review = read(OUT/'instant_cloud_full_geometry_review.json')
    assert review['stationary_10_to_15']['pass'] and review['below_0_08m_sample_count'] == 0
    assert REPORT.is_file()
    remaining = process_evidence(); assert not remaining, remaining
    final_diff = subprocess.check_output(['git','diff','--binary'],cwd=ROOT)
    assert final_diff == (OUT/'git_diff_before.patch').read_bytes(), 'Pre-existing tracked dirty work changed.'
    with (OUT/'git_diff_final.patch').open('xb') as stream: stream.write(final_diff)
    with (OUT/'git_status_final.txt').open('x') as stream:
        stream.write(subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True))
    save(OUT/'optimization_completion_audit.json', {
        'all_pass':True, 'status':'STAGE 2 STEP 4 INSTANTANEOUS CLOUD PROCESSING FIX VERIFIED',
        'classification':'TRAINING-ONLY / PROVISIONAL',
        'competition_arena':'COMPETITION ARENA NOT VERIFIED', 'contact':'CONTACT DATA UNAVAILABLE',
        'stage5_started':False, 'original_baseline_source_unchanged':233,
        'original_baseline_evidence_unchanged':1338, 'existing_tracked_dirty_diff_unchanged':True,
        'strict_mission_check_count':34, 'mission_audit':str(audit_path.relative_to(ROOT)),
        'single_Full':audit['csv_terminal'], 'native_geometry':review,
        'corrected_cloud_frames':audit['point_set_evidence']['frames'],
        'test_counts':tests, 'replay_results':replays,
        'rejected_experiments':['reduced observation variance','existing planar-mode state projection'],
        'accepted_scope':'Instantaneous scan measurement correctness; optional new LIO profile. No statistical accuracy claim.',
        'post_trial_audit_correction':versions, 'remaining_simulation_processes':remaining,
        'limitations':['One Full after the fix; no statistical repeatability or continuous-time safety.',
            '4cm pose reserve remains an engineering budget, not a calibrated bound.',
            'GT is offline observer/abort evidence only, never LIO/navigation localization feedback.',
            'Instantaneous simulation is not physical MID-360 scan-pattern/packet/timing verification.',
            'Real nonzero scan-time hardware operation was not tested.',
            'Old FAIL reports, startup data and unsuccessful tooling logs retained.',
            'OS and external libraries were not captured as a complete machine image.']})
    sources = set(previous['source_hashes'])
    sources.update(str(f.relative_to(ROOT)) for f in (ROOT/'tools/stage2/lio_optimization').rglob('*')
                   if f.is_file() and '__pycache__' not in f.parts)
    sources.update([
        'src/stage2_lio_sim/scripts/lio_point_boundary_mapping.py',
        'src/uav_bringup/launch/provincial_stage2_lio_point_boundary.launch.py',
        'install/stage2_lio_sim/lib/stage2_lio_sim/lio_point_boundary_mapping.py',
        'install/uav_bringup/share/uav_bringup/launch/provincial_stage2_lio_point_boundary.launch.py',
        'install/stage2_lio_sim/lib/lio_point_boundary/spark_lio_mapping',
        'install/stage2_lio_sim/lib/lio_point_boundary/libspark_lio_component.so',
        str(REPORT.relative_to(ROOT))])
    production = OUT/'instant_cloud_source'
    declared = read(production/'production_manifest.json')['files']
    assert all(sha(production/name) == value for name,value in declared.items())
    sources.update(str((production/name).relative_to(ROOT)) for name in declared)
    sources.add(str((production/'production_manifest.json').relative_to(ROOT)))
    evidence = set(previous['evidence_hashes'])
    evidence.add(str(PRIOR.relative_to(ROOT)))
    evidence.update(str(f.relative_to(ROOT)) for f in OUT.rglob('*')
                    if f.is_file() and f != seal_path and '__pycache__' not in f.parts)
    save(seal_path, {'source_hashes':{n:sha(ROOT/n) for n in sorted(sources)},
        'source_resolved_paths':{n:str((ROOT/n).resolve()) for n in sorted(sources)},
        'evidence_hashes':{n:sha(ROOT/n) for n in sorted(evidence)},
        'previous_seal':str(PRIOR.relative_to(ROOT)),
        'status':'STAGE 2 STEP 4 INSTANTANEOUS CLOUD PROCESSING FIX VERIFIED',
        'classification':'TRAINING-ONLY / PROVISIONAL', 'stage5_started':False})
    print(json.dumps({'pass':True,'source_files':len(sources),'evidence_files':len(evidence)},indent=2))


if __name__ == '__main__': main()
