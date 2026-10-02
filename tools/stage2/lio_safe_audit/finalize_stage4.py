"""Seal the completed simulation scope without changing recorded trials."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'tools/results/stage2_step4_lio_safe_profile_20261002'
PREVIOUS = ROOT / 'tools/results/stage2_step4_lio_source_and_guard_20261002/final_evidence_seal.json'
REPORT = ROOT / 'docs/stage2_step4_simulated_lio_completion_20261002.md'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)


def verify_hashes(mapping):
    return [name for name, value in mapping.items()
            if not (ROOT / name).is_file() or sha(ROOT / name) != value]


def stale_velocity_evidence():
    """Compare world position differences, avoiding body/world twist confusion."""
    gt, sensor = [], []
    for line in (OUT / 'full_01/lio_full_01.native.jsonl').open():
        row = json.loads(line)
        if row['topic'] == '/gazebo/odometry' and 55 <= row['message_stamp_s'] <= 56:
            pos = row['data']['pose']['pose']['position']
            gt.append([row['message_stamp_s'], pos['x'], pos['y']])
        elif row['topic'] == '/simulation/imu_kinematics':
            data = json.loads(row['data']['data'])
            if 55 <= data['stamp_s'] <= 56:
                sensor.append(data['world_velocity'])
    assert len(gt) >= 45 and len(sensor) >= 240
    speeds = [math.dist(a[1:], b[1:]) / (b[0] - a[0]) for a, b in zip(gt, gt[1:])]
    return {'window_sim_s': [55, 56], 'GT_raw_position_samples': len(gt),
            'physical_cached_velocity_samples': len(sensor),
            'maximum_GT_position_difference_speed_mps': max(speeds),
            'minimum_cached_world_xy_speed_mps': min(math.hypot(*v[:2]) for v in sensor),
            'scope': 'World position differences versus world sensor velocity; no body-frame twist compared as world axes.',
            'supersedes': 'velocity_component_vs_truth.json moving-frame maximum discrepancy; that initial comparison mixed body and world twist axes.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--verify-existing', action='store_true')
    args = parser.parse_args()
    seal_path = OUT / 'final_evidence_seal.json'
    if args.verify_existing:
        seal = read(seal_path)
        errors = verify_hashes(seal['source_hashes']) + verify_hashes(seal['evidence_hashes'])
        print(json.dumps({'pass': not errors, 'source_files': len(seal['source_hashes']),
                          'evidence_files': len(seal['evidence_hashes']), 'mismatches': errors}, indent=2))
        raise SystemExit(2 if errors else 0)
    assert not seal_path.exists(), 'Do not overwrite an existing seal.'
    previous = read(PREVIOUS)
    old_errors = verify_hashes(previous['source_hashes']) + verify_hashes(previous['evidence_hashes'])
    assert not old_errors, old_errors
    trials = {}
    for name in ['full_reserved_map_01', 'short_reserved_map_01']:
        audit = read(OUT / name / 'independent_audit_v2.json')
        assert audit['all_pass'] and len(audit['checks']) == 31 and all(audit['checks'].values())
        assert audit['safe_evaluator_sha256'] == sha(ROOT / 'tools/stage2/lio_safe_audit/audit_safe_navigation.py')
        assert audit['input_history_evaluator_sha256'] == sha(ROOT / 'tools/stage2/lio_source_guard_audit/audit_input_history.py')
        progress = read(OUT / name / 'progress.json')
        assert progress['launches'] == 1 and progress['goals_requested'] == 1
        assert progress['remaining_gazebo_servers'] == []
        t = audit['csv_terminal']
        trials[name] = {'pass': True, 'top_level_check_count': len(audit['checks']),
            'GT_error_m': t['gt_error_m'], 'odom_error_m': t['odom_error_m'],
            'native_minimum': audit['native']['minimum'], 'duration_sim_s': t['duration_sim_s'],
            'retry_count': t['retry_count'], 'stuck_count': t['stuck_count'],
            'full_2_plus_5': audit['native_terminal']['full_7s'],
            'waypoints': audit['waypoint']['completed_ids'],
            'audit_path': str((OUT / name / 'independent_audit_v2.json').relative_to(ROOT))}
    review = read(OUT / 'full_native_geometry_review_v2.json')
    assert review['stationary_10_to_15']['pass'] and review['below_0_08m_sample_count'] == 0
    for name, count in [('isolated_tests_v5.log', 17), ('physical_mutation_tests_v2.log', 6), ('original_guard_tests.log', 6)]:
        log = (OUT / name).read_text()
        assert 'Ran '+str(count)+' tests' in log and log.rstrip().endswith('OK')
    # Check executable names/argument prefixes, not substrings inside this Python command.
    ps = subprocess.check_output(['ps', '-eo', 'pid=,comm=,args='], text=True)
    relevant = []
    for line in ps.splitlines():
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        comm, command = parts[1:]
        if comm in ['ruby', 'ign', 'gz', 'rviz2', 'spark_fastlio_m', 'spark_fastlio_mapping', 'ego_planner_node']:
            relevant.append(line)
        elif comm in ['python3', 'python'] and ('/install/stage2_lio_sim/' in command or command.startswith('/opt/ros/humble/bin/ros2 launch')):
            relevant.append(line)
    assert not relevant, relevant
    save(OUT / 'stale_velocity_stationary_world_audit_v2.json', stale_velocity_evidence())
    (OUT / 'git_status_final.txt').write_text(subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True))
    (OUT / 'git_diff_final.patch').write_bytes(subprocess.check_output(['git', 'diff', '--binary'], cwd=ROOT))
    save(OUT / 'stage4_completion_audit.json', {
        'all_pass': True,
        'status': 'STAGE 2 STEP 4 SIMULATED LIO CLOSED-LOOP VALIDATION COMPLETE',
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'competition_arena': 'COMPETITION ARENA NOT VERIFIED',
        'contact': 'CONTACT DATA UNAVAILABLE', 'stage5_started': False,
        'previous_source_files_unchanged': len(previous['source_hashes']),
        'previous_evidence_files_unchanged': len(previous['evidence_hashes']),
        'final_trials': trials, 'stationary': review['stationary_10_to_15'],
        'tool_tests': {'production_safety': 17, 'physical_mutations': 6, 'original_LIO_watchdogs': 6},
        'remaining_simulation_processes': relevant,
        'limitations': [
            'Instantaneous simulated 3D lidar uses MID-360-compatible fields; actual scan pattern, packets and per-point timing are not validated.',
            'One final Full and one short trial do not prove statistical repeatability.',
            '4cm pose reserve is an engineering budget, not a calibrated bound; observed maximum XY deviation is about 5.49cm.',
            'Completion GT error about 4.71cm leaves about 2.87mm against the 5cm completion cap.',
            'GT is offline observer/abort evidence only; no GT localization feedback.',
            'Full startup sensor receiving gaps remain recorded; task window continuity passes, full-recording continuity does not.',
            'Raw lidar minimum-gap check is shadow only; scan validity, planning obstacles and stronger map execution guard remain active.',
            'Sampled geometry is neither independent contact evidence nor continuous-time safety.',
            'Run code/logs/one subscribed external goal support no reset; all reset/set_pose service calls were not independently monitored.',
            'OS and third-party system dependencies are not a complete machine image.'
        ]})
    assert REPORT.is_file()
    sources = set(previous['source_hashes'])
    for directory in ['tools/stage2/lio_safe_validation', 'tools/stage2/lio_safe_audit']:
        sources.update(str(p.relative_to(ROOT)) for p in (ROOT / directory).rglob('*')
                       if p.is_file() and '__pycache__' not in p.parts)
    sources.update([
        'src/stage2_lio_sim/scripts/lio_safety_geometry.py',
        'src/stage2_lio_sim/scripts/lio_safe_navigation_interface.py',
        'src/uav_bringup/launch/provincial_stage2_lio_safe.launch.py',
        'src/uav_bringup/worlds/provincial_stage2_lio_position_imu.sdf',
        'install/stage2_lio_sim/lib/stage2_lio_sim/lio_safe_navigation_interface.py',
        'install/stage2_lio_sim/lib/stage2_lio_sim/lio_safety_geometry.py',
        'install/stage2_lio_sim/lib/libstage2_position_imu.so',
        'install/uav_bringup/share/uav_bringup/launch/provincial_stage2_lio_safe.launch.py',
        'install/uav_bringup/share/uav_bringup/worlds/provincial_stage2_lio_position_imu.sdf',
        str(REPORT.relative_to(ROOT))])
    evidence = set(previous['evidence_hashes'])
    evidence.add(str(PREVIOUS.relative_to(ROOT)))
    evidence.update(str(p.relative_to(ROOT)) for p in OUT.rglob('*')
                    if p.is_file() and p != seal_path and '__pycache__' not in p.parts)
    save(seal_path, {'source_hashes': {n: sha(ROOT / n) for n in sorted(sources)},
                     'evidence_hashes': {n: sha(ROOT / n) for n in sorted(evidence)},
                     'stage4_simulation_scope_complete': True,
                     'previous_seal': str(PREVIOUS.relative_to(ROOT)),
                     'classification': 'TRAINING-ONLY / PROVISIONAL',
                     'contact': 'CONTACT DATA UNAVAILABLE', 'stage5_started': False})
    print(json.dumps({'pass': True, 'sources': len(sources), 'evidence': len(evidence), 'seal': str(seal_path)}))


if __name__ == '__main__':
    main()
