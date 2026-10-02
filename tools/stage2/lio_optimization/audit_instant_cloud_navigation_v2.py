"""Recompute mission and point-set evidence; gravity needs stationary evidence.

Preserve v1 (FAIL) and the unchanged older evaluator. The one changed semantic
check retains the old 0.2 tolerance and additionally requires every stationary
sample, raw stationarity/continuity and the existing dynamic force audit.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from audit_instant_cloud_navigation import evaluate as original_evaluate
from audit_stationary_gravity import evaluate as gravity_evaluate


def evaluate(directory, point_output):
    result = original_evaluate(directory, point_output)
    sensor = result['sensor']
    result['legacy_moving_norm_sensor_audit'] = dict(sensor)
    gravity = gravity_evaluate(directory)
    result['stationary_gravity'] = gravity
    sensor['checks'] = dict(sensor['checks'])
    sensor['checks']['specific_force_includes_gravity'] = (
        gravity['pass'] and result['LIO_source']['checks']['IMU_specific_force_matches_kinematics_and_declared_noise'])
    sensor['pass'] = all(sensor['checks'].values())
    result['checks']['three_dimensional_sensor_evidence'] = sensor['pass']
    result['all_pass'] = all(result['checks'].values())
    result['post_trial_evaluator_change'] = {
        'old_statistic_preserved': 'legacy_moving_norm_sensor_audit',
        'reason': 'Specific force norm includes dynamic acceleration; whole moving-record norm is not gravity.',
        'navigation_thresholds_changed': False, 'gravity_tolerance_changed': False,
        'runtime_data_or_code_changed': False,
        'version': 'instant-cloud-v2-fixed-stationary-gravity',
        'command': sys.argv,
        'sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'gravity_module_sha256': hashlib.sha256(Path(__file__).with_name('audit_stationary_gravity.py').read_bytes()).hexdigest()}
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--result-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--point-output', type=Path, required=True)
    a = p.parse_args()
    try: r = evaluate(a.result_dir, a.point_output)
    except Exception as e: r = {'all_pass': False, 'status': 'INCOMPLETE_EVIDENCE', 'error': repr(e)}
    with a.output.open('x') as f: json.dump(r,f,indent=2,allow_nan=False)
    print(json.dumps({k:r.get(k) for k in ['all_pass','checks','stationary_gravity','error']},indent=2))
    raise SystemExit(0 if r['all_pass'] else 2)
