"""Strict task audit plus all-input source history and physical-position IMU."""
import argparse
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools/stage2/lio_source_guard_audit'))
from audit_input_history import audit_rows
path = ROOT/'tools/stage2/lio_model_validation_v2/audit_model_navigation.py'
spec = importlib.util.spec_from_file_location('preserved_model', path)
original = importlib.util.module_from_spec(spec); spec.loader.exec_module(original)


def physical_positions(rows):
    phys, truth = {}, {}
    receive_order_bad = []
    previous_stamp = -math.inf
    for row in rows:
        if row['topic'] == '/simulation/imu_kinematics':
            item = json.loads(row['data']['data'])
            if not math.isfinite(item['stamp_s']) or item['stamp_s'] <= previous_stamp:
                receive_order_bad.append(item['stamp_s'])
            previous_stamp = item['stamp_s']
            phys[round(item['stamp_s'], 8)] = item
        if row['topic'] == '/gazebo/odometry':
            truth[round(row['message_stamp_s'], 8)] = row
    errors, velocities, accelerations, bad = [], [], [], list(receive_order_bad)
    ordered = sorted(phys)
    for t in ordered:
        p = phys[t]
        required = [*p.get('world_position', []), *p['world_velocity'], *p['world_acceleration'], p['dt_s']]
        if (len(p.get('world_position', [])) != 3 or len(p['world_velocity']) != 3 or
                len(p['world_acceleration']) != 3 or p['dt_s'] <= 0 or
                not all(math.isfinite(v) for v in required)):
            bad.append(t)
            continue
        if t in truth:
            g = truth[t]['data']['pose']['pose']['position']
            if not all(math.isfinite(g[k]) for k in 'xy'):
                bad.append(t)
                continue
            errors.append(max(abs(p['world_position'][i]-g[k]) for i, k in enumerate('xy')))
    for a, b in zip(ordered, ordered[1:]):
        old, new = phys[a], phys[b]
        if 'world_position' not in old or 'world_position' not in new:
            continue
        dt = b-a
        if abs(dt-new['dt_s']) > 1e-8:
            bad.append(b)
            continue
        velocities.append(max(abs((y-x)/dt-v) for x, y, v in
            zip(old['world_position'], new['world_position'], new['world_velocity'])))
        accelerations.append(max(abs((y-x)/dt-v) for x, y, v in
            zip(old['world_velocity'], new['world_velocity'], new['world_acceleration'])))
    return {'pass': not bad and len(errors) > 100 and len(velocities) > 100 and
            max(errors) < 1e-5 and max(velocities) < 1e-5 and max(accelerations) < 1e-5,
            'matched_physical_GT_position_samples': len(errors),
            'consecutive_position_velocity_pairs': len(velocities),
            'max_physical_GT_xy_residual_m': max(errors) if errors else None,
            'max_position_derivative_velocity_residual_mps': max(velocities) if velocities else None,
            'max_velocity_derivative_acceleration_residual_mps2': max(accelerations) if accelerations else None,
            'bad_or_missing_position_stamps': bad[:30],
            'scope': 'GT used only by this offline audit; physical sensor derives positions inside Gazebo, not from ROS GT.'}


def reserved_guard(rows, walls):
    from provincial_safety_geometry import rectangle, polygon_distance
    from collections import defaultdict
    source = defaultdict(list)
    for row in rows:
        if row['topic'] == '/localization/lio_navigation_odometry':
            source[round(row['message_stamp_s'], 8)].append(row)
    failures = []
    residuals, tested = [], 0
    for row in rows:
        if row['topic'] != '/lio/safety_diagnostics': continue
        d = json.loads(row['data']['data'])
        if d.get('sampled_reserved_map_gap_m') is None: continue
        t = d['LIO_pose_stamp_s']
        poses = []
        for raw in source[round(t, 8)]:
            p = raw['data']['pose']['pose']; q = p['orientation']
            poses.append([p['position']['x'], p['position']['y'],
                math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))])
        position = d['latest_LIO_pose_for_map_guard']
        if (not poses or not all(math.isfinite(v) for v in position) or
                min(max(abs(a-b) for a, b in zip(position, p)) for p in poses) > 1e-9):
            failures.append({'sim_s': d['sim_s'], 'reason': 'guard pose not matched to raw LIO input'})
            continue
        x, y, yaw = position
        vx, vy, wz = d['requested']
        gap = math.inf
        for _ in range(11):
            body = rectangle(x, y, yaw, .52, .42)
            gap = min(gap, *(polygon_distance(body, wall) for wall in walls))
            mid = yaw+wz*.025/2
            x += (math.cos(mid)*vx-math.sin(mid)*vy)*.025
            y += (math.sin(mid)*vx+math.cos(mid)*vy)*.025
            yaw += wz*.025
        age = d['sim_s']-t
        motion = math.hypot(vx, vy)+math.hypot(.26,.21)*abs(wz)
        required = .08+.04+motion*(max(0.,age)+.02+.0125)
        residuals.append(max(abs(gap-d['sampled_reserved_map_gap_m']),
                             abs(required-d['required_map_gap_m'])))
        tested += 1
        if (d['raw_sensor_gap_mode'] != 'diagnostic_only' or
                residuals[-1] > 1e-9 or
                (d['action'] == 'allowed' and gap < required) or
                (d['action'] != 'allowed' and d['final'] != [0.,0.,0.])):
            failures.append({'sim_s': d['sim_s'], 'reason': 'geometry/budget/output disagreement'})
    return {'pass': tested > 100 and not failures, 'diagnostic_decisions_recomputed': tested,
            'maximum_geometry_or_budget_residual': max(residuals) if residuals else None,
            'failures': failures[:30],
            'scope': 'Independent raw-LIO anchor + SDF polygon sweep and budget; diagnostics are throttled, not every final publication. No GT feedback or continuous-time guarantee.'}


def evaluate(directory):
    result = original.evaluate(directory)
    phase = json.loads((directory/'experiment_metadata.json').read_text())['stage4_phase']
    name = 'lio_'+phase+'_01'
    rows = [json.loads(l) for l in (directory/(name+'.native.jsonl')).open()]
    history = audit_rows(rows)
    before = result['LIO_source']
    checks = dict(before['checks']); checks.pop('actual_output_causal_velocity_recomputed')
    checks['actual_input_history_causal_velocity_recomputed'] = history['pass']
    result['LIO_source_original_output_only'] = before
    result['LIO_source'] = {'pass': all(checks.values()), 'checks': checks, 'input_history': history}
    result['checks']['actual_LIO_and_physical_IMU_source_recomputed'] = result['LIO_source']['pass']
    position = physical_positions(rows)
    result['physical_position_IMU'] = position
    result['checks']['IMU_velocity_from_actual_physical_positions'] = position['pass']
    params = next(iter(yaml.safe_load((directory/'runtime_parameters/gazebo_navigation_interface.yaml').read_text()).values()))['ros__parameters']
    result['checks']['independent_LIO_safety_profile'] = params.get('lio_planning_wall_padding_m') == .075 and params.get('lio_sensor_gap_reserve_m') == .035
    snapshot = directory/'input_snapshot/src/uav_bringup/worlds'
    old = (snapshot/'provincial_stage2_lio.sdf').read_text()
    new = (snapshot/'provincial_stage2_lio_position_imu.sdf').read_text()
    result['checks']['world_change_only_IMU_plugin'] = new.replace('libstage2_position_imu.so', 'libstage2_kinematic_imu.so').replace('stage2::PositionImu', 'stage2::KinematicImu') == old
    boundary = json.loads((directory/'capture_callback_boundary.json').read_text())
    ledger = history['counts']
    expected_boundary = {'inputs': ledger['raw'], 'dispositions': ledger['events'],
        'outputs': ledger['outputs'], 'accepted': ledger.get('dispositions', {}).get('accepted')}
    result['checks']['complete_observed_source_capture_boundary'] = (
        boundary['closed_observed_ledger'] is True and boundary['counts'] == expected_boundary)
    diagnostics = [json.loads(r['data']['data']) for r in rows if r['topic'] == '/lio/safety_diagnostics']
    result['checks']['sensor_gate_diagnostics_present_finite'] = len(diagnostics) > 50 and all(
        d['required_sensor_gap_m'] == .115 and len(d['final']) == 3 and
        all(math.isfinite(v) for v in d['final']) and
        (d['action'] == 'allowed' or d['final'] == [0., 0., 0.])
        for d in diagnostics)
    if 'lio_pose_safety_margin_m' in params:
        result['checks']['stronger_map_guard_parameters'] = params['lio_pose_safety_margin_m'] == .04 and params['lio_sensor_gap_mode'] == 'diagnostic_only'
        from analyze_provincial_forward_clearance import walls_from_sdf
        guard = reserved_guard(rows, [w['polygon'] for w in walls_from_sdf(snapshot/'provincial_stage2_lio_position_imu.sdf')])
        result['reserved_map_guard'] = guard
        result['checks']['reserved_map_guard_independent_recomputation'] = guard['pass']
        loaded = json.loads((directory/'actual_position_imu_loaded.json').read_text())
        inp = json.loads((directory/'input_manifest.json').read_text())['inputs']['install/stage2_lio_sim/lib/libstage2_position_imu.so']
        result['checks']['actual_physical_IMU_library_identity'] = len(loaded) == 1 and loaded[0]['libraries'] == [inp['resolved_path']] and list(loaded[0]['sha256'].values()) == [inp['sha256']]
    result['sensor_gate_action_counts'] = {s: sum(d['action'] == s for d in diagnostics)
        for s in sorted({d['action'] for d in diagnostics})}
    result['all_pass'] = all(result['checks'].values())
    result['safe_evaluator_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['input_history_evaluator_sha256'] = hashlib.sha256(ROOT.joinpath('tools/stage2/lio_source_guard_audit/audit_input_history.py').read_bytes()).hexdigest()
    result['safe_evaluator_command'] = sys.argv
    result['limitations'] = ['Sensor gate adds an engineering reserve, not calibrated LIO or lidar confidence bounds.',
        'GT sampled geometry is not contact detection or continuous-time safety proof.',
        'Observer receives every recorded row but cannot prove publishers lost no messages.',
        'World/plugin declaration and physical recurrence are checked; dynamically loaded library evidence has separate scope.']
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--result-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = evaluate(args.result_dir)
    except Exception as error:
        result = {'all_pass': False, 'status': 'INCOMPLETE_EVIDENCE', 'error': repr(error)}
    with args.output.open('x') as f: json.dump(result, f, indent=2)
    print(json.dumps({k: result.get(k) for k in ['all_pass', 'checks', 'error']}, indent=2))
    raise SystemExit(0 if result['all_pass'] else 2)
