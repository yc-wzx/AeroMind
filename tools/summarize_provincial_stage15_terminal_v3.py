#!/usr/bin/env python3
"""Independently validate the entire 2+5 s terminal window from saved traces."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from summarize_provincial_stage15_terminal_v2 import ROOT, sdf_walls, summarize


def load_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def bounded_sequence(values, maximum_gap, *, strict=True):
    return (len(values) >= 2 and
            all(math.isfinite(value) for value in values) and
            all(0 < b-a <= maximum_gap if strict else 0 <= b-a <= maximum_gap
                for a, b in zip(values, values[1:])))


def full_window(out, name):
    summary = json.loads((out / f'{name}.summary.json').read_text())
    with (out / f'{name}.csv').open(newline='') as stream:
        rows = [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]
    commands = load_jsonl(out / f'{name}.published_commands.jsonl')
    events = load_jsonl(out / f'{name}.events.jsonl')
    start = summary.get('stable_stop_start_sim_s')
    observe = summary.get('post_stop_observation_start_sim_s')
    snapshot = summary.get('terminal_snapshot') or {}
    end = snapshot.get('gt_stamp_s')
    goal = summary.get('goal_xy')
    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in (start, observe, end)) or goal is None:
        return {'pass': False, 'reason': 'missing or nonfinite terminal times/goal'}
    window = [row for row in rows if start <= row['sim_t'] <= end]
    direct = [item for item in commands
              if isinstance(item.get('gt_stamp_s'), (int, float)) and
              start <= item['gt_stamp_s'] <= end]
    finite_fields = ('sim_t', 'wall_s', 'gt_x', 'gt_y', 'gt_yaw',
                     'gt_speed', 'gt_yaw_speed', 'odom_x', 'odom_y',
                     'published_vx', 'published_vy', 'published_wz')
    finite = (len(window) >= 2 and len(direct) >= 2 and
              all(all(math.isfinite(row[key]) for key in finite_fields)
                  for row in window) and
              all(all(isinstance(item.get(key), (int, float)) and
                          math.isfinite(item[key])
                      for key in ('wall_s', 'gt_stamp_s', 'vx', 'vy', 'wz'))
                  for item in direct))
    if not finite:
        return {'pass': False, 'reason': 'missing or nonfinite raw window evidence'}
    sim_times = [row['sim_t'] for row in window]
    gt_wall = [row['wall_s'] for row in window]
    cmd_wall = [item['wall_s'] for item in direct]
    cmd_sim = [item['gt_stamp_s'] for item in direct]
    xy = [(row['gt_x'], row['gt_y']) for row in window]
    goal_errors = [math.dist(point, goal) for point in xy]
    drifts = [math.dist(point, xy[0]) for point in xy]
    gt_speeds = [abs(row['gt_speed']) for row in window]
    gt_yaw_speeds = [abs(row['gt_yaw_speed']) for row in window]
    direct_speeds = [math.hypot(item['vx'], item['vy']) for item in direct]
    direct_yaw_speeds = [abs(item['wz']) for item in direct]
    event_messages = [event.get('message') for event in events]
    completion = summary.get('navigation_completion') or {}
    waypoint = summary.get('final_waypoint_id')
    route = waypoint.split(':')[0] if isinstance(waypoint, str) and ':' in waypoint else None
    completion_message = completion.get('message')
    accepted_event = any(message and
                         f'RMUC route diagnostic route={route} ' in message and
                         f'{waypoint}=' in message
                         for message in event_messages) if route else False
    completion_event = (bool(completion_message) and
                        completion_message in event_messages and
                        f'waypoint={waypoint} ' in completion_message)
    snapshots_match = (
        isinstance(snapshot.get('gt_xy'), list) and
        len(snapshot['gt_xy']) == 2 and
        all(math.isfinite(value) for value in snapshot['gt_xy']) and
        math.dist(snapshot['gt_xy'], xy[-1]) <= 0.02 and
        abs(snapshot.get('gt_yaw_rad', math.nan)-window[-1]['gt_yaw']) <= 0.01 and
        abs(math.dist(snapshot['gt_xy'], goal)-summary['gt_goal_error_m']) <= 1e-6 and
        snapshot.get('gt_received_age_s') is not None and
        0 <= snapshot['gt_received_age_s'] <= 0.2 and
        snapshot.get('odom_received_age_s') is not None and
        0 <= snapshot['odom_received_age_s'] <= 0.5 and
        snapshot.get('final_command_received_age_s') is not None and
        0 <= snapshot['final_command_received_age_s'] <= 0.2)
    checks = {
        'two_plus_five_sim_seconds': observe-start >= 2.0-1e-6 and
            end-observe >= 5.0-1e-6,
        'raw_gt_covers_complete_window': sim_times[0] <= start+0.1 and
            sim_times[-1] >= end-0.1 and
            any(abs(value-observe) <= 0.1 for value in sim_times),
        'raw_command_covers_complete_window': cmd_sim[0] <= start+0.2 and
            cmd_sim[-1] >= end-0.2 and
            any(abs(value-observe) <= 0.2 for value in cmd_sim),
        'gt_sim_and_wall_continuous': bounded_sequence(sim_times, 0.2) and
            bounded_sequence(gt_wall, 0.2),
        'direct_command_wall_and_sim_continuous':
            bounded_sequence(cmd_wall, 0.2) and
            bounded_sequence(cmd_sim, 0.2, strict=False),
        'gt_stopped_for_full_window': max(gt_speeds) <= 0.02 and
            max(gt_yaw_speeds) <= 0.03,
        'direct_output_stopped_for_full_window': max(direct_speeds) <= 0.02 and
            max(direct_yaw_speeds) <= 0.03,
        'near_goal_and_no_drift_for_full_window':
            max(goal_errors) <= 0.10 and max(drifts) <= 0.03,
        'accepted_and_completed_events_in_raw_log':
            accepted_event and completion_event and
            completion.get('waypoint_id') == waypoint and
            completion.get('gt_fresh') is True and
            isinstance(completion.get('gt_goal_error_m'), (int, float)) and
            math.isfinite(completion['gt_goal_error_m']) and
            completion['gt_goal_error_m'] <= 0.05 and
            isinstance(completion.get('gt_stamp_s'), (int, float)) and
            completion['gt_stamp_s'] <= start+0.2,
        'terminal_snapshot_matches_raw_trace': bool(snapshots_match),
    }
    return {
        'pass': all(checks.values()), 'checks': checks,
        'stable_start_sim_s': start, 'observation_start_sim_s': observe,
        'terminal_sim_s': end, 'gt_samples': len(window),
        'direct_command_samples': len(direct),
        'max_gt_speed_mps': max(gt_speeds),
        'max_direct_speed_mps': max(direct_speeds),
        'max_goal_error_m': max(goal_errors),
        'max_stop_drift_m': max(drifts),
    }


def mission_evidence(out, name):
    """Require finite, continuous mission data with a narrow startup exception."""
    with (out / f'{name}.csv').open(newline='') as stream:
        rows = [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]
    commands = load_jsonl(out / f'{name}.published_commands.jsonl')
    if len(rows) < 2 or len(commands) < 2:
        return {'pass': False, 'reason': 'mission trace missing'}
    truth_and_odom = ('sim_t', 'wall_s', 'gt_x', 'gt_y', 'gt_yaw',
                      'gt_speed', 'gt_yaw_speed', 'gt_goal_error',
                      'wall_clearance', 'odom_x', 'odom_y', 'odom_goal_error')
    published = ('published_vx', 'published_vy', 'published_wz')
    base_finite = all(all(math.isfinite(row[key]) for key in truth_and_odom)
                      for row in rows)
    first_valid = next((index for index, row in enumerate(rows)
                        if all(math.isfinite(row[key]) for key in published)), None)
    if first_valid is None:
        return {'pass': False, 'reason': 'no valid final command sample'}
    prefix = rows[:first_valid]
    startup_fields_only = (
        all(all(math.isnan(row[key]) for key in published) for row in prefix) and
        all(all(math.isfinite(row[key]) for key in published)
            for row in rows[first_valid:]))
    command_finite = all(all(isinstance(item.get(key), (int, float)) and
                             math.isfinite(item[key])
                             for key in ('wall_s', 'gt_stamp_s', 'vx', 'vy', 'wz'))
                         for item in commands)
    first_command_wall = commands[0].get('wall_s')
    startup_confirmed = (not prefix or
                         command_finite and
                         rows[0]['wall_s'] <= prefix[-1]['wall_s'] <
                         first_command_wall <= rows[first_valid]['wall_s'] and
                         first_command_wall - rows[0]['wall_s'] <= 0.2 and
                         rows[first_valid]['sim_t'] - rows[0]['sim_t'] <= 0.2)
    checks = {
        'gt_and_odom_finite_throughout': base_finite,
        'final_command_missing_only_before_first_reception': startup_fields_only,
        'startup_exception_bounded_and_confirmed_by_command_trace':
            bool(startup_confirmed),
        'direct_command_trace_finite_throughout': command_finite,
        'gt_sim_and_wall_continuous_throughout':
            bounded_sequence([row['sim_t'] for row in rows], 0.2) and
            bounded_sequence([row['wall_s'] for row in rows], 0.2),
        'direct_command_continuous_throughout':
            command_finite and
            bounded_sequence([item['wall_s'] for item in commands], 0.2) and
            bounded_sequence([item['gt_stamp_s'] for item in commands],
                             0.2, strict=False),
    }
    return {'pass': all(checks.values()), 'checks': checks,
            'allowed_startup_missing_rows': first_valid,
            'first_valid_final_command_row_sim_s': rows[first_valid]['sim_t'],
            'first_direct_command_wall_s': first_command_wall}


def audit_run(out, name, walls):
    prior = summarize(out, name, walls)
    full = full_window(out, name)
    mission = mission_evidence(out, name)
    # The old global check cannot distinguish a verified startup prefix from
    # a later missing sample. The mission check above makes that distinction.
    relevant_v2 = {key: value for key, value in prior['checks'].items()
                   if key != 'all_critical_samples_finite'}
    prior_except_startup = (prior['result'] == 'terminal_pass' and
                            all(relevant_v2.values()))
    return {**prior, 'v2_checks_pass': prior['pass'],
            'v2_checks_excluding_full_run_finiteness': prior_except_startup,
            'full_run_finiteness_check':
                prior['checks']['all_critical_samples_finite'],
            'full_2_plus_5_window': full,
            'mission_evidence': mission,
            'pass': prior_except_startup and full['pass'] and mission['pass']}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--names', required=True, nargs='+')
    parser.add_argument('--output-name', default='terminal_evidence_v3.json')
    parser.add_argument('--historical-audit', action='store_true',
                        help='use the saved v2 provenance attestation for prior runs')
    parser.add_argument('--attestation-file',
                        help='saved in-directory audit whose capture-time hashes matched')
    args = parser.parse_args()
    out = args.output_dir.resolve()
    target = out / args.output_name
    if target.exists():
        raise FileExistsError(target)
    manifest = json.loads((out / 'input_manifest.json').read_text())
    mismatches = [path for path, digest in manifest['sha256'].items()
                  if hashlib.sha256((ROOT / path).read_bytes()).hexdigest() != digest]
    prior_attestation = None
    attestation_file = (args.attestation_file or
                        ('terminal_evidence_v2.json' if args.historical_audit else None))
    if attestation_file:
        attestation_path = out / attestation_file
        if attestation_path.resolve().parent != out:
            raise ValueError('attestation must be in the audited result directory')
        old = json.loads(attestation_path.read_text())
        old_mismatches = old.get('current_workspace_input_hash_mismatches',
                                 old.get('input_hash_mismatches'))
        prior_attestation = (old.get('all_pass') is True and
                             old_mismatches == [] and
                             {item['name'] for item in old['runs']} == set(args.names))
    walls = sdf_walls()
    runs = [audit_run(out, name, walls) for name in args.names]
    report = {
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'current_workspace_input_hash_mismatches': mismatches,
        'capture_time_hash_attestation': prior_attestation,
        'capture_time_attestation_file': attestation_file,
        'runs': runs,
        'all_pass': ((prior_attestation if attestation_file else not mismatches)
                     and all(run['pass'] for run in runs)),
    }
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({item['name']: item['pass'] for item in runs}))
    if not report['all_pass']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
