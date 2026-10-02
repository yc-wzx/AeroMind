#!/usr/bin/env python3
"""Recalculate the provisional forward trial verdict from preserved evidence."""

import argparse
import hashlib
import json
import math
import re
import csv
from pathlib import Path

from run_provincial_forward_integration import BASE, FULL, ROOT, STAGES
from summarize_provincial_stage15_terminal_v3 import audit_run, load_jsonl, sdf_walls


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


ROUTE_EVENT = re.compile(r'RMUC route diagnostic route=(R\d+) planned=(.+)')
PLAN_ITEM = re.compile(r'(R\d+:W\d+)=\((-?\d+\.\d+),(-?\d+\.\d+)\)')
DONE_EVENT = re.compile(
    r'RMUC waypoint diagnostic completed route=(R\d+) '
    r'waypoint=(R\d+:W\d+) reason=([a-z_]+) '
    r'pose=\((-?\d+\.\d+),(-?\d+\.\d+)\) '
    r'distance=(\d+\.\d+) speed=(\d+\.\d+)')
SELF = 'tools/summarize_provincial_forward_integration.py'
CURRENT_ONLY_DEPENDENCIES = (
    'tools/run_provincial_safety_fix_matrix.py',
    'src/uav_planning/scripts/ego_goal_adapter.py',
    'src/uav_planning/config/ego_goal_adapter.yaml',
    'src/uav_planning/config/ego_trajectory_executor.yaml',
    'install/uav_planning/lib/uav_planning/ego_goal_adapter.py',
    'install/uav_planning/lib/uav_planning/ego_trajectory_executor.py',
    'install/uav_planning/share/uav_planning/config/ego_goal_adapter.yaml',
    'install/uav_planning/share/uav_planning/config/ego_trajectory_executor.yaml',
)


def raw_waypoint_sequence(events, summary, preflight):
    """Check this route's complete waypoint sequence using original log events."""
    planned_events = [event for event in events if
                      'RMUC route diagnostic route=' in event.get('message', '')]
    done_events = [event for event in events if
                   'RMUC waypoint diagnostic completed' in event.get('message', '')]
    result = {'pass': False, 'reason': None, 'planned_ids': [],
              'completed_ids': [], 'duplicate_delivery_count': 0}
    if len(planned_events) != 1:
        result['reason'] = 'expected exactly one route acceptance event'
        return result
    accepted = planned_events[0]
    match = ROUTE_EVENT.fullmatch(accepted['message'])
    if not match:
        result['reason'] = 'malformed route acceptance event'
        return result
    route_id, payload = match.groups()
    items = PLAN_ITEM.findall(payload)
    reconstructed = ';'.join(f'{name}=({x},{y})' for name, x, y in items)
    if not items or reconstructed != payload:
        result['reason'] = 'malformed planned waypoint list'
        return result
    expected = [(name, (float(x), float(y))) for name, x, y in items]
    result['planned_ids'] = [name for name, _ in expected]
    stages = preflight.get('reference_stages') or []
    if (len(stages) != len(expected) or
            any(not name.startswith(route_id + ':') or
                math.dist(xy, stage) > 0.002
                for (name, xy), stage in zip(expected, stages)) or
            summary.get('final_waypoint_id') != expected[-1][0] or
            math.dist(expected[-1][1], summary.get('goal_xy', [])) > 0.002):
        result['reason'] = 'planned waypoints disagree with preflight/final goal'
        return result
    try:
        accepted_wall = float(accepted['wall_s'])
        if not math.isfinite(accepted_wall):
            raise ValueError('nonfinite acceptance time')
        seen = {}
        unique = []
        for event in done_events:
            parsed = DONE_EVENT.fullmatch(event['message'])
            if not parsed:
                raise ValueError('malformed completion event')
            source_route, waypoint, reason, x, y, distance, speed = parsed.groups()
            if source_route != route_id or not waypoint.startswith(route_id + ':'):
                raise ValueError('completion event belongs to another route')
            if waypoint not in dict(expected):
                raise ValueError('unexpected waypoint completion')
            xy = (float(x), float(y))
            distance, speed = float(distance), float(speed)
            event_wall = float(event['wall_s'])
            gt_pose = event.get('gt_pose')
            if (not all(map(math.isfinite, (*xy, distance, speed, event_wall))) or
                    not isinstance(gt_pose, list) or len(gt_pose) < 2 or
                    not all(isinstance(v, (int, float)) and math.isfinite(v)
                            for v in gt_pose[:2]) or
                    math.dist(xy, gt_pose[:2]) > 0.02 or
                    abs(math.dist(xy, dict(expected)[waypoint])-distance) > 0.003 or
                    distance > 0.05 or reason != 'tolerance'):
                raise ValueError('completion pose/distance inconsistent with plan')
            signature = (event['message'], tuple(gt_pose[:2]))
            if waypoint in seen:
                if signature != seen[waypoint]:
                    raise ValueError('contradictory duplicate completion')
                result['duplicate_delivery_count'] += 1
                continue
            seen[waypoint] = signature
            unique.append((waypoint, event_wall))
        result['completed_ids'] = [name for name, _ in unique]
        ordered = (len(unique) == len(expected) and
                   result['completed_ids'] == result['planned_ids'] and
                   all(a < b for a, b in zip(
                       [accepted_wall] + [wall for _, wall in unique],
                       [wall for _, wall in unique] +
                       [summary['terminal_snapshot']['wall_elapsed_s']])))
        if not ordered:
            raise ValueError('missing, out-of-order, or stale waypoint completion')
        completion = summary.get('navigation_completion') or {}
        if completion.get('message') != seen[expected[-1][0]][0]:
            raise ValueError('final completion differs from original event')
        result['pass'] = True
        result['route_id'] = route_id
    except (ValueError, TypeError, KeyError) as error:
        result['reason'] = str(error)
    return result


def csv_bounds(out, name):
    with (out / f'{name}.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    first, last = rows[0], rows[-1]
    return {'first_sim_s': float(first['sim_t']),
            'first_gt_xy': [float(first['gt_x']), float(first['gt_y'])],
            'last_sim_s': float(last['sim_t']),
            'last_gt_xy': [float(last['gt_x']), float(last['gt_y'])]}


def verify(mode, *, output_name=None, verify_only=False):
    out = BASE / ('continuous_abc' if mode == 'chain' else 'single_full')
    target = out / output_name if output_name else None
    if target is not None:
        if target.resolve().parent != out.resolve():
            raise ValueError('new report must stay in this result directory')
        if target.exists():
            raise FileExistsError(target)
    manifest = json.loads((out / 'input_manifest.json').read_text())
    recorded = json.loads((out / 'raw_data_sha256.json').read_text())
    progress = json.loads((out / 'progress.json').read_text())
    trials = STAGES if mode == 'chain' else (FULL,)
    names = [trial[0] for trial in trials]
    input_checks = {}
    for relative in manifest['sha256']:
        expected = manifest['sha256'].get(relative)
        current = ROOT / relative
        snapshot = out / 'input_snapshot' / relative
        input_checks[relative] = {
            'snapshot_matches_capture_hash':
                bool(expected and (relative.startswith('install/') or
                     snapshot.exists() and digest(snapshot) == expected)),
            'current_workspace_matches_capture_hash':
                bool(expected and current.exists() and digest(current) == expected),
        }
    raw_checks = {name: (out / name).exists() and digest(out / name) == value
                  for name, value in recorded.items()}
    required_raw = {'launch.log', 'input_manifest.json'}
    for name in names:
        required_raw.update((f'{name}.csv', f'{name}.summary.json',
                             f'{name}.events.jsonl',
                             f'{name}.plans.jsonl',
                             f'{name}.actuation.jsonl',
                             f'{name}.published_commands.jsonl',
                             f'{name}.runner.log'))
    raw_checks['all_required_files_hashed'] = required_raw <= recorded.keys()
    walls = sdf_walls()
    reports = []
    previous = None
    previous_bounds = None
    route_ids = []
    continuity = []
    for trial in trials:
        name, route, nominal_start, goal, _ = trial
        summary = json.loads((out / f'{name}.summary.json').read_text())
        evidence = audit_run(out, name, walls)
        events = load_jsonl(out / f'{name}.events.jsonl')
        accepted = [item for item in events if
                    'RMUC route diagnostic route=' in item.get('message', '')]
        completed = [item for item in events if
                     'RMUC waypoint diagnostic completed' in item.get('message', '')]
        final_id = summary.get('final_waypoint_id')
        route_id = final_id.split(':')[0] if isinstance(final_id, str) and ':' in final_id else None
        route_ids.append(route_id)
        preflight = json.loads((out / f'{name}.offline_preflight.json').read_text())
        waypoint_events = raw_waypoint_sequence(events, summary, preflight)
        bounds = csv_bounds(out, name)
        start = summary.get('initial_gt_pose')
        preflight_start = preflight.get('measured_start_gt')
        stage_checks = {
            'raw_terminal_evidence_pass': evidence['pass'],
            'navigation_completed': summary.get('result') == 'terminal_pass',
            'accepted_and_unique_route_event':
                summary.get('goal_accepted') is True and len(accepted) == 1 and
                route_id is not None and
                f'route={route_id} ' in accepted[0]['message'],
            'matched_final_completion':
                any(f'waypoint={final_id} ' in item['message'] for item in completed) and
                (summary.get('navigation_completion') or {}).get('waypoint_id') == final_id,
            'all_internal_waypoints_completed_in_order': waypoint_events['pass'],
            'one_route_acceptance_record': len(accepted) == 1,
            'goal_and_yaw_match':
                summary.get('route') == route and
                summary.get('goal_xy') == list(goal) and
                abs(summary.get('sent_goal_yaw_rad', math.inf)-math.pi/2) <= 0.05 and
                abs(summary.get('final_gt_yaw_rad', math.inf)-math.pi/2) <= 0.05,
            'measured_start_matches_preflight':
                start is not None and preflight_start is not None and
                math.dist(start[:2], preflight_start[:2]) <= 0.03 and
                math.dist(start[:2], nominal_start) <= 0.20,
            'route_preflight_gap':
                preflight['min_sampled_body_wall_gap_m'] >= 0.08 and
                preflight['min_grid_clearance_m'] >= 0.40-0.001,
            'no_retry_stuck_false_success_overlap':
                summary.get('retry_count') == 0 and
                summary.get('stuck_count') == 0 and
                summary.get('false_success') is False and
                summary.get('geometric_overlap_samples') == 0 and
                summary.get('minimum_physical_wall_clearance_m', 0) >= 0.08 and
                not any('GOAL REJECTED' in item.get('message', '') for item in events),
            'preflight_yaw90':
                preflight_start is not None and
                abs(preflight_start[2]-math.pi/2) <= 0.05,
            'raw_start_and_end_match_summary':
                math.dist(bounds['first_gt_xy'], start[:2]) <= 0.03 and
                math.dist(bounds['last_gt_xy'], summary['final_gt_xy']) <= 0.02 and
                abs(bounds['last_sim_s']-
                    summary['terminal_snapshot']['gt_stamp_s']) <= 0.1,
        }
        if previous is not None:
            same_world = (
                bounds['first_sim_s'] >
                previous['terminal_snapshot']['gt_stamp_s'] and
                previous_bounds['last_sim_s'] <= bounds['first_sim_s'] and
                math.dist(bounds['first_gt_xy'],
                          previous_bounds['last_gt_xy']) <= 0.03 and
                math.dist(start[:2], previous['final_gt_xy']) <= 0.03)
            stage_checks['continuous_same_world_pose_and_sim_time'] = same_world
            continuity.append(same_world)
        previous = summary
        previous_bounds = bounds
        reports.append({
            'name': name, 'checks': stage_checks,
            'pass': all(stage_checks.values()),
            'event_id': final_id, 'route_id': route_id,
            'internal_completed_waypoint_ids': [
                item['message'].split('waypoint=')[1].split()[0]
                for item in completed if 'waypoint=' in item['message']],
            'waypoint_event_audit': waypoint_events,
            'raw_gt_bounds': bounds,
            'initial_gt_pose': start,
            'final_gt_pose': [*summary['final_gt_xy'], summary['final_gt_yaw_rad']],
            'goal_xy': goal,
            'gt_goal_error_m': summary['gt_goal_error_m'],
            'odom_goal_error_m': summary['odom_goal_error_m'],
            'minimum_sampled_body_wall_gap_m':
                evidence['minimum_sampled_body_wall_gap_m'],
            'max_raw_cmd': summary.get('max_observed_cmd'),
            'duration_sim_s': summary['duration_sim_s'],
            'duration_wall_s': summary['duration_wall_s'],
            'terminal_2_plus_5': evidence['full_2_plus_5_window'],
            'mission_evidence': evidence['mission_evidence'],
        })
    checks = {
        'runner_finished_without_failure':
            progress.get('status') == 'RUNNERS_COMPLETED_PENDING_INDEPENDENT_AUDIT' and
            'failure' not in progress,
        'only_requested_trials_ran':
            [item['name'] for item in progress.get('runs', [])] == names,
        'one_gazebo_and_clean_exit':
            len(progress.get('gazebo_server_processes', [])) == 1 and
            progress.get('remaining_gazebo_servers') == [],
        'listed_current_inputs_except_auditor_match_capture':
            all(item['current_workspace_matches_capture_hash']
                for relative, item in input_checks.items() if relative != SELF),
        'non_install_input_snapshots_match_capture':
            all(item['snapshot_matches_capture_hash']
                for item in input_checks.values()),
        'raw_files_match_post_write_hashes': all(raw_checks.values()),
        'distinct_route_ids': len(set(route_ids)) == len(names) and None not in route_ids,
        'each_stage_evidence_pass': all(item['pass'] for item in reports),
        'same_world_continuity': all(continuity),
    }
    evaluator = ROOT / SELF
    uncaptured = {
        path: {'current_sha256': digest(ROOT / path) if (ROOT / path).exists()
               else None, 'capture_time_hash_available': False}
        for path in CURRENT_ONLY_DEPENDENCIES
        if path not in manifest['sha256']}
    command = (f'cd {ROOT} && source /opt/ros/humble/setup.bash && '
               f'source install/setup.bash && python3 -B '
               f'tools/summarize_provincial_forward_integration.py --mode {mode} ' +
               (f'--output-name {output_name}' if output_name else '--verify-only'))
    report = {
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'competition_arena_verified': False,
        'mode': mode, 'checks': checks,
        'all_pass': all(checks.values()),
        'capture_input_checks': input_checks,
        'raw_file_hash_checks': raw_checks,
        'evaluator_sha256_current': digest(evaluator),
        'evaluator_sha256_at_capture': manifest['sha256'].get(SELF),
        'evaluator_changed_since_capture':
            digest(evaluator) != manifest['sha256'].get(SELF),
        'current_only_uncaptured_dependencies': uncaptured,
        'complete_capture_time_dependency_attestation': not uncaptured,
        'runtime_package_versions_captured': False,
        'external_goal_publications_independently_observed': False,
        'no_reset_service_calls_independently_recorded': False,
        'status_levels': {
            'runner_processes_exited_successfully':
                all(item.get('runner_exit_code') == 0
                    for item in progress.get('runs', [])),
            'navigation_completed_each_route':
                all(stage['checks']['navigation_completed'] for stage in reports),
            'raw_evidence_audit_pass':
                all(stage['checks']['raw_terminal_evidence_pass'] and
                    stage['checks']['all_internal_waypoints_completed_in_order']
                    for stage in reports),
        },
        'stages': reports,
        'total_stage_duration_sim_s': sum(item['duration_sim_s'] for item in reports),
        'contact_data': 'CONTACT DATA UNAVAILABLE',
        'verification_command': command,
    }
    if target is not None and not verify_only:
        target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'mode': mode, 'all_pass': report['all_pass'],
                      'stages': {item['name']: item['pass'] for item in reports},
                      'failed_checks': [name for name, value in checks.items()
                                        if not value]}), flush=True)
    if not report['all_pass']:
        raise SystemExit(2)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('chain', 'full'), required=True)
    outputs = parser.add_mutually_exclusive_group(required=True)
    outputs.add_argument('--verify-only', action='store_true')
    outputs.add_argument('--output-name')
    args = parser.parse_args()
    verify(args.mode, output_name=args.output_name, verify_only=args.verify_only)


if __name__ == '__main__':
    main()
