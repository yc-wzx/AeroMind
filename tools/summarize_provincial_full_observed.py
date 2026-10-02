#!/usr/bin/env python3
"""Recompute the one observed Full verdict from immutable native ROS evidence."""

import argparse
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path

from analyze_provincial_forward_clearance import walls_from_sdf
from run_provincial_forward_integration import ROOT
from summarize_provincial_forward_integration import (raw_waypoint_sequence,
    ROUTE_EVENT, PLAN_ITEM, DONE_EVENT)
from summarize_provincial_stage15_terminal_v3 import audit_run, load_jsonl, sdf_walls

OUT = ROOT / 'tools/results/provincial_stage15_full_observed_20260928_preflight2'
NAME = 'trial_full_observed_01'
SDF = 'src/uav_bringup/worlds/provincial_2025_training.sdf'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_tree(value):
    if isinstance(value, dict):
        return all(finite_tree(item) for item in value.values())
    if isinstance(value, list):
        return all(finite_tree(item) for item in value)
    return not isinstance(value, (int, float)) or math.isfinite(value)


def yaw(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                      1-2*(q['y']*q['y']+q['z']*q['z']))


def recorder_health_audit(out, name, topics, required):
    """Legacy evidence stays auditable; new writer health must be independently complete."""
    path = out / (name + '.native.health.json')
    if not required and not path.exists():
        return {'pass': True, 'status': 'LEGACY_NO_WRITER_HEALTH',
                'writer_health_verified': False}
    timing = out / (name + '.native.timing.jsonl')
    if not path.is_file() or not timing.is_file():
        return {'pass': False, 'status': 'MISSING_WRITER_EVIDENCE'}
    try:
        health = json.loads(path.read_text())
        observed = {topic: item['count'] for topic, item in topics.items() if item['count']}
        records = 0
        valid = True
        with timing.open() as stream:
            for line in stream:
                row = json.loads(line)
                start, end, duration = (row[key] for key in
                    ('start_monotonic_ns', 'end_monotonic_ns', 'duration_ns'))
                valid = valid and all(isinstance(v, int) for v in (start, end, duration))
                valid = valid and 0 <= start <= end and end-start == duration
                records += 1
        checks = {
            'closed_without_errors': health.get('schema') == 1 and
                health.get('pass') is True and health.get('worker_joined') is True
                and health.get('errors') == [],
            'received_written_raw_counts_match': observed == health.get('received_counts')
                == health.get('written_counts'),
            'timing_count_and_fields_match': valid and records == health.get('timing_count')
                == health.get('written_timing_count'),
        }
        return {'pass': all(checks.values()), 'checks': checks,
                'status': 'AUDITED', 'health': health}
    except (ValueError, KeyError, TypeError) as error:
        return {'pass': False, 'status': 'INVALID_WRITER_EVIDENCE', 'error': str(error)}


def native_audit(out, summary, walls, name=NAME, expected_goal=(8.7, 4.25)):
    from provincial_safety_geometry import polygon_distance, rectangle
    required = ('/clock', '/gazebo/odometry', '/ground/odometry',
                '/cmd_vel', '/model/omni_robot/cmd_vel', '/goal_pose',
                '/ground/planning/goal',
                '/ground/planning/actuation_diagnostics',
                '/ground/planning/commanded_setpoint', '/planning/bspline',
                '/ground/planning/planned_trajectory', '/rosout/relevant')
    topics = defaultdict(lambda: {'count': 0, 'max_stamp_gap_s': None,
                                  'duplicate_stamp_count': 0,
                                  'out_of_order_stamp_count': 0,
                                  'receive_order_error_count': 0,
                                  'nonfinite_count': 0,
                                  'missing_stamp_count': 0, 'max_receive_gap_s': None,
                                  'active_max_receive_gap_s': None})
    start_receive_ns = (summary.get('goal_delivery') or {}).get('publish_monotonic_ns', -math.inf)
    last_stamp = {}
    last_receive = {}
    starts = summary['initial_gt_pose']
    start_t = summary['terminal_snapshot']['gt_stamp_s'] - summary['duration_sim_s']
    end_t = summary['terminal_snapshot']['gt_stamp_s']
    minimum = {'gap_m': math.inf}
    gt_active = []
    goals = []
    route_receipts = []
    with (out / f'{name}.native.jsonl').open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            topic = row['topic']
            item = topics[topic]
            item['count'] += 1
            if row['receive_sequence'] != item['count']:
                item['receive_order_error_count'] += 1
            received = row['receive_monotonic_ns']
            if not isinstance(received, (int, float)) or not math.isfinite(received):
                item['nonfinite_count'] += 1
                continue
            if topic in last_receive:
                delta_receive = (received-last_receive[topic])*1e-9
                if delta_receive < 0:
                    item['receive_order_error_count'] += 1
                item['max_receive_gap_s'] = max(item['max_receive_gap_s'] or 0, delta_receive)
                if last_receive[topic] >= start_receive_ns:
                    item['active_max_receive_gap_s'] = max(
                        item['active_max_receive_gap_s'] or 0, delta_receive)
            last_receive[topic] = received
            stamp = row['message_stamp_s']
            if stamp is None:
                item['missing_stamp_count'] += 1
            elif not math.isfinite(stamp):
                item['nonfinite_count'] += 1
            else:
                if topic in last_stamp:
                    delta = stamp - last_stamp[topic]
                    if delta < 0:
                        item['out_of_order_stamp_count'] += 1
                    elif delta == 0:
                        item['duplicate_stamp_count'] += 1
                    else:
                        item['max_stamp_gap_s'] = max(
                            item['max_stamp_gap_s'] or 0, delta)
                last_stamp[topic] = stamp
            if not finite_tree(row['data']):
                item['nonfinite_count'] += 1
            if topic == '/goal_pose':
                goals.append({'stamp_s': stamp, 'receive_sim_s': row['receive_sim_s'],
                              'receive_monotonic_ns': received,
                              'pose': row['data']['pose']})
            if (topic == '/rosout/relevant' and
                    row['data'].get('msg', '').startswith('RMUC route diagnostic route=')):
                route_receipts.append({'message': row['data']['msg'],
                                       'logger': row['data'].get('name'),
                                       'stamp': row['data'].get('stamp'),
                                       'receive_monotonic_ns': received})
            if topic != '/gazebo/odometry' or stamp is None or not start_t <= stamp <= end_t:
                continue
            pose = row['data']['pose']['pose']
            twist = row['data']['twist']['twist']
            p, q = pose['position'], pose['orientation']
            if not finite_tree(p) or not finite_tree(q) or not finite_tree(twist):
                continue
            angle = yaw(q)
            body = rectangle(p['x'], p['y'], angle, .52, .42)
            distance, wall = min(
                ((polygon_distance(body, w['polygon']), w['name']) for w in walls),
                key=lambda pair: pair[0])
            row_summary = {'sim_t': stamp, 'x': p['x'], 'y': p['y'],
                           'yaw_rad': angle, 'gap_m': distance,
                           'nearest_wall': wall, 'line_number': line_number}
            gt_active.append(row_summary)
            if distance < minimum['gap_m']:
                minimum = row_summary
    gaps = [b['sim_t']-a['sim_t'] for a, b in zip(gt_active, gt_active[1:])]
    positive = [value for value in gaps if value > 0]
    elapsed = gt_active[-1]['sim_t']-gt_active[0]['sim_t'] if len(gt_active) >= 2 else 0
    goal = goals[0]['pose'] if len(goals) == 1 else None
    goal_xy = ([goal['position']['x'], goal['position']['y']] if goal else None)
    goal_yaw = yaw(goal['orientation']) if goal else None
    delivery = summary.get('goal_delivery') or {}
    ack = delivery.get('acceptance') or {}
    raw_ack_matches = False
    if len(route_receipts) == 1 and len(goals) == 1 and ack:
        receipt = route_receipts[0]
        source_stamp = receipt.get('stamp') or {}
        stamp_ns = source_stamp.get('sec', 0)*1_000_000_000 + source_stamp.get('nanosec', 0)
        raw_ack_matches = (
            receipt['logger'] == 'gazebo_navigation_interface' and
            receipt['message'] == ack.get('message') and
            stamp_ns == ack.get('log_stamp_ns') and
            stamp_ns >= delivery.get('publish_wall_ns', math.inf) and
            0 <= receipt['receive_monotonic_ns']-delivery.get('publish_monotonic_ns', math.inf)
               <= delivery.get('acknowledgement_timeout_s', 0)*1e9 and
            abs(receipt['receive_monotonic_ns']-ack.get('receive_monotonic_ns', math.inf)) < 1e8)
    missing_topics = [topic for topic in required if topics[topic]['count'] == 0]
    snapshot = out/'input_snapshot/tools/provincial_native_trace.py'
    health_required = snapshot.is_file() and 'HEALTH_SCHEMA = 1' in snapshot.read_text()
    writer = recorder_health_audit(out, name, topics, health_required)
    continuity = {topic: topics[topic]['active_max_receive_gap_s'] is not None and
                  topics[topic]['active_max_receive_gap_s'] <= limit
                  for topic, limit in (('/gazebo/odometry', .2),
                                       ('/ground/odometry', .5),
                                       ('/model/omni_robot/cmd_vel', .2))}
    checks = {
        'native_receive_continuity': all(continuity.values()),
        'writer_evidence_complete': writer['pass'],
        'required_topics_observed': not missing_topics,
        'one_external_goal_observed': len(goals) == 1 and goal_xy == list(expected_goal)
            and abs(goal_yaw-math.pi/2) <= .05,
        'raw_route_acceptance_matches_delivery_evidence': raw_ack_matches,
        'all_critical_native_fields_finite': all(
            topics[name]['nonfinite_count'] == 0
            for name in ('/clock', '/gazebo/odometry', '/ground/odometry',
                         '/cmd_vel', '/model/omni_robot/cmd_vel')),
        'native_critical_receive_order': all(
            topics[name]['receive_order_error_count'] == 0 and
            topics[name]['out_of_order_stamp_count'] == 0
            for name in ('/clock', '/gazebo/odometry', '/ground/odometry',
                         '/model/omni_robot/cmd_vel')),
        'full_active_gt_coverage': bool(gt_active) and
            gt_active[0]['sim_t'] <= start_t + .1 and
            gt_active[-1]['sim_t'] >= end_t - .1 and elapsed >= 7,
        'native_gt_no_large_gap': len(positive) >= 2 and max(positive) <= .20,
        'sampled_gap_above_existing_gate': minimum['gap_m'] >= .08,
    }
    return {'checks': checks, 'pass': all(checks.values()),
            'topics': dict(topics), 'receive_continuity_checks': continuity,
            'writer': writer, 'observed_external_goals': goals,
            'missing_topics': missing_topics,
            'raw_route_receipts': route_receipts,
            'active_gt_count': len(gt_active), 'active_gt_elapsed_sim_s': elapsed,
            'active_gt_receive_rate_hz': (len(gt_active)-1)/elapsed if elapsed else None,
            'active_gt_max_gap_s': max(positive) if positive else None,
            'active_gt_min_gap_s': min(positive) if positive else None,
            'active_gt_duplicate_stamp_count': sum(value == 0 for value in gaps),
            'active_gt_out_of_order_stamp_count': sum(value < 0 for value in gaps),
            'minimum': minimum,
            'gt_trace': gt_active}


def native_terminal_audit(out, summary, waypoint, name=NAME, expected_stages=None):
    """Validate the declared 2+5 s window from every original native message.

    No later replacement window is searched. Clock associations are observations
    at reception, never invented Twist generation stamps.
    """
    thresholds = {'gt_speed_mps': .02, 'gt_yaw_speed_rad_s': .03,
                  'final_speed_mps': .02, 'final_yaw_speed_rad_s': .03,
                  'goal_error_m': .10, 'drift_m': .03,
                  'gt_receive_gap_s': .2, 'final_receive_gap_s': .2,
                  'odom_receive_age_s': .5, 'odom_sim_age_s': .2,
                  'completion_error_m': .05}
    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('missing or nonfinite numeric evidence')
        return value
    def seq(values, maximum, strict=True):
        return len(values) >= 2 and all(
            0 < b-a <= maximum if strict else 0 <= b-a <= maximum
            for a, b in zip(values, values[1:]))
    def pose(data):
        p, q = data['position'], data['orientation']
        xy = [number(p['x']), number(p['y'])]
        for k in ('x', 'y', 'z', 'w'): number(q[k])
        return xy, yaw(q)
    def velocity(data):
        return (math.hypot(number(data['linear']['x']), number(data['linear']['y'])),
                abs(number(data['angular']['z'])))
    def age(row, other):
        return (row['recv']-other['recv']) if other else math.inf
    try:
        start = number(summary.get('stable_stop_start_sim_s'))
        middle = number(summary.get('post_stop_observation_start_sim_s'))
        snapshot = summary.get('terminal_snapshot') or {}
        end = number(snapshot.get('gt_stamp_s'))
        if middle-start < 2.-1e-6 or end-middle < 5.-1e-6:
            raise ValueError('declared window is not complete 2+5 seconds')
        rows = load_jsonl(out/f'{name}.native.jsonl')
        latest = {}; streams = defaultdict(list); goals = []; accepted = []; done = []
        previous = None
        for raw in rows:
            topic = raw['topic']; recv = number(raw['receive_monotonic_ns'])*1e-9
            if previous is not None and recv < previous:
                raise ValueError('native receive order inconsistent')
            previous = recv
            row = {'recv': recv, 'clock_age_s': age({'recv': recv}, latest.get('/clock'))}
            if topic == '/clock':
                clock = raw['data']['clock']
                t = number(clock['sec']) + number(clock['nanosec'])*1e-9
                if abs(t-number(raw['message_stamp_s'])) > 1e-9:
                    raise ValueError('clock raw stamp inconsistent')
                row['t'] = t; latest[topic] = row; streams[topic].append(row)
                continue
            clock = latest.get('/clock')
            observed_sim = raw.get('receive_sim_s')
            row['clock_matches'] = (clock is not None and observed_sim is not None and
                abs(number(observed_sim)-clock['t']) <= 1e-9 and 0 <= row['clock_age_s'] <= .2)
            if topic in ('/gazebo/odometry', '/ground/odometry'):
                stamp = raw['data']['header']['stamp']
                t = number(stamp['sec']) + number(stamp['nanosec'])*1e-9
                if abs(t-number(raw['message_stamp_s'])) > 1e-9:
                    raise ValueError('odometry header/raw stamp inconsistent')
                row['t'] = t
                row['xy'], row['yaw'] = pose(raw['data']['pose']['pose'])
                row['speed'], row['wz'] = velocity(raw['data']['twist']['twist'])
                row['odom_age_s'] = age(row, latest.get('/ground/odometry'))
                row['odom_sim_age_s'] = abs(t-latest['/ground/odometry']['t']) if '/ground/odometry' in latest else math.inf
                row['cmd_age_s'] = age(row, latest.get('/model/omni_robot/cmd_vel'))
                row['clock_sim_age_s'] = abs(t-clock['t']) if clock else math.inf
                latest[topic] = row; streams[topic].append(row)
            elif topic == '/model/omni_robot/cmd_vel':
                if observed_sim is None:
                    if clock is not None or recv >= number(summary['goal_delivery']['publish_monotonic_ns'])*1e-9:
                        raise ValueError('missing observed clock during valid command recording')
                    row['t'] = None  # pre-clock startup, retained without invented time
                else:
                    row['t'] = number(observed_sim)
                if raw.get('message_stamp_s') is not None:
                    raise ValueError('Twist cannot supply a fabricated generation stamp')
                row['speed'], row['wz'] = velocity(raw['data'])
                row['command'] = [number(raw['data']['linear']['x']),
                                  number(raw['data']['linear']['y']),
                                  number(raw['data']['angular']['z'])]
                latest[topic] = row; streams[topic].append(row)
            elif topic == '/goal_pose':
                row['xy'], row['yaw'] = pose(raw['data']['pose']); goals.append(row)
            elif topic == '/rosout/relevant' and raw['data'].get('name') == 'gazebo_navigation_interface':
                row['message'] = raw['data'].get('msg', '')
                row['gt'] = latest.get('/gazebo/odometry')
                if ROUTE_EVENT.fullmatch(row['message']): accepted.append(row)
                if DONE_EVENT.fullmatch(row['message']): done.append(row)
        if len(goals) != 1 or len(accepted) != 1:
            raise ValueError('expected one raw external goal and production acceptance')
        goal = goals[0]['xy']; route, payload = ROUTE_EVENT.fullmatch(accepted[0]['message']).groups()
        planned = PLAN_ITEM.findall(payload)
        if not planned or not waypoint['pass'] or route != waypoint['route_id']:
            raise ValueError('route/waypoint raw evidence incomplete')
        raw_plan = [(ident, [float(x), float(y)]) for ident,x,y in planned]
        if expected_stages is None:
            preflight = json.loads((out/f'{name}.offline_preflight.json').read_text())
            expected_coordinates = preflight.get('reference_stages') or []
        else:
            expected_coordinates = expected_stages
        if ([ident for ident,_ in raw_plan] != waypoint['planned_ids'] or
                len(raw_plan) != len(expected_coordinates) or
                any(math.dist(xy, ref) > .002 for (_,xy),ref in zip(raw_plan, expected_coordinates)) or
                ';'.join(f'{ident}=({x},{y})' for ident,x,y in planned) != payload):
            raise ValueError('native planned waypoints disagree with independently audited route')
        raw_unique = []; signatures = {}
        plan_coordinates = dict(raw_plan)
        for event in done:
            parsed = DONE_EVENT.fullmatch(event['message']).groups()
            source_route, ident, reason, px, py, distance, speed = parsed
            observed_gt = event['gt']; point = [float(px), float(py)]
            if (source_route != route or ident not in plan_coordinates or reason != 'tolerance' or
                    observed_gt is None or not 0 <= age(event, observed_gt) <= .2 or
                    float(distance) > .05 or math.dist(point, observed_gt['xy']) > .02 or
                    abs(math.dist(point, plan_coordinates[ident])-float(distance)) > .003):
                raise ValueError('native waypoint completion inconsistent with route/GT')
            if ident in signatures:
                if signatures[ident] != event['message']:
                    raise ValueError('contradictory native waypoint retransmission')
                continue
            signatures[ident] = event['message']; raw_unique.append((ident,event['recv']))
        if ([ident for ident,_ in raw_unique] != waypoint['planned_ids'] or
                not all(a < b for a,b in zip([accepted[0]['recv']]+[t for _,t in raw_unique], [t for _,t in raw_unique]))):
            raise ValueError('native waypoint completions missing or out of order')
        final_id, gx, gy = planned[-1]
        if (final_id != summary.get('final_waypoint_id') or
                math.dist([float(gx), float(gy)], goal) > .002 or
                math.dist(goal, summary.get('goal_xy', [])) > .002 or
                not goals[0]['recv'] <= accepted[0]['recv']):
            raise ValueError('raw final target disagrees with task')
        final_events = [r for r in done if DONE_EVENT.fullmatch(r['message']).group(2) == final_id]
        # Exact retransmissions are permitted; contradictory completion logs fail.
        if not final_events or len({r['message'] for r in final_events}) != 1:
            raise ValueError('missing or contradictory native final completion')
        completion = final_events[0]; fields = DONE_EVENT.fullmatch(completion['message']).groups()
        completion_gt = completion['gt']
        completion_valid = (fields[0] == route and fields[2] == 'tolerance' and
            float(fields[5]) <= .05 and completion_gt is not None and
            0 <= age(completion, completion_gt) <= .2 and
            completion_gt['t'] <= start+.2 and math.dist(completion_gt['xy'], goal) <= .05 and
            math.dist([float(fields[3]), float(fields[4])], completion_gt['xy']) <= .02 and
            abs(math.dist([float(fields[3]), float(fields[4])], goal)-float(fields[5])) <= .003 and
            completion['message'] == (summary.get('navigation_completion') or {}).get('message') and
            accepted[0]['recv'] < completion['recv'])
        truth = streams['/gazebo/odometry']; commands = streams['/model/omni_robot/cmd_vel']
        all_gt = [r for r in truth if start <= r['t'] <= end]
        all_cmd = [r for r in commands if r['t'] is not None and start <= r['t'] <= end]
        if len(all_gt) < 2 or len(all_cmd) < 2:
            raise ValueError('native terminal window missing')
        anchor = all_gt[0]['xy']
        def window(low, high):
            gt = [r for r in all_gt if low <= r['t'] <= high]
            cmd = [r for r in all_cmd if low <= r['t'] <= high]
            if len(gt) < 2 or len(cmd) < 2:
                return {'pass': False, 'reason': 'native subwindow missing', 'bounds': [low, high]}
            errors = [math.dist(r['xy'], goal) for r in gt]
            drifts = [math.dist(r['xy'], anchor) for r in gt]
            checks = {
                'gt_coverage': gt[0]['t'] <= low+.1 and gt[-1]['t'] >= high-.1,
                'final_command_coverage': cmd[0]['t'] <= low+.2 and cmd[-1]['t'] >= high-.2,
                'gt_sim_and_receive_continuity': seq([r['t'] for r in gt], .2) and seq([r['recv'] for r in gt], .2),
                'final_command_sim_and_receive_continuity': seq([r['t'] for r in cmd], .2, False) and seq([r['recv'] for r in cmd], .2),
                'clock_association_and_freshness': all(r['clock_matches'] for r in gt+cmd) and all(r['clock_sim_age_s'] <= .2 for r in gt),
                'odom_and_command_fresh_at_every_gt': all(0 <= r['odom_age_s'] <= .5 and r['odom_sim_age_s'] <= .2 and 0 <= r['cmd_age_s'] <= .2 for r in gt),
                'gt_stopped': max(r['speed'] for r in gt) <= .02 and max(r['wz'] for r in gt) <= .03,
                'final_command_stopped': max(r['speed'] for r in cmd) <= .02 and max(r['wz'] for r in cmd) <= .03,
                'near_goal_no_drift': max(errors) <= .10 and max(drifts) <= .03,
            }
            return {'pass': all(checks.values()), 'checks': checks, 'bounds': [low, high],
                    'gt_samples': len(gt), 'final_command_samples': len(cmd),
                    'max_gt_speed_mps': max(r['speed'] for r in gt),
                    'max_gt_yaw_speed_rad_s': max(r['wz'] for r in gt),
                    'max_final_speed_mps': max(r['speed'] for r in cmd),
                    'max_final_yaw_speed_rad_s': max(r['wz'] for r in cmd),
                    'max_goal_error_m': max(errors), 'max_stop_drift_m': max(drifts),
                    'max_gt_receive_gap_s': max(b['recv']-a['recv'] for a,b in zip(gt,gt[1:])),
                    'max_command_receive_gap_s': max(b['recv']-a['recv'] for a,b in zip(cmd,cmd[1:]))}
        first = window(start, middle); last = window(middle, end); full = window(start, end)
        final_gt = truth[-1]; final_cmd = commands[-1]; final_odom = streams['/ground/odometry'][-1]
        snap_wall = number(summary['goal_delivery']['publish_monotonic_ns'])*1e-9 + number(snapshot['wall_elapsed_s'])
        snap_command = snapshot['final_command']
        snap_values = [number(snap_command[k]) for k in ('vx','vy','wz')]
        snapshot_match = (
            abs(end-final_gt['t']) <= 1e-6 and math.dist(snapshot['gt_xy'], final_gt['xy']) <= .02 and
            abs(math.atan2(math.sin(number(snapshot['gt_yaw_rad'])-final_gt['yaw']), math.cos(snapshot['gt_yaw_rad']-final_gt['yaw']))) <= .01 and
            abs(number(snapshot['gt_speed_mps'])-final_gt['speed']) <= 1e-6 and
            abs(number(snapshot['gt_yaw_speed_rad_s'])-final_gt['wz']) <= 1e-6 and
            abs(number(summary['gt_goal_error_m'])-math.dist(final_gt['xy'], goal)) <= 1e-6 and
            math.dist(snapshot['odom_xy'], final_odom['xy']) <= .02 and
            abs(number(summary['odom_goal_error_m'])-math.dist(final_odom['xy'], goal)) <= 1e-6 and
            max(abs(a-b) for a,b in zip(snap_values, final_cmd['command'])) <= 1e-9 and
            0 <= snap_wall-final_gt['recv'] <= .2 and
            0 <= snap_wall-final_cmd['recv'] <= .2 and
            0 <= snap_wall-final_odom['recv'] <= .5 and
            abs(end-final_odom['t']) <= .2)
        checks = {'raw_goal_acceptance_completion_match': completion_valid,
                  'completion_before_declared_window': completion['recv'] <= all_gt[0]['recv']+.2,
                  'first_2s_pass': first['pass'], 'last_5s_pass': last['pass'],
                  'full_7s_pass': full['pass'], 'snapshot_matches_native': snapshot_match}
        return {'pass': all(checks.values()), 'checks': checks, 'thresholds': thresholds,
                'first_2s': first, 'last_5s': last, 'full_7s': full,
                'raw_route_id': route, 'raw_final_waypoint_id': final_id,
                'raw_completed_waypoints': [ident for ident,_ in raw_unique],
                'command_time_basis': 'monotonic reception and observed /clock; no Twist generation stamp'}
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        return {'pass': False, 'reason': str(error), 'thresholds': thresholds}


def verify(out=OUT, output_name='mission_verification_observed.json', verify_only=False):
    target = out / output_name if not verify_only else None
    if target is not None and (Path(output_name).name != output_name or target.exists()):
        raise FileExistsError(target)
    manifest = json.loads((out / 'input_manifest.json').read_text())
    name = manifest.get('trial_id', NAME)
    after = json.loads((out / 'input_sha256_after.json').read_text())
    raw = json.loads((out / 'raw_data_sha256.json').read_text())
    progress = json.loads((out / 'progress.json').read_text())
    preflight = json.loads((out / f'{name}.offline_preflight.json').read_text())
    summary = json.loads((out / f'{name}.summary.json').read_text())
    events = load_jsonl(out / f'{name}.events.jsonl')
    waypoint = raw_waypoint_sequence(events, summary, preflight)
    terminal = audit_run(out, name, sdf_walls())
    walls = walls_from_sdf(out / 'input_snapshot' / SDF)
    native_terminal = native_terminal_audit(out, summary, waypoint, name)
    try:
        native = native_audit(out, summary, walls, name)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        native = {'pass': False, 'reason': str(error), 'minimum': None,
                  'active_gt_receive_rate_hz': None}
    native.pop('gt_trace', None)
    input_checks = {name: item['sha256'] == after[name]['before'] ==
                    after[name]['after'] == sha(out/'input_snapshot'/name)
                    for name, item in manifest['inputs'].items()}
    resolved_path_checks = {name: after[name].get('resolved_path_before') == item['resolved_path']
                           and after[name].get('resolved_path_after') == item['resolved_path']
                           for name, item in manifest['inputs'].items()}
    raw_checks = {name: (out/name).exists() and sha(out/name) == value
                  for name, value in raw.items()}
    required = {f'{name}{suffix}' for suffix in
                ('.native.jsonl', '.csv', '.summary.json', '.events.jsonl',
                 '.plans.jsonl', '.actuation.jsonl',
                 '.published_commands.jsonl', '.runner.log')}
    required.add('launch.log')
    recorder_snapshot = out/'input_snapshot/tools/provincial_native_trace.py'
    if recorder_snapshot.is_file() and 'HEALTH_SCHEMA = 1' in recorder_snapshot.read_text():
        required.update({name + '.native.health.json', name + '.native.timing.jsonl'})
    from run_provincial_full_observed import local_input_closure
    missing_local_inputs = sorted(set(local_input_closure(manifest['inputs'])) -
                                  set(manifest['inputs']))
    parameter_hashes = json.loads((out/'runtime_parameter_sha256.json').read_text())
    parameter_checks = {key: (out/'runtime_parameters'/key).is_file() and
                        sha(out/'runtime_parameters'/key) == value
                        for key, value in parameter_hashes.items()}
    delivery = summary.get('goal_delivery') or {}
    checks = {
        'local_dependency_snapshot_coverage': not missing_local_inputs,
        'runtime_parameter_hashes_match': bool(parameter_checks) and all(parameter_checks.values()),
        'matched_navigation_receiver_and_acknowledgement':
            delivery.get('publish_count') == 1 and delivery.get('state') == 'accepted'
            and (delivery.get('readiness') or {}).get('ready') is True
            and (delivery.get('acceptance') or {}).get('route_id') == waypoint.get('route_id'),
        'pre_goal_input_snapshots_match_before_and_after': all(input_checks.values()),
        'resolved_input_paths_unchanged': all(resolved_path_checks.values()),
        'all_required_raw_files_hashed': required <= raw.keys(),
        'raw_files_match_closed_hashes': all(raw_checks.values()),
        'exactly_one_world_and_one_requested_trial': progress.get('launches') == 1
            and progress.get('goals_requested') == 1
            and len(progress.get('gazebo_server_processes', [])) == 1
            and progress.get('remaining_gazebo_servers') == [],
        'runner_exited_successfully': progress.get('runner_returncode') == 0,
        'navigation_completed': summary['result'] == 'terminal_pass',
        'terminal_full_2_plus_5_pass': terminal['pass'],
        'all_internal_waypoint_events_pass': waypoint['pass'],
        'native_record_pass': native['pass'],
        'native_terminal_2_plus_5_pass': native_terminal['pass'],
        'no_retry_stuck_false_success': summary['retry_count'] == 0
            and summary['stuck_count'] == 0 and not summary['false_success']
            and summary['geometric_overlap_samples'] == 0,
        'goal_yaw_and_start_match': summary['goal_xy'] == [8.7, 4.25]
            and abs(summary['sent_goal_yaw_rad']-math.pi/2) <= .05
            and abs(summary['final_gt_yaw_rad']-math.pi/2) <= .05
            and math.dist(summary['initial_gt_pose'][:2],
                          preflight['measured_start_gt'][:2]) <= .03,
    }
    old = json.loads((ROOT / 'tools/results/provincial_stage15_forward_integration_20260928'
                      / 'offline_clearance_review_v2'
                      / 'full_min_clearance_offline_audit.json').read_text())
    report = {
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'competition_arena_verified': False,
        'contact_data': 'CONTACT DATA UNAVAILABLE',
        'all_pass': all(checks.values()), 'checks': checks,
        'input_checks': input_checks, 'raw_checks': raw_checks,
        'resolved_input_path_checks': resolved_path_checks,
        'missing_local_capture_dependencies': missing_local_inputs,
        'runtime_parameter_checks': parameter_checks,
        'goal_delivery': delivery,
        'terminal': terminal, 'waypoint': waypoint, 'native': native,
        'native_terminal': native_terminal,
        'evaluator_version': 'native_terminal_v2_20260930',
        'summary': {'result': summary['result'],
                    'gt_goal_error_m': summary['gt_goal_error_m'],
                    'odom_goal_error_m': summary['odom_goal_error_m'],
                    'final_gt_yaw_rad': summary['final_gt_yaw_rad'],
                    'completion': summary['navigation_completion'],
                    'duration_sim_s': summary['duration_sim_s'],
                    'legacy_downsampled_gap_m':
                        summary['minimum_physical_wall_clearance_m']},
        'old_full_sampled_min_gap_m': .0844864394,
        'old_full_reference_source': str(ROOT / 'tools/results/provincial_stage15_forward_integration_20260928'
                                         / 'offline_clearance_review_v2'
                                         / 'full_min_clearance_offline_audit.json'),
        'old_review_sha256': sha(ROOT / 'tools/results/provincial_stage15_forward_integration_20260928'
                                 / 'offline_clearance_review_v2'
                                 / 'full_min_clearance_offline_audit.json'),
        'sampling_is_not_continuous_time_proof': True,
        'no_repeatability_conclusion_from_one_new_trial': True,
        'evaluator_sha256': sha(ROOT / 'tools/summarize_provincial_full_observed.py'),
        'command': f'python3 -B tools/summarize_provincial_full_observed.py --result-dir {out} ' +
            ('--verify-only' if verify_only else f'--output-name {output_name}'),
    }
    if target is not None:
        target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'all_pass': report['all_pass'],
                      'failed_checks': [k for k, v in checks.items() if not v],
                      'native_minimum': native['minimum'],
                      'gt_receive_rate_hz': native['active_gt_receive_rate_hz']}), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--result-dir', type=Path, default=OUT)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--output-name', default='mission_verification_observed.json')
    group.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    result = verify(out=args.result_dir, output_name=args.output_name,
                    verify_only=args.verify_only)
    if not result['all_pass']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
