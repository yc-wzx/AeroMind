"""Stage 2 terminal audit derived from frozen native_terminal_audit.
Only completion-log pose association changes from GT to recorded navigation
odom, with the same freshness and position tolerance. GT completion <= .05 m
and every original 2+5 s stopping/continuity/drift/safety threshold remain.
Stage 1.5 evaluator and historical evidence are unchanged.
"""
import json, math
from collections import defaultdict
from summarize_provincial_forward_integration import ROUTE_EVENT, PLAN_ITEM, DONE_EVENT
from summarize_provincial_stage15_terminal_v3 import load_jsonl
from summarize_provincial_full_observed import yaw

def stage2_native_terminal_audit(out, summary, waypoint, name=None, expected_stages=None):
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
                row['nav'] = latest.get('/ground/odometry')
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
            observed_nav = event['nav']
            if (source_route != route or ident not in plan_coordinates or reason != 'tolerance' or
                    observed_gt is None or not 0 <= age(event, observed_gt) <= .2 or
                    float(distance) > .05 or observed_nav is None or not 0 <= age(event, observed_nav) <= .5 or
                    abs(observed_nav['t']-observed_gt['t']) > .2 or
                    math.dist(point, observed_nav['xy']) > .02 or
                    abs(math.dist(point, plan_coordinates[ident])-float(distance)) > .003):
                raise ValueError('native waypoint completion inconsistent with route/navigation odometry')
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
        completion_nav = completion['nav']
        completion_valid = (fields[0] == route and fields[2] == 'tolerance' and
            float(fields[5]) <= .05 and completion_gt is not None and
            0 <= age(completion, completion_gt) <= .2 and
            completion_gt['t'] <= start+.2 and math.dist(completion_gt['xy'], goal) <= .05 and
            completion_nav is not None and 0 <= age(completion, completion_nav) <= .5 and
            abs(completion_nav['t']-completion_gt['t']) <= .2 and
            math.dist([float(fields[3]), float(fields[4])], completion_nav['xy']) <= .02 and
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
