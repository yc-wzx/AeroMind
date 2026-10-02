"""Pure state machine for the provincial training trial's terminal evidence."""

import math


class TerminalGate:
    def __init__(self):
        self.last_gt_sim = None
        self.last_gt_wall = None
        self.stop_start_sim = None
        self.observation_start_sim = None
        self.anchor = None
        self.invalid_reasons = []

    def reset(self):
        self.stop_start_sim = None
        self.observation_start_sim = None
        self.anchor = None

    def check_gap(self, wall_now):
        if (self.stop_start_sim is not None and self.last_gt_wall is not None
                and wall_now-self.last_gt_wall > 0.2):
            self.reset()
            if 'GT reception gap during stop window' not in self.invalid_reasons:
                self.invalid_reasons.append('GT reception gap during stop window')

    def advance(self, *, wall_now, sim_t, gt_received_wall, xy, goal_error,
                speed, yaw_speed, odom_received_wall, odom_stamp,
                final_command_received_wall, final_command, completed):
        self.check_gap(wall_now)
        finite = all(math.isfinite(value) for value in
                     (sim_t, *xy, goal_error, speed, yaw_speed, odom_stamp,
                      *final_command))
        clock_ok = (self.last_gt_sim is None or
                    0 < sim_t-self.last_gt_sim <= 0.2)
        wall_gap_ok = (self.last_gt_wall is None or
                       0 <= gt_received_wall-self.last_gt_wall <= 0.2)
        if not clock_ok or not wall_gap_ok:
            self.reset()
            reason = 'GT clock discontinuity' if not clock_ok else 'GT reception gap'
            if reason not in self.invalid_reasons:
                self.invalid_reasons.append(reason)
        self.last_gt_sim = sim_t
        self.last_gt_wall = gt_received_wall
        fresh = (wall_now-gt_received_wall <= 0.2 and
                 odom_received_wall is not None and
                 wall_now-odom_received_wall <= 0.5 and
                 final_command_received_wall is not None and
                 wall_now-final_command_received_wall <= 0.2 and
                 abs(sim_t-odom_stamp) <= 0.2)
        stopped = (finite and fresh and clock_ok and wall_gap_ok and completed
                   and goal_error <= 0.10 and speed <= 0.02
                   and yaw_speed <= 0.03
                   and math.hypot(*final_command[:2]) <= 0.02
                   and abs(final_command[2]) <= 0.03)
        if not stopped:
            self.reset()
            return False
        if self.stop_start_sim is None:
            self.stop_start_sim = sim_t
            self.anchor = xy
        if math.dist(xy, self.anchor) > 0.03:
            self.reset()
            return False
        if self.observation_start_sim is None and sim_t-self.stop_start_sim >= 2.0:
            self.observation_start_sim = sim_t
        return (self.observation_start_sim is not None and
                sim_t-self.observation_start_sim >= 5.0)


def terminal_evidence_pass(*, result, accepted, final_waypoint_id, completion,
                           false_success, departure, stop_start_sim,
                           observation_start_sim, invalid_reasons, snapshot,
                           goal, minimum_wall_gap, overlap_count):
    if result != 'terminal_pass' or not accepted or not final_waypoint_id:
        return False
    if not completion or completion.get('waypoint_id') != final_waypoint_id:
        return False
    if not completion.get('gt_fresh') or completion.get('gt_goal_error_m') is None:
        return False
    if false_success or departure or invalid_reasons:
        return False
    if stop_start_sim is None or observation_start_sim is None:
        return False
    if observation_start_sim-stop_start_sim < 2.0:
        return False
    if not snapshot or not snapshot.get('gt_xy') or not snapshot.get('final_command'):
        return False
    if not math.isfinite(minimum_wall_gap) or minimum_wall_gap < 0.08 or overlap_count:
        return False
    values = (*snapshot['gt_xy'], snapshot['gt_speed_mps'],
              snapshot['gt_yaw_speed_rad_s'], *snapshot['final_command'].values(),
              *goal)
    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in values):
        return False
    return (abs(math.dist(snapshot['gt_xy'], goal)-snapshot['gt_goal_error_m']) < 1e-9
            and snapshot['gt_goal_error_m'] <= 0.10
            and snapshot['gt_speed_mps'] <= 0.02
            and snapshot['gt_yaw_speed_rad_s'] <= 0.03
            and math.hypot(snapshot['final_command']['vx'],
                           snapshot['final_command']['vy']) <= 0.02
            and abs(snapshot['final_command']['wz']) <= 0.03)
