#!/usr/bin/env python3
"""Scale a planar request without changing direction or safety thresholds.

Every candidate uses the same rectangle predictor and timing reserve as the
existing final guard. The final guard still independently checks the result.
This is sampled-model command limiting, not continuous-time certification.
"""
import math
from circle_geometry import CirclePlanarGapPredictor as PlanarGapPredictor

GOVERNOR_COMPUTE_BUDGET_S = .01


def limit_planar_command(pose, velocity, walls, pose_age, minimum_gap=.12):
    values = [*pose, *velocity, pose_age, minimum_gap]
    if (not all(math.isfinite(v) for v in values) or len(pose) != 3
            or len(velocity) != 3 or velocity[2] != 0.
            or not -.05 <= pose_age <= .05 or minimum_gap <= 0):
        raise ValueError('invalid planar governor input')
    speed = math.hypot(*velocity[:2])
    # The pose ages while candidates are evaluated. Reserve an additional
    # 10ms here; the caller stops if this new computation exceeds that budget.
    budget = max(0., pose_age) + .02 + .0125 + GOVERNOR_COMPUTE_BUDGET_S

    predictor = PlanarGapPredictor(pose, walls)

    def evaluate(scale):
        v = (velocity[0]*scale, velocity[1]*scale, 0.)
        gap = predictor(v[0], v[1])
        required = minimum_gap + speed*scale*budget
        return gap >= required + 1e-6, gap, required

    full_safe, gap, required = evaluate(1.)
    if full_safe or speed == 0.:
        return tuple(velocity), dict(scale=1., action='unchanged', gap_m=gap, required_gap_m=required)
    zero_safe, zero_gap, _ = evaluate(0.)
    if not zero_safe:
        return (0.,0.,0.), dict(scale=0., action='no_static_reserve_stop', gap_m=zero_gap, required_gap_m=minimum_gap)
    low, high = 0., 1.
    # Eight bounded evaluations avoid unbounded searches on the control thread.
    # Optimality is not claimed; the selected candidate is checked explicitly.
    for _ in range(8):
        mid = (low+high)/2
        if evaluate(mid)[0]: low = mid
        else: high = mid
    safe, gap, required = evaluate(low)
    if not safe:
        return (0.,0.,0.), dict(scale=0., action='candidate_rejected_stop', gap_m=gap, required_gap_m=required)
    return (velocity[0]*low,velocity[1]*low,0.), dict(
        scale=low, action='safe_speed_limited', gap_m=gap, required_gap_m=required)
