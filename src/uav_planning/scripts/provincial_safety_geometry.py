#!/usr/bin/env python3
"""Pure 2-D geometry for the provisional provincial passage safety trial."""

import math
from functools import lru_cache


def rectangle(cx, cy, yaw, length, width):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [(cx + cosine*x - sine*y, cy + sine*x + cosine*y)
            for x, y in ((-length/2, -width/2), (length/2, -width/2),
                         (length/2, width/2), (-length/2, width/2))]


def wall_box(segment, thickness):
    x0, y0, x1, y1 = segment
    return rectangle((x0+x1)/2, (y0+y1)/2,
                     math.atan2(y1-y0, x1-x0),
                     math.hypot(x1-x0, y1-y0), thickness)


def point_segment_distance(point, a, b):
    vx, vy = b[0]-a[0], b[1]-a[1]
    scale = vx*vx + vy*vy
    ratio = 0.0 if scale < 1e-12 else max(0.0, min(1.0,
        ((point[0]-a[0])*vx + (point[1]-a[1])*vy)/scale))
    return math.hypot(point[0]-a[0]-ratio*vx, point[1]-a[1]-ratio*vy)


def polygon_distance(first, second):
    """Return zero for intersection, otherwise the Euclidean edge distance."""
    for shape in (first, second):
        for a, b in zip(shape, shape[1:] + shape[:1]):
            axis = (a[1]-b[1], b[0]-a[0])
            left = [p[0]*axis[0]+p[1]*axis[1] for p in first]
            right = [p[0]*axis[0]+p[1]*axis[1] for p in second]
            if max(left) < min(right) or max(right) < min(left):
                break
        else:
            continue
        break
    else:
        return 0.0
    return min(point_segment_distance(p, a, b)
               for shape, other in ((first, second), (second, first))
               for p in shape for a, b in zip(other, other[1:] + other[:1]))


@lru_cache(maxsize=1024)
def _wall_bounds(wall):
    return (min(p[0] for p in wall), min(p[1] for p in wall),
            max(p[0] for p in wall), max(p[1] for p in wall))


def body_wall_gap(x, y, yaw, walls, length=0.52, width=0.42):
    if not walls:
        raise ValueError('min() arg is an empty sequence')
    body = rectangle(x, y, yaw, length, width)
    # AABB separation is a lower bound; retain the original exact polygon
    # test for every wall that can improve the minimum. No samples are omitted.
    bounds = (min(p[0] for p in body), min(p[1] for p in body),
              max(p[0] for p in body), max(p[1] for p in body))
    candidates = []
    for wall in walls:
        wb = _wall_bounds(tuple(tuple(p) for p in wall))
        dx = max(0.0, bounds[0]-wb[2], wb[0]-bounds[2])
        dy = max(0.0, bounds[1]-wb[3], wb[1]-bounds[3])
        candidates.append((math.hypot(dx, dy), wall))
    minimum = math.inf
    for lower_bound, wall in sorted(candidates, key=lambda pair: pair[0]):
        # A small conservative tolerance protects against roundoff in bounds.
        if lower_bound > minimum + 1e-12:
            break
        minimum = min(minimum, polygon_distance(body, wall))
    return minimum


def predict_command_gap(x, y, yaw, vx_body, vy_body, wz, walls,
                        horizon=0.25, step=0.025, length=0.52, width=0.42):
    """Minimum swept-sample gap if the current body velocity stays constant.

    This is a short final-command guard, not a proof of the complete EGO spline.
    """
    minimum = body_wall_gap(x, y, yaw, walls, length, width)
    count = max(1, math.ceil(horizon/step))
    dt = horizon/count
    for _ in range(count):
        mid_yaw = yaw + wz*dt/2
        x += (math.cos(mid_yaw)*vx_body - math.sin(mid_yaw)*vy_body)*dt
        y += (math.sin(mid_yaw)*vx_body + math.cos(mid_yaw)*vy_body)*dt
        yaw += wz*dt
        minimum = min(minimum, body_wall_gap(x, y, yaw, walls, length, width))
    return minimum


def project_reference(point, reference):
    """Closest point and arclength on the configured corridor centre polyline."""
    best = (math.inf, None, None)
    walked = 0.0
    for index, (a, b) in enumerate(zip(reference, reference[1:])):
        vx, vy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(vx, vy)
        if length < 1e-9:
            raise ValueError('reference route has a zero-length leg')
        ratio = max(0.0, min(1.0,
            ((point[0]-a[0])*vx+(point[1]-a[1])*vy)/(length*length)))
        foot = (a[0]+ratio*vx, a[1]+ratio*vy)
        candidate = (math.dist(point, foot), walked+ratio*length,
                     index, foot)
        if candidate[0] < best[0]:
            best = candidate
        walked += length
    return best


def reference_stages(start, goal, reference, max_projection=0.20):
    """Add only intervening configured corners; retain the user's exact goal."""
    if len(reference) < 2:
        raise ValueError('reference route needs at least two points')
    start_distance, start_s, _, _ = project_reference(start, reference)
    goal_distance, goal_s, _, _ = project_reference(goal, reference)
    if max(start_distance, goal_distance) > max_projection:
        raise ValueError('start or goal is outside the configured centreline trial')
    arc = 0.0
    corners = []
    for a, b in zip(reference, reference[1:]):
        arc += math.dist(a, b)
        if min(start_s, goal_s)+1e-6 < arc < max(start_s, goal_s)-1e-6:
            corners.append(tuple(b))
    if goal_s < start_s:
        corners.reverse()
    return corners + [tuple(goal)]
