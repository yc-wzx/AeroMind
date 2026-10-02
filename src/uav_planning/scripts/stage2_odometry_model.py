"""Synthetic planar dead-reckoning fault injection, not a sensor or LIO model.

Source twist is in body axes; pose is in world axes. Initial pose is known.
After initialization, drift profiles integrate increments without truth resets.
"""
from dataclasses import dataclass
import math
import random


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class ErrorParameters:
    scale_x: float = 1.0
    scale_y: float = 1.0
    yaw_bias_rad_s: float = 0.0
    position_walk_m_sqrt_s: float = 0.0
    yaw_walk_rad_sqrt_s: float = 0.0
    seed: int = 20261001
    max_source_gap_s: float = 0.2

    def __post_init__(self):
        vals = (self.scale_x, self.scale_y, self.yaw_bias_rad_s,
                self.position_walk_m_sqrt_s, self.yaw_walk_rad_sqrt_s,
                self.max_source_gap_s)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('nonfinite error parameter')
        if (self.scale_x <= 0 or self.scale_y <= 0 or
                self.position_walk_m_sqrt_s < 0 or self.yaw_walk_rad_sqrt_s < 0
                or self.max_source_gap_s <= 0):
            raise ValueError('invalid error parameter')

    @property
    def zero_error(self):
        return (self.scale_x == self.scale_y == 1.0 and
                self.yaw_bias_rad_s == self.position_walk_m_sqrt_s ==
                self.yaw_walk_rad_sqrt_s == 0.0)


class OdometryErrorModel:
    def __init__(self, parameters):
        self.parameters = parameters
        self.rng = random.Random(parameters.seed)
        self.previous = None
        self.pose = None
        self.elapsed_s = 0.0
        self.fault = None

    def advance(self, stamp_s, truth_pose, body_twist):
        if self.fault:
            raise ValueError('latched source discontinuity: ' + self.fault)
        if not all(math.isfinite(v) for v in (stamp_s, *truth_pose, *body_twist)):
            raise ValueError('nonfinite odometry')
        if self.previous is None:
            self.pose = tuple(truth_pose)
            self.previous = (stamp_s, *truth_pose)
            return self.pose, tuple(body_twist)
        old_t, tx, ty, tyaw = self.previous
        dt = stamp_s-old_t
        if dt == 0:
            raise ValueError('duplicate source timestamp')
        if dt < 0 or dt > self.parameters.max_source_gap_s + 1e-9:
            self.fault = 'backward clock' if dt < 0 else 'source gap'
            raise ValueError(self.fault)
        p = self.parameters
        if p.zero_error:
            # Exact identity is intentional for the Stage 1.5 equivalence test.
            self.pose, output_twist = tuple(truth_pose), tuple(body_twist)
        else:
            dx, dy = truth_pose[0]-tx, truth_pose[1]-ty
            c, s = math.cos(tyaw), math.sin(tyaw)
            bx = (c*dx+s*dy)*p.scale_x
            by = (-s*dx+c*dy)*p.scale_y
            dyaw = (wrap(truth_pose[2]-tyaw) + p.yaw_bias_rad_s*dt +
                    self.rng.gauss(0, p.yaw_walk_rad_sqrt_s*math.sqrt(dt)))
            x, y, angle = self.pose
            # Previous heading, not midpoint: zero-error increments reconstruct
            # source displacement exactly even when source orientation changes.
            c, s = math.cos(angle), math.sin(angle)
            walk = p.position_walk_m_sqrt_s*math.sqrt(dt)
            self.pose = (x+c*bx-s*by+self.rng.gauss(0, walk),
                         y+s*bx+c*by+self.rng.gauss(0, walk), wrap(angle+dyaw))
            output_twist = (body_twist[0]*p.scale_x, body_twist[1]*p.scale_y,
                            body_twist[2]+p.yaw_bias_rad_s)
        self.elapsed_s += dt
        self.previous = (stamp_s, *truth_pose)
        return self.pose, output_twist
