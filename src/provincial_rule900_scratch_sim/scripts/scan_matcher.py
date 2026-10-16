"""Small 2-D lidar-to-known-wall matcher for the Gazebo navigation exercise."""

import math

import numpy as np
from scipy.optimize import least_squares


class WallScanMatcher:
    def __init__(self, segments, wall_half_width=0.0275):
        walls = np.asarray(segments, dtype=float).reshape(-1, 2, 2)
        self.starts = walls[:, 0]
        self.vectors = walls[:, 1] - walls[:, 0]
        self.length_squared = np.sum(self.vectors**2, axis=1)
        self.wall_half_width = wall_half_width

    def distances(self, points):
        offsets = points[:, None, :] - self.starts[None, :, :]
        along = np.clip(
            np.sum(offsets*self.vectors[None, :, :], axis=2)
            / self.length_squared[None, :], 0.0, 1.0)
        nearest = self.starts[None, :, :] + along[:, :, None]*self.vectors[None, :, :]
        return np.sqrt(np.min(np.sum((points[:, None, :]-nearest)**2, axis=2), axis=1))

    def match(self, pose, scan):
        ranges = np.asarray(scan.ranges[::4], dtype=float)
        indexes = np.arange(0, len(scan.ranges), 4)
        good = np.isfinite(ranges) & (ranges > max(scan.range_min, 0.12)) & (
            ranges < min(scan.range_max, 10.0))
        if np.count_nonzero(good) < 18:
            return None
        angles = scan.angle_min + indexes[good]*scan.angle_increment
        local = np.column_stack((ranges[good]*np.cos(angles),
                                 ranges[good]*np.sin(angles)))

        def transform(delta, local_points):
            yaw = pose[2]+delta[2]
            c, s = math.cos(yaw), math.sin(yaw)
            return np.column_stack((
                pose[0]+delta[0]+c*local_points[:, 0]-s*local_points[:, 1],
                pose[1]+delta[1]+s*local_points[:, 0]+c*local_points[:, 1]))

        predicted = self.distances(transform((0.0, 0.0, 0.0), local))
        inliers = predicted < 0.15
        if np.count_nonzero(inliers) < 16:
            return None
        local = local[inliers]

        def residual(delta):
            wall_error = self.distances(transform(delta, local))-self.wall_half_width
            # A weak odometry prior resolves feature-poor straight corridors.
            prior = np.array((delta[0]*0.2, delta[1]*0.2, delta[2]*0.4))
            return np.r_[wall_error, prior]

        solution = least_squares(
            residual, np.zeros(3), bounds=([-0.15, -0.15, -0.08],
                                           [0.15, 0.15, 0.08]),
            loss='soft_l1', f_scale=0.025, max_nfev=18)
        error = np.median(np.abs(residual(solution.x)[:-3]))
        if not solution.success or error > 0.04:
            return None
        return tuple(float(value) for value in solution.x)
