#!/usr/bin/env python3
"""Known-map 2-D scan correction. No ground truth or injected-error diagnostics.

Wall faces use exactly the training collision boxes, rather than treating
wall centre lines as laser surfaces. This is local tracking, not global
relocalization, LIO, or a calibrated probabilistic estimator.
"""
import math
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from provincial_safety_geometry import wall_box

SCAN_FRAME = 'omni_robot/base_link/lidar'


def validate_scan_metadata(scan):
    """Validate geometry, not individual no-return beams; never fill missing data."""
    fields = ('angle_min', 'angle_max', 'angle_increment', 'range_min',
              'range_max', 'time_increment', 'scan_time')
    if not all(math.isfinite(getattr(scan, k)) for k in fields):
        raise ValueError('nonfinite scan metadata')
    if (scan.angle_increment <= 0 or scan.angle_max <= scan.angle_min or
            scan.range_min < 0 or scan.range_max <= scan.range_min or
            scan.time_increment < 0 or scan.scan_time < 0 or len(scan.ranges) < 2):
        raise ValueError('invalid scan angles, range bounds, timing or sample count')
    last = scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment
    # LaserScan fields are float32; allow roundoff, not a missing whole beam.
    if abs(last-scan.angle_max) > max(1e-5, scan.angle_increment*.01):
        raise ValueError('scan angle span does not match sample count')
    # LaserScan.scan_time is time BETWEEN scans, not an acquisition-duration
    # guarantee. This tracker has no beam motion compensation; only the
    # instantaneous training scan (time_increment == 0) is supported.
    if scan.time_increment != 0:
        raise ValueError('rolling scan unsupported without motion compensation')


def verify_lidar_contract(world_path, frame=SCAN_FRAME):
    """Refuse unsupported mounts instead of silently applying zero extrinsics.

    This attests the configured SDF, not an independently queried running
    world. The isolated launch uses that same installed training SDF.
    """
    path = Path(world_path).resolve(strict=True)
    root = ET.parse(path).getroot()
    robot = root.find("./world/model[@name='omni_robot']")
    if robot is None:
        raise ValueError('missing omni_robot in configured SDF')
    link = robot.find("link[@name='base_link']")
    sensor = None if link is None else link.find("sensor[@name='lidar']")
    if (sensor is None or sensor.get('type') != 'gpu_lidar' or
            sensor.findtext('topic') != '/scan' or frame != SCAN_FRAME):
        raise ValueError('unsupported lidar sensor/topic/frame contract')
    for element, expected in ((link.find('pose'), (0.,)*6),
                              (sensor.find('pose'), (0., 0., .25, 0., 0., 0.))):
        values = tuple(float(v) for v in element.text.split()) if element is not None else (0.,)*6
        if (len(values) != 6 or not all(math.isfinite(v) for v in values) or
                any(abs(a-b) > 1e-9 for a,b in zip(values, expected)) or
                (element is not None and element.get('relative_to') not in (None, '', 'base_link'))):
            raise ValueError('unsupported lidar/base extrinsics or pose frame')
    return {'frame_id': frame, 'lidar_in_base_xyzrpy': [0., 0., .25, 0., 0., 0.],
            'resolved_sdf': str(path), 'sdf_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'scope': 'configured SDF; only co-located planar, aligned training lidar supported'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def transform(pose, local):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return local @ np.array([[c, s], [-s, c]]) + pose[:2]


class Correction:
    def __init__(self):
        self.offset = (0., 0., 0.)

    def apply(self, raw):
        tx, ty, a = self.offset
        c, s = math.cos(a), math.sin(a)
        return (tx+c*raw[0]-s*raw[1], ty+s*raw[0]+c*raw[1], wrap(a+raw[2]))

    def update(self, raw, delta, gain=.5):
        prior = self.apply(raw)
        desired = (prior[0]+gain*delta[0], prior[1]+gain*delta[1],
                   wrap(prior[2]+gain*delta[2]))
        a = wrap(desired[2]-raw[2])
        c, s = math.cos(a), math.sin(a)
        self.offset = (desired[0]-c*raw[0]+s*raw[1],
                       desired[1]-s*raw[0]-c*raw[1], a)


class FaceMatcher:
    def __init__(self, segments, thickness=.055):
        faces = []
        for segment in segments:
            box = wall_box(segment, thickness)
            faces.extend(zip(box, box[1:]+box[:1]))
        faces = np.asarray(faces)
        self.starts = faces[:, 0]
        self.vectors = faces[:, 1]-faces[:, 0]
        self.length2 = np.sum(self.vectors**2, axis=1)
        if not len(faces) or np.any(self.length2 <= 0):
            raise ValueError('invalid wall geometry')
        self.normals = np.column_stack((-self.vectors[:, 1], self.vectors[:, 0])) / np.sqrt(self.length2[:, None])

    def match(self, pose, scan):
        try:
            validate_scan_metadata(scan)
            if not all(math.isfinite(v) for v in pose):
                raise ValueError('nonfinite scan prior pose')
        except (ValueError, AttributeError, TypeError) as error:
            return {'accepted': False, 'reason': str(error)}
        ranges = np.asarray(scan.ranges[::4], dtype=float)
        angles = scan.angle_min + np.arange(0, len(scan.ranges), 4)*scan.angle_increment
        good = np.isfinite(ranges) & (ranges > max(.12, scan.range_min)) & (ranges < min(10., scan.range_max))
        local = np.column_stack((ranges[good]*np.cos(angles[good]), ranges[good]*np.sin(angles[good])))
        if len(local) < 24:
            return {'accepted': False, 'reason': 'insufficient finite returns'}
        points = transform(pose, local)
        offsets = points[:, None, :]-self.starts
        along = np.sum(offsets*self.vectors, axis=2)/self.length2
        nearest = self.starts + np.clip(along, 0., 1.)[:, :, None]*self.vectors
        distances = np.linalg.norm(points[:, None, :]-nearest, axis=2)
        index = distances.argmin(axis=1)
        selected = np.arange(len(local))
        valid = (distances[selected, index] < .12) & (along[selected, index] > 0.) & (along[selected, index] < 1.)
        local, index = local[valid], index[valid]
        if len(local) < 24:
            return {'accepted': False, 'reason': 'insufficient map-associated returns'}
        starts, normals = self.starts[index], self.normals[index]

        def residual(delta):
            return np.sum((transform(np.asarray(pose)+delta, local)-starts)*normals, axis=1)

        fit = least_squares(residual, np.zeros(3), bounds=([-.08, -.08, -.04], [.08, .08, .04]),
                            loss='soft_l1', f_scale=.01, max_nfev=15)
        # Refine face association after the first coarse fit. In particular a
        # shifted corner can initially pair with the far side of a 55mm wall.
        # Do not relax residual gates to absorb that wrong correspondence.
        original_count = len(local)
        for _ in range(2):
            points = transform(np.asarray(pose)+fit.x, local)
            offsets = points[:, None, :]-self.starts
            along = np.sum(offsets*self.vectors, axis=2)/self.length2
            nearest = self.starts+np.clip(along, 0., 1.)[:, :, None]*self.vectors
            distances = np.linalg.norm(points[:, None, :]-nearest, axis=2)
            index = distances.argmin(axis=1)
            selected = np.arange(len(local))
            valid = (distances[selected, index] < .05) & (along[selected, index] > 0.) & (along[selected, index] < 1.)
            local, index = local[valid], index[valid]
            if len(local) < max(24, .65*original_count):
                return {'accepted': False, 'reason': 'refinement lost map support'}
            starts, normals = self.starts[index], self.normals[index]
            fit = least_squares(residual, fit.x, bounds=([-.08, -.08, -.04], [.08, .08, .04]),
                                loss='soft_l1', f_scale=.01, max_nfev=15)
        errors = np.abs(residual(fit.x))
        # Gate all three tracking dimensions; a prior must not manufacture
        # information in a genuinely feature-poor corridor.
        eigen = np.linalg.eigvalsh(fit.jac.T @ fit.jac)
        condition = float(eigen[-1]/max(eigen[0], 1e-12))
        accepted = (fit.success and float(eigen[0]) > .5 and condition < 10000 and
                    float(np.median(errors)) < .012 and float(np.quantile(errors, .9)) < .025 and
                    np.all(np.abs(fit.x) < np.array([.0784, .0784, .0392])))
        return {'accepted': bool(accepted), 'reason': 'accepted' if accepted else 'fit/residual/observability gate',
                'delta': fit.x.tolist(), 'inliers': len(local), 'median_residual_m': float(np.median(errors)),
                'p90_residual_m': float(np.quantile(errors, .9)), 'information_eigenvalues': eigen.tolist(),
                'condition_number': condition, 'nfev': fit.nfev}
