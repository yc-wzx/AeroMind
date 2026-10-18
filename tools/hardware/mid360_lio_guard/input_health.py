"""Fail-closed continuity and gross pose sanity; not accuracy acceptance."""
import math
from collections import deque

INPUT_LIMIT_NS = {'points': 200_000_000, 'imu': 100_000_000}
STATIC_SPAN_M, STATIC_ANGLE_DEG, MOTION_SPEED_M_S = .10, 10., 3.

class InputHealth:
    def __init__(self):
        self.latest = {}
        self.counts = {'points': 0, 'imu': 0}
        self.failure = None
        self.first_ns = None

    def fail(self, reason):
        if self.failure is None:
            self.failure = str(reason)
        return False

    def tick(self, now_ns):
        if self.failure:
            return False
        if len(self.latest) == 2:
            for kind, (_, receipt) in self.latest.items():
                if now_ns - receipt > INPUT_LIMIT_NS[kind]:
                    return self.fail(kind + ': receive gap; fresh LIO session required')
        return True

    def accept(self, kind, stamp_ns, receipt_ns):
        if kind not in INPUT_LIMIT_NS:
            return self.fail('Unknown stream')
        if not self.tick(receipt_ns):
            return False
        if type(stamp_ns) is not int or stamp_ns <= 0:
            return self.fail(kind + ': invalid sensor timestamp')
        old = self.latest.get(kind)
        if old:
            delta = stamp_ns - old[0]
            if delta <= 0 or delta > INPUT_LIMIT_NS[kind]:
                return self.fail(kind + ': regressed/duplicate/discontinuous sensor time')
        self.latest[kind] = (stamp_ns, receipt_ns)
        self.counts[kind] += 1
        if self.first_ns is None:
            self.first_ns = receipt_ns
        if len(self.latest) == 2 and abs(self.latest['points'][0] - self.latest['imu'][0]) > 200_000_000:
            return self.fail('LiDAR/IMU sensor time mismatch')
        return True

    def ready(self, now_ns):
        return bool(self.tick(now_ns) and len(self.latest) == 2
                    and self.counts['points'] >= 10 and self.counts['imu'] >= 100
                    and now_ns - self.first_ns >= 1_000_000_000)

class PoseHealth:
    """Engineering catastrophic-motion screening; stationary limits are explicit."""
    def __init__(self):
        self.rows = deque()
        self.last = None
        self.failure = None

    def accept(self, stamp_ns, receipt_ns, position, quaternion):
        if self.failure:
            return False
        if len(position) != 3 or len(quaternion) != 4 or not all(math.isfinite(v) for v in list(position)+list(quaternion)):
            self.failure = 'Nonfinite or malformed LIO pose'
            return False
        if abs(math.sqrt(sum(v*v for v in quaternion)) - 1.) > 1e-3:
            self.failure = 'Invalid LIO quaternion'
            return False
        row = dict(stamp_ns=stamp_ns, receive_monotonic_ns=receipt_ns,
                   position=list(position), quaternion=list(quaternion))
        if self.last:
            dt = (stamp_ns - self.last['stamp_ns']) / 1e9
            if dt <= 0 or dt > .2 or receipt_ns < self.last['receive_monotonic_ns']:
                self.failure = 'LIO corrected pose time discontinuity'
                return False
            if math.dist(position, self.last['position']) / dt > MOTION_SPEED_M_S:
                self.failure = 'LIO implied speed exceeds 3m/s gross-sanity gate'
                return False
        self.last = row
        self.rows.append(row)
        while self.rows and receipt_ns - self.rows[0]['receive_monotonic_ns'] > 6_000_000_000:
            self.rows.popleft()
        return True

    def static_report(self, minimum_s=4.8, start_ns=None):
        cutoff = start_ns if start_ns is not None else (self.last['receive_monotonic_ns']-5_000_000_000 if self.last else 0)
        rows = [r for r in self.rows if r['receive_monotonic_ns'] >= cutoff]
        span = (rows[-1]['receive_monotonic_ns'] - rows[0]['receive_monotonic_ns'])/1e9 if len(rows)>1 else 0.
        displacement = max((math.dist(r['position'], rows[0]['position']) for r in rows), default=0.)
        angle = max((math.degrees(2*math.acos(min(1., abs(sum(a*b for a,b in zip(r['quaternion'], rows[0]['quaternion'])))))) for r in rows), default=0.)
        return dict(pass_=bool(not self.failure and span >= minimum_s and displacement <= STATIC_SPAN_M and angle <= STATIC_ANGLE_DEG),
                    failure=self.failure, samples=len(rows), span_s=span,
                    max_displacement_from_first_m=displacement, max_rotation_from_first_deg=angle,
                    limits=dict(position_span_m=STATIC_SPAN_M, angle_deg=STATIC_ANGLE_DEG, gross_motion_speed_m_s=MOTION_SPEED_M_S),
                    meaning='Engineering gross-sanity only; operator-declared stationarity; NOT accuracy acceptance')
