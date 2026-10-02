"""Real recorded physical evidence; mutations in memory only."""
import copy
import json
import unittest
from pathlib import Path
from audit_safe_navigation import physical_positions
ROOT = Path(__file__).resolve().parents[3]
FILE = ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002/short_position_imu_01/lio_short_01.native.jsonl'


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with FILE.open() as stream:
            cls.rows = [json.loads(line) for line in stream
                        if any(topic in line for topic in ['/simulation/imu_kinematics', '/gazebo/odometry'])]

    def changed(self, mutate):
        rows = copy.deepcopy(self.rows)
        mutate(rows)
        self.assertFalse(physical_positions(rows)['pass'])

    def test_original(self):
        self.assertTrue(physical_positions(self.rows)['pass'])

    def test_false_position(self):
        def mutate(rows):
            row = next(r for r in rows if r['topic'] == '/simulation/imu_kinematics')
            d = json.loads(row['data']['data']); d['world_position'][0] += .1
            row['data']['data'] = json.dumps(d)
        self.changed(mutate)

    def test_wrong_velocity(self):
        def mutate(rows):
            row = next(r for r in rows if r['topic'] == '/simulation/imu_kinematics')
            d = json.loads(row['data']['data']); d['world_velocity'][0] = .25
            row['data']['data'] = json.dumps(d)
        self.changed(mutate)

    def test_missing_record(self):
        def mutate(rows):
            indices = [i for i, r in enumerate(rows) if r['topic'] == '/simulation/imu_kinematics']
            rows.pop(indices[500])
        self.changed(mutate)

    def test_nonfinite_GT(self):
        def mutate(rows):
            row = next(r for r in rows if r['topic'] == '/gazebo/odometry' and r['message_stamp_s'] > 1.)
            row['data']['pose']['pose']['position']['x'] = float('nan')
        self.changed(mutate)

    def test_swapped_physical_record_order(self):
        def mutate(rows):
            indices = [i for i, r in enumerate(rows) if r['topic'] == '/simulation/imu_kinematics']
            a, b = indices[500:502]; rows[a], rows[b] = rows[b], rows[a]
        self.changed(mutate)


if __name__ == '__main__': unittest.main(verbosity=2)
