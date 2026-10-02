"""Negative tests on disposable copies of a recorded Gazebo trial."""

import csv
import json
import shutil
import tempfile
import unittest
from pathlib import Path

from summarize_provincial_stage15_terminal_v3 import (
    audit_run, full_window, mission_evidence, sdf_walls)


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tools/results/provincial_stage15_hold_resume_20260928/bc90_regression'
NAME = 'trial_b_22'


class FullWindowEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='stage15_v3_negative_')
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        for suffix in ('.summary.json', '.csv', '.published_commands.jsonl',
                       '.events.jsonl'):
            shutil.copy2(SOURCE / f'{NAME}{suffix}', self.out / f'{NAME}{suffix}')
        self.summary = json.loads((self.out / f'{NAME}.summary.json').read_text())

    def edit_csv(self, transform):
        path = self.out / f'{NAME}.csv'
        with path.open(newline='') as stream:
            reader = csv.DictReader(stream)
            fields = reader.fieldnames
            rows = list(reader)
        rows = transform(rows)
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def edit_jsonl(self, suffix, transform):
        path = self.out / f'{NAME}{suffix}'
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        rows = transform(rows)
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows))

    def test_unmodified_record_passes(self):
        self.assertTrue(full_window(self.out, NAME)['pass'])

    def test_speed_above_limit_during_first_two_seconds_fails(self):
        start = self.summary['stable_stop_start_sim_s']
        observe = self.summary['post_stop_observation_start_sim_s']

        def change(rows):
            changed = 0
            for row in rows:
                if start <= float(row['sim_t']) < observe:
                    row['gt_speed'] = '0.1'
                    changed += 1
            self.assertGreater(changed, 0)
            return rows

        self.edit_csv(change)
        self.assertFalse(full_window(self.out, NAME)['pass'])

    def test_nonzero_final_command_during_last_five_seconds_fails(self):
        observe = self.summary['post_stop_observation_start_sim_s']

        def change(rows):
            target = next(row for row in rows if row['gt_stamp_s'] > observe + 1)
            target['vx'] = 0.1
            return rows

        self.edit_jsonl('.published_commands.jsonl', change)
        self.assertFalse(full_window(self.out, NAME)['pass'])

    def test_missing_gt_chunk_fails(self):
        start = self.summary['stable_stop_start_sim_s']
        self.edit_csv(lambda rows: [row for row in rows
                                  if not start+0.4 < float(row['sim_t']) < start+0.9])
        self.assertFalse(full_window(self.out, NAME)['pass'])

    def test_missing_command_chunk_fails(self):
        start = self.summary['stable_stop_start_sim_s']
        self.edit_jsonl('.published_commands.jsonl',
                        lambda rows: [row for row in rows
                                      if row['gt_stamp_s'] is None or
                                      not start+0.4 < row['gt_stamp_s'] < start+0.9])
        self.assertFalse(full_window(self.out, NAME)['pass'])

    def test_missing_completion_event_fails(self):
        self.edit_jsonl('.events.jsonl', lambda rows: [row for row in rows
                        if 'RMUC waypoint diagnostic completed' not in
                        row.get('message', '')])
        self.assertFalse(full_window(self.out, NAME)['pass'])

    def test_inconsistent_terminal_snapshot_fails(self):
        self.summary['terminal_snapshot']['gt_xy'][0] += 0.1
        (self.out / f'{NAME}.summary.json').write_text(json.dumps(self.summary))
        self.assertFalse(full_window(self.out, NAME)['pass'])

    def test_bounded_missing_first_final_command_sample_is_reported(self):
        mission = mission_evidence(self.out, NAME)
        self.assertTrue(mission['pass'])
        self.assertEqual(mission['allowed_startup_missing_rows'], 1)

    def test_midrun_odom_nan_rejects_overall_pass(self):
        def change(rows):
            next(row for row in rows if float(row['sim_t']) > 5)['odom_x'] = 'nan'
            return rows
        self.edit_csv(change)
        self.assertFalse(audit_run(self.out, NAME, sdf_walls())['pass'])

    def test_midrun_gt_nan_rejects_overall_pass(self):
        def change(rows):
            next(row for row in rows if float(row['sim_t']) > 5)['gt_speed'] = 'nan'
            return rows
        self.edit_csv(change)
        self.assertFalse(audit_run(self.out, NAME, sdf_walls())['pass'])

    def test_midrun_final_command_nan_rejects_overall_pass(self):
        def change(rows):
            next(row for row in rows if float(row['sim_t']) > 5)['published_vx'] = 'nan'
            return rows
        self.edit_csv(change)
        self.assertFalse(audit_run(self.out, NAME, sdf_walls())['pass'])

    def test_second_missing_final_sample_is_not_startup(self):
        def change(rows):
            next(row for row in rows if float(row['sim_t']) > 5)['published_vy'] = 'nan'
            return rows
        self.edit_csv(change)
        self.assertFalse(mission_evidence(self.out, NAME)['pass'])


if __name__ == '__main__':
    unittest.main()
