"""Mutation tests of preserved forward-run event evidence."""

import copy
import contextlib
import hashlib
import io
import json
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import summarize_provincial_forward_integration as audit
from summarize_provincial_stage15_terminal_v3 import load_jsonl


SOURCE = audit.BASE


class WaypointEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.out = SOURCE / 'single_full'
        self.name = 'trial_full_01'
        self.events = load_jsonl(self.out / f'{self.name}.events.jsonl')
        self.summary = json.loads((self.out / f'{self.name}.summary.json').read_text())
        self.preflight = json.loads(
            (self.out / f'{self.name}.offline_preflight.json').read_text())

    def check(self, events):
        return audit.raw_waypoint_sequence(events, self.summary, self.preflight)

    def test_original_four_trials_have_complete_sequences(self):
        for mode in ('continuous_abc', 'single_full'):
            folder = SOURCE / mode
            for path in folder.glob('trial_*.summary.json'):
                name = path.name.removesuffix('.summary.json')
                summary = json.loads(path.read_text())
                preflight = json.loads(
                    (folder / f'{name}.offline_preflight.json').read_text())
                events = load_jsonl(folder / f'{name}.events.jsonl')
                self.assertTrue(audit.raw_waypoint_sequence(
                    events, summary, preflight)['pass'], name)

    def test_missing_first_and_middle_completion_fails(self):
        for waypoint in ('W00', 'W01'):
            modified = [event for event in self.events if
                        f'waypoint=R0001:{waypoint} ' not in
                        event.get('message', '')]
            self.assertFalse(self.check(modified)['pass'])

    def test_missing_final_completion_fails(self):
        modified = [event for event in self.events if
                    'waypoint=R0001:W02 ' not in event.get('message', '')]
        self.assertFalse(self.check(modified)['pass'])

    def test_out_of_order_completion_fails(self):
        modified = copy.deepcopy(self.events)
        first, second = [event for event in modified if
                         'RMUC waypoint diagnostic completed' in
                         event.get('message', '')][:2]
        first['wall_s'], second['wall_s'] = second['wall_s'], first['wall_s']
        self.assertFalse(self.check(modified)['pass'])

    def test_old_route_event_cannot_replace_completion(self):
        modified = copy.deepcopy(self.events)
        event = next(event for event in modified if
                     'waypoint=R0001:W00 ' in event.get('message', ''))
        event['message'] = event['message'].replace(
            'route=R0001 waypoint=R0001:W00',
            'route=R0000 waypoint=R0000:W00')
        self.assertFalse(self.check(modified)['pass'])

    def test_planned_coordinate_inconsistent_with_preflight_fails(self):
        modified = copy.deepcopy(self.events)
        event = next(event for event in modified if
                     'RMUC route diagnostic' in event.get('message', ''))
        event['message'] = event['message'].replace(
            'R0001:W01=(8.700,1.800)', 'R0001:W01=(8.700,2.000)')
        self.assertFalse(self.check(modified)['pass'])

    def test_identical_retransmission_does_not_add_a_waypoint(self):
        modified = copy.deepcopy(self.events)
        first_index = next(index for index, event in enumerate(modified) if
                           'waypoint=R0001:W00 ' in event.get('message', ''))
        modified.insert(first_index + 1, copy.deepcopy(modified[first_index]))
        result = self.check(modified)
        self.assertTrue(result['pass'])
        self.assertEqual(result['duplicate_delivery_count'], 1)
        self.assertEqual(len(result['completed_ids']), 3)

    def test_conflicting_duplicate_fails(self):
        modified = copy.deepcopy(self.events)
        first_index = next(index for index, event in enumerate(modified) if
                           'waypoint=R0001:W00 ' in event.get('message', ''))
        duplicate = copy.deepcopy(modified[first_index])
        duplicate['message'] = duplicate['message'].replace(
            'distance=0.018', 'distance=0.019')
        modified.insert(first_index + 1, duplicate)
        self.assertFalse(self.check(modified)['pass'])

    def test_aggregate_rejects_missing_intermediate_even_with_new_hash(self):
        with tempfile.TemporaryDirectory(prefix='stage15_forward_negative_') as tmp:
            folder = Path(tmp) / 'single_full'
            shutil.copytree(self.out, folder)
            (folder / 'mission_verification.json').unlink()
            path = folder / f'{self.name}.events.jsonl'
            events = [event for event in self.events if
                      'waypoint=R0001:W00 ' not in event.get('message', '')]
            path.write_text(''.join(json.dumps(item) + '\n' for item in events))
            hashes_path = folder / 'raw_data_sha256.json'
            hashes = json.loads(hashes_path.read_text())
            hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
            hashes_path.write_text(json.dumps(hashes))
            with patch.object(audit, 'BASE', Path(tmp)):
                with self.assertRaises(SystemExit):
                    audit.verify('full', verify_only=True)
            self.assertFalse((folder / 'mission_verification.json').exists())

    def test_read_only_reaudit_leaves_existing_report_unchanged(self):
        original = self.out / 'mission_verification.json'
        before = hashlib.sha256(original.read_bytes()).hexdigest()
        # Current tools legitimately evolve after a captured run. The original
        # raw stage remains valid; workspace drift must still be reported.
        manifest = json.loads((self.out/'input_manifest.json').read_text())
        drift = any(audit.digest(audit.ROOT/path) != value
                    for path, value in manifest['sha256'].items() if path != audit.SELF)
        results = []
        for _ in range(2):
            output = io.StringIO()
            status = 0
            with contextlib.redirect_stdout(output):
                try:
                    audit.verify('full', verify_only=True)
                except SystemExit as error:
                    status = error.code
            result = json.loads(output.getvalue().splitlines()[-1])
            self.assertEqual(status, 2 if drift else 0)
            self.assertEqual(result['stages'], {'trial_full_01': True})
            self.assertEqual(result['failed_checks'],
                             ['listed_current_inputs_except_auditor_match_capture']
                             if drift else [])
            results.append(result)
        self.assertEqual(results[0], results[1])
        self.assertEqual(before, hashlib.sha256(original.read_bytes()).hexdigest())


if __name__ == '__main__':
    unittest.main()
