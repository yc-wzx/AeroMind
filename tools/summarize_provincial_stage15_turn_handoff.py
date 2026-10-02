#!/usr/bin/env python3
"""Summarize the final-code Stage 1.5 A/B/C trial set without rewriting old data."""
import json

from summarize_provincial_safety_fix import OUT, summarize


FINAL_RUNS = {
    'A': [f'trial_a_{index:02d}' for index in range(20, 28)],
    'B': [f'trial_b_{index:02d}' for index in range(14, 17)],
    'C': [f'trial_c_{index:02d}' for index in range(21, 24)],
}
PREVIOUS_C = [f'trial_c_{index:02d}' for index in range(4, 7)]
STARTUP_FAILURES = ['trial_a_07', 'trial_a_09', 'trial_a_16', 'trial_a_17']


def main():
    result = {
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'final_arena_verified': False,
        'current_code_trials': {},
        'prior_c_reference': [summarize(name, 'C') for name in PREVIOUS_C],
        'historical_startup_failures': [],
        'controlled_startup_guard': json.loads(
            (OUT/'ego_startup_guard_injected.json').read_text()),
        'limits': {
            'body_wall_gap_trial_m': 0.08,
            'contact_data': 'UNAVAILABLE',
            'no_full_or_return_route': True,
            'imperfect_sensors': False,
        },
    }
    for letter, names in FINAL_RUNS.items():
        runs = [summarize(name, letter) for name in names]
        result['current_code_trials'][letter] = {
            'runs': runs,
            'arrived': sum(run['result'] == 'success' for run in runs),
            'total': len(runs),
            'gt_error_range_m': [min(run['gt_goal_error_m'] for run in runs),
                                 max(run['gt_goal_error_m'] for run in runs)],
            'sampled_body_wall_gap_range_m': [
                min(run['min_gt_body_wall_gap_m'] for run in runs),
                max(run['min_gt_body_wall_gap_m'] for run in runs)],
            'total_retries': sum(run['retry_count'] for run in runs),
            'unsafe_plan_status_messages': sum(
                run['actuation_counts'].get('unsafe_plan', 0) for run in runs),
            'sampled_geometric_overlaps': sum(
                run['geometric_overlap_samples'] for run in runs),
            'all_pass_runner_assessment': all(
                run['assessment'] == 'PASS' for run in runs),
        }
    for name in STARTUP_FAILURES:
        data = json.loads((OUT/(name+'.summary.json')).read_text())
        result['historical_startup_failures'].append({
            'name': name,
            'result': data['result'],
            'published_plan_count': data.get('published_plan_count'),
            'retry_count': data['retry_count'],
        })
    path = OUT/'turn_handoff_aggregate.json'
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(path)


if __name__ == '__main__':
    main()
