#!/usr/bin/env python3
"""Read-only closure of the bounded Stage 2 simulation experiment scope."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from summarize_provincial_stage15_completion import historical_hashes
from audit_trial import audit

BASE=ROOT/'tools/results/stage2_steps123_20261001'

def summarize():
    frozen=json.loads((ROOT/'tools/results/provincial_stage15_completion_20261001/provisional_stage15_completion_audit_v3.json').read_text())
    frozen_now={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in frozen['runtime_input_sha256'].items()}
    history=historical_hashes()
    trials={}
    for profile in ('zero','scale','yaw_bias','random_walk'):
        try:trials[profile]=audit(BASE/profile)
        except (OSError,KeyError,ValueError,TypeError,IndexError) as error:
            trials[profile]={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':str(error)}
    tests=(BASE/'tests_final.log').read_text()
    counterexamples=json.loads((BASE/'terminal_counterexamples_v1.json').read_text())
    observed={p:t for p,t in trials.items() if 'summary' in t}
    checks={'stage15_92_frozen_inputs_preserved':len(frozen_now)==92 and all(frozen_now.values()),
            'registered_historical_evidence_preserved':history['pass'],
            '19_tool_tests_pass':'Ran 19 tests' in tests and tests.rstrip().endswith('OK'),
            'seven_native_terminal_counterexamples_verified':counterexamples['all_pass'],
            'zero_error_regression_pass':trials['zero']['all_pass'],
            'all_three_error_experiments_recorded':all(p in observed for p in ('scale','yaw_bias','random_walk')),
            'no_retrying_error_runs':all(json.loads((BASE/p/'progress.json').read_text()).get('launches')==1 for p in observed),
            'no_residual_world_in_trial_records':all(json.loads((BASE/p/'progress.json').read_text()).get('remaining_gazebo_servers')==[] for p in observed),
            'all_error_model_recurrences_verified':all(trials[p].get('odometry',{}).get('pass') is True for p in ('scale','yaw_bias','random_walk'))}
    checks['all_experiments_have_valid_inputs_and_recording']=len(observed)==4 and all(
        all(t['checks'].get(k) is True for k in ('one_world_one_goal','truth_decoupled_graph',
            'inputs_snapshotted_unchanged','raw_hashes_match','runtime_parameter_hashes_match',
            'profile_runtime_matches','frozen_safety_and_control_parameters','odometry_separation_and_recording'))
        and all(t['native'].get('checks',{}).get(k) is True for k in
            ('native_receive_continuity','writer_evidence_complete','required_topics_observed',
             'raw_route_acceptance_matches_delivery_evidence','all_critical_native_fields_finite','native_critical_receive_order'))
        for t in observed.values())
    return {'requested_steps123_complete':all(checks.values()),'checks':checks,'trials':trials,
            'all_navigation_profiles_pass':len(observed)==4 and all(t['all_pass'] for t in trials.values()),
            'stage15_preservation':frozen_now,'historical_preservation':history,
            'scope':'Stage 2 steps 1–3 only; simulation fault injection, not hardware acceptance',
            'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,
            'mid360_imu_lio_integrated':False,'contact':'CONTACT DATA UNAVAILABLE',
            'continuous_time_safety_proven':False,'statistical_reliability_proven':False,
            'hardware_noise_calibrated':False,'assessment_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'command':sys.argv}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);a=p.parse_args()
    if a.output and a.output.exists():raise FileExistsError(a.output)
    result=summarize()
    if a.output:a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'requested_steps123_complete':result['requested_steps123_complete'],
                      'checks':result['checks'],'all_navigation_profiles_pass':result['all_navigation_profiles_pass']},ensure_ascii=False))
    sys.exit(0 if result['requested_steps123_complete'] else 2)
