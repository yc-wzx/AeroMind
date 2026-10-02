#!/usr/bin/env python3
"""Counterfactual read-only guard budgets; no goal/velocity publication or repair."""
import argparse,bisect,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
def load(p):return json.loads(p.read_text())

def assess(source,output):
    rows=[json.loads(l) for l in (source/'lio_full_01.native.jsonl').open()]
    relay=[r for r in rows if r['topic']=='/ground/odometry'];times=[r['receive_monotonic_ns'] for r in relay]
    diagnostics=[r for r in rows if r['topic']=='/ground/planning/actuation_diagnostics']
    goal=next(r['receive_sim_s'] for r in rows if r['topic']=='/goal_pose')
    radius=math.hypot(.52/2,.42/2);vmax=math.hypot(.25,.25);omega=.30
    yaw_budget=.01;tracking_budget=.01;pose_age_budget=.02;rotation_displacement=2*radius*math.sin(yaw_budget/2)
    lipschitz=vmax+radius*omega;command_discrete=lipschitz*.025/2;plan_discrete=lipschitz*.05/2
    observations=[]
    for r in diagnostics:
        d=json.loads(r['data']['data'])
        if r['receive_sim_s']<goal or d.get('action')!='allowed' or not any(abs(d[k])>1e-12 for k in ['vx','vy','wz']):continue
        i=bisect.bisect_right(times,r['receive_monotonic_ns'])-1
        if i<0:raise ValueError('No causal planning odometry record')
        age=r['receive_sim_s']-relay[i]['message_stamp_s']
        if not all(isinstance(d.get(k),(float,int)) and math.isfinite(d[k]) for k in ['predicted_gap_m','planned_path_min_gap_m']):
            raise ValueError('Missing/nonfinite active guard evidence')
        observations.append({'sim_s':r['receive_sim_s'],'waypoint':d['waypoint'],'observer_clock_minus_relay_stamp_s':age,
            'predicted_gap_m':d['predicted_gap_m'],'planned_gap_m':d['planned_path_min_gap_m'],
            'nonzero_final_command':[d[k] for k in ['vx','vy','wz']]})
    scenarios=[]
    for xy in [.01,.02,.04]:
        records=[]
        for r in observations:
            # Separate explicit age assumption; observer callback order cannot
            # independently identify the age inside the navigation node.
            margin=xy+rotation_displacement+tracking_budget+lipschitz*pose_age_budget
            point_reject=min(r['predicted_gap_m'],r['planned_gap_m'])<.08+margin
            sweep_reject=(r['predicted_gap_m']<.08+margin+command_discrete or r['planned_gap_m']<.08+margin+plan_discrete)
            records.append(dict(r,pointwise_assumed_required_gap_m=.08+margin,
                sampled_sweep_assumed_required_plan_gap_m=.08+margin+plan_discrete,
                pointwise_candidate_reject=point_reject,sampled_sweep_candidate_reject=sweep_reject))
        rejects=[r for r in records if r['pointwise_candidate_reject']]
        sweep_rejects=[r for r in records if r['sampled_sweep_candidate_reject']]
        w01=[r for r in records if r['waypoint']=='R0001:W01']
        scenarios.append({'assumed_xy_budget_m':xy,'observations':len(records),'pointwise_rejections':len(rejects),
            'first_pointwise_rejection':rejects[0] if rejects else None,'swept_sample_rejections':len(sweep_rejects),
            'first_sweep_rejection':sweep_rejects[0] if sweep_rejects else None,
            'W01_observations':len(w01),'W01_pointwise_rejections':sum(r['pointwise_candidate_reject'] for r in w01),
            'zero_age_centerline_pointwise_remaining_budget_m':.14-(.08+xy+rotation_displacement+tracking_budget),
            'zero_age_centerline_sweep_remaining_budget_m':.14-(.08+xy+rotation_displacement+tracking_budget+plan_discrete),
            'records':records})
    replay=load(ROOT/'tools/results/stage2_step4_lio_replay_20261001/noise_model_analysis_v1/analysis.json')
    minimum=load(source/'independent_audit_v1.json')['native']['minimum']
    result={'status':'OFFLINE BUDGET STUDY ONLY / NOT DEPLOYED',
        'assumptions':{'xy_budgets_m':[.01,.02,.04],'yaw_budget_rad':yaw_budget,'tracking_budget_m':tracking_budget,
            'assumed_pose_age_s':pose_age_budget,'pose_age_bound_independently_proven':False,
            'body_radius_m':radius,'max_speed_norm_mps':vmax,'max_yaw_rate_rad_s':omega,
            'command_swept_sample_extra_m':command_discrete,'plan_swept_sample_extra_m':plan_discrete,
            'base_physical_gate_m':.08,'reference_centerline_sample_gap_m':.14},
        'observed_failed_raw_sample':minimum,'scenarios':scenarios,
        'known_same_trajectory_noise_model_replay_max_xy_m':replay['replay']['max_xy_error_m'],
        'negative_observer_age_proxy_count':sum(r['observer_clock_minus_relay_stamp_s']<0 for r in observations),
        'candidate_error_budgets_calibrated':False,'continuous_time_safety_proven':False,
        'cannot_select_1_or_2cm_as_guaranteed_Full_bound':replay['replay']['max_xy_error_m']>.02,
        'limitations':['1/2/4cm, yaw and tracking budgets are explicit design assumptions, not certified error bounds.',
            'Position plus rotated-body displacement bounds geometry sensitivity under the stated assumptions only.',
            'Swept-sample bound additionally assumes known speed/yaw bounds, synchronized pose/time, unchanged geometry and correct motion model.',
            'Observer clock can precede an independently received relay stamp; signed differences retained, never clamped to zero or treated as actual navigation pose age.',
            '20ms age is an explicit study assumption, not a new freshness threshold or validated runtime bound.',
            'Counterfactual rejection on recorded data does not reproduce the changed closed-loop trajectory, stopping or resumption.',
            'Guard alone cannot force planner to provide a safer replacement; persistent rejected plans can cause a permanent hold.',
            'No new Full or production guard modification, no GT feedback or threshold reduction.']}
    with output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({'status':result['status'],'observations':len(observations),'summary':[{k:v for k,v in s.items() if k!='records'} for s in scenarios]},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();assess(a.result_dir,a.output)
