#!/usr/bin/env python3
"""Seal one interrupted Full as FAIL, retaining successful tool gates separately."""
import hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'tools/results/stage2_step4_lio_live_noise_model_v2_20261002'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def load(p):return json.loads(p.read_text())
def write(p,data):
    with p.open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False)

def main():
    previous=load(ROOT/'tools/results/stage2_step4_lio_live_noise_model_20261002/final_evidence_seal.json');preservation={}
    for key in ['source_hashes','evidence_hashes']:
        bad=[p for p,h in previous[key].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
        preservation[key]={'checked':len(previous[key]),'mismatches':bad,'pass':not bad};assert not bad
    trial=OUT/'full_01';audit=load(trial/'independent_audit_v1.json');review=load(OUT/'clearance_review_01/review.json');summary=audit['summary']
    progress=load(trial/'progress.json');assert progress['launches']==progress['goals_requested']==1 and not progress['remaining_gazebo_servers']
    assert not audit['all_pass'] and review['minimum']['gap_m']<.08
    assert abs(audit['native']['minimum']['gap_m']-review['minimum']['gap_m'])<1e-12
    assert audit['checks']['runtime_parameter_evidence_hashes'] and audit['checks']['runtime_parameter_gate_passed']
    manifest=load(trial/'input_manifest.json');current_mismatch=[p for p,v in manifest['inputs'].items() if sha(ROOT/p)!=v['sha256'] or str((ROOT/p).resolve())!=v['resolved_path']];assert not current_mismatch
    result={'round_single_Full_and_independent_review_completed':True,'stage4_fully_accepted':False,
        'status':'LIO FULL INTERRUPTED: TRUTH CLEARANCE BELOW 0.08m',
        'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,
        'contact':'CONTACT DATA UNAVAILABLE','gazebo_count':1,'external_goal_count':1,'automatic_retries':0,
        'acceptance_delay_wall_s':summary['goal_delivery']['acceptance']['wall_elapsed_s'],
        'completed_waypoint_ids':audit['waypoint']['completed_ids'],'planned_waypoint_ids':audit['waypoint']['planned_ids'],
        'duration_sim_s':summary['duration_sim_s'],'duration_wall_s':summary['duration_wall_s'],
        'minimum':review['minimum'],'terminal_2_plus_5_exercised':False,
        'Full_remaining_GT_goal_distance_m':summary['gt_goal_error_m'],'Full_remaining_odom_goal_distance_m':summary['odom_goal_error_m'],
        'sampled_geometry_overlap_count':summary['geometric_overlap_samples'],
        'input_items':len(manifest['inputs']),'current_input_mismatch':current_mismatch,
        'runtime_parameter_files_hashed':len(load(trial/'runtime_parameter_sha256.json')),
        'strict_failed_checks':[k for k,v in audit['checks'].items() if not v],
        'task_max_LIO_GT_xy_error_m':review['task_max_LIO_GT_xy_m'],
        'prior_seal_preservation':preservation,
        'source_velocity_audit':'FAIL retained; 94 output-only recomputation mismatches before goal, none after goal. Actual initialization history differs from published-output-only history; independent full startup reconstruction still required.',
        'next_minimal_work':['Repair offline source velocity audit to reconstruct actual initialization/reset history, preserving strict thresholds and incomplete-evidence rejection.',
            'Design a conservative LIO navigation geometry guard accounting for localization/tracking margins before any new Full; no GT feedback or threshold relaxation.'],
        'limitations':['Only interrupted partial Full, cannot infer full-route localization accuracy or terminal performance.',
            'Safety violation is sampled geometry, not contact detection.',
            'First-attempt parameter reads succeeded this run; middleware timeout unique cause remains unproven.',
            'Raw startup receive gaps retained, despite active-task receive continuity passing.',
            'No complete system image, continuous-time safety, repeatability or competition arena verification.']}
    write(OUT/'final_review.json',result)
    m=review['minimum'];report=ROOT/'docs/stage2_step4_lio_full_clearance_failure_20261002.md'
    text=f'''# Stage 2 第4项：Full实跑与间距失败诊断

**本轮一次Full运行、独立审计和离线诊断已完成；结果FAIL，第4项尚未全部通过。**
TRAINING-ONLY / PROVISIONAL · COMPETITION ARENA NOT VERIFIED · CONTACT DATA UNAVAILABLE

## 实际运行

一个独立Gazebo、一个外部Full目标(8.7,4.25), yaw90，无额外外部中间目标、set_pose或自动重跑。reset/set_pose结论依据运行器代码和日志，没有宣称监控所有服务调用。

本次v2参数查询读取7个节点全部成功，每个服务首个请求得到响应，保存15个实际参数/查询/gate文件哈希。181项本地运行输入内容、路径和符号链接目标封存且运行前后无差异；未封存完整系统镜像。导航接收端、GID、QoS预检通过，唯一目标接受确认约{result['acceptance_delay_wall_s']*1000:.3f}ms。

R0001计划W00=(4.7,1.8)、W01=(8.7,1.8)、W02=(8.7,4.25)。只有W00完成；W01进行中因GT几何门槛停止。运行约{summary['duration_sim_s']:.2f}仿真秒，约{summary['duration_wall_s']:.2f}墙钟秒。未到最终目标，没有完整2+5秒终点窗口。中止时距最终目标GT约{summary['gt_goal_error_m']:.4f}m、odom约{summary['odom_goal_error_m']:.4f}m，不能称为“终点误差”。

## 原始证据与失败判据

独立审计all_pass=false。全部内部航点、任务完成、终点窗口、采样间距检查均未通过；实际LIO源速度重算另有启动期失败。目标交接、三维传感器任务区间连续性、原始哈希、运行参数证据、原速度与安全参数、GT未作为定位输入、导航只订阅LIO以及进程关闭检查通过。不得用这些局部通过代替总验收。

全程原生GT2097条，任务区间989条、50Hz、最大消息时间间隔0.020s。任务区间GT最大单调接收间隔0.023213s，最终指令0.064958s；原始全文件启动间隔仍保留，GT约0.352s、原生IMU约0.469s，不能宣称全文件严格连续。

在sim={m['sim_s']:.2f}s，GT=({m['x']:.9f},{m['y']:.9f})，yaw={m['yaw_rad']:.9f}rad，矩形车体距boundary_7最小采样间距={m['gap_m']:.12f}m，低于0.08m约{review['threshold_shortfall_m']*1e6:.3f}微米。试验运行器立即返回TRUTH_CLEARANCE_STOP并关闭本次Gazebo，不重跑、不修改阈值。小幅越界仍保留FAIL；不能据此忽略后续可能继续贴近。

没有记录几何重叠样本，但无接触传感器，继续CONTACT DATA UNAVAILABLE。停止时仍有非零最终运动指令；停止试验及关闭仿真不等于已验证生产安全停车或恢复。

## 几何、轨迹与定位偏差

boundary_7固定碰撞盒中心(6.3,2.2275,0.27)，尺寸(4.0,0.055,0.54)，近侧边界y=2.2m，x范围4.3–8.3m。

最窄点附近最新局部计划（stamp=41.272s）相近x处y约{review['plan_point_near_same_x'][1]:.9f}，比参考y=1.8m北移{review['plan_reference_cte_m']*100:.4f}cm。GT比该计划点北移{review['GT_minus_plan_y_m']*1000:.4f}mm，最近原始LIO样本比GT偏南约{-review['LIO_minus_GT_y_m']*1000:.4f}mm。相近x的计划点是空间诊断，不是同一时刻严格跟踪证明。

由该LIO位姿推算车体间距约{review['inferred_nearest_LIO_body_gap_m']:.6f}m。最近守卫诊断sim=41.86s，距GT最窄样本80ms，planned_path_min_gap=0.0812996125m、predicted_gap=0.0825482624m，动作仍allowed。二者基于估计位姿、假定轨迹，不能当作物理GT；任务中约几毫米横向定位/跟踪误差已超过计划保留的毫米级裕量。

任务已执行部分的最大LIO–GT xy偏差约{review['task_max_LIO_GT_xy_m']*100:.4f}cm，最后配对约{review['task_end_LIO_GT_xy_m']*100:.4f}cm。GT与LIO最近原始样本配对限25ms，不插值。不能把部分路线的改善推断为完整Full通过。机体系Twist正确按GT yaw换算成世界系，yaw90时body vy负值对应world x正方向，不能把它称为倒车。

已证实：计划本身明显偏离参考线；估计位姿下的安全余量不足以容纳观测到的偏差。尚未证实：具体优化项导致贴墙、噪声配置变化是否唯一造成这条计划、下一采样后物理间距和完整恢复表现。本轮不调planner或控制，不用GT做在线纠偏。

## 另一个未通过项：启动期速度源审计

原源审计使用“已发布LIO导航输出”重建0.20秒速度历史。94个不一致样本全部在启动期，约4.308–10.056s，外部目标在22.196s下发，之后没有该类残差。

第一处4.308s：原审计历史仅4个已发布样本，但生产适配器诊断记录真实内部窗口29个样本、0.112s。适配器在速度初始化期间会保留输入历史而不发布输出；原审计不能从输出独立恢复这部分历史。当前最大重算残差约0.014174834，不按任务时间范围豁免，不把diag自报当成独立证明。原始FAIL保留，下一步需从真实输入/初始化/重置事件重建，证据不足继续判不完整。

## 文件、图和保护

- 运行原始目录：tools/results/stage2_step4_lio_live_noise_model_v2_20261002/full_01/。
- 独立审计：full_01/independent_audit_v1.json；速度源、IMU物理链和启动期问题保留在其中。
- 只读诊断：clearance_review_01/review.json、minimum_window_raw_timeline.json、failed_full_clearance.png。
- 本轮结构化结论：final_review.json；封存：final_evidence_seal.json。
- 新增只读分析/封存工具：tools/stage2/lio_clearance_review/。
- 没有修改任何运行导航、LIO、控制、planner、参数、地图或物理/传感器频率；上轮202项源文件、222项证据哈希完全一致。
- 运行器入口：tools/stage2/lio_model_validation_v2/run_model_experiment.py，审计入口：tools/stage2/lio_model_validation_v2/audit_model_navigation.py。

## 最小下一步

先修只读速度源审计的初始化/重置历史重建，保留阈值和缺证失败；再针对LIO定位误差下的矩形安全守卫设计保守裕量及停车行为。实际运动链若需修改，必须单独封存旧基线、隔离验证后再做有界实跑。不能降低0.08m判据、手工修改地图/路线、固定用GT偏差补偿或盲目重复Full寻找成功。

本轮不进入真实设备第5项，不宣称连续时间安全、统计重复性或正式省赛场地验收。
'''
    with report.open('x') as f:f.write(text)
    with (OUT/'git_status_final.txt').open('x') as f:f.write(subprocess.run(['git','status','--short'],cwd=ROOT,capture_output=True,text=True,check=True).stdout)
    sources=dict(previous['source_hashes'])
    for p in list((ROOT/'tools/stage2/lio_clearance_review').glob('*.py'))+[report]:sources[str(p.relative_to(ROOT))]=sha(p)
    evidence={str(p.relative_to(ROOT)):sha(p) for p in OUT.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    write(OUT/'final_evidence_seal.json',{'source_hashes':sources,'evidence_hashes':evidence,
        'prior_seal_verified':preservation,'stage4_fully_accepted':False,'single_world_single_external_goal':True,
        'note':'Execution/audit complete; strict Full FAIL. No automatic retry or navigation modification.'})
    print(json.dumps({'report':str(report),'source_count':len(sources),'evidence_count':len(evidence),'stage4_fully_accepted':False,'minimum':m},indent=2))

if __name__=='__main__':main()
