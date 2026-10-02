#!/usr/bin/env python3
"""Archive this diagnostic round without upgrading offline results to acceptance."""
import ast,hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'tools/results/stage2_step4_lio_replay_20261001'

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def load(p):return json.loads(p.read_text())
def write(p,data):
    with p.open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False)

def main():
    old={}
    for label,rel,key in [
        ('stage15','tools/results/provincial_stage15_completion_20261001/provisional_stage15_completion_audit_v3.json','runtime_input_sha256'),
        ('steps123_source','tools/results/stage2_scan_capture_startup_fix_20261001/final_evidence_seal.json','source_hashes'),
        ('steps123_evidence','tools/results/stage2_scan_capture_startup_fix_20261001/final_evidence_seal.json','evidence_hashes'),
        ('step4_source','tools/results/stage2_step4_lio_20261001/final_evidence_seal.json','source_hashes'),
        ('step4_evidence','tools/results/stage2_step4_lio_20261001/final_evidence_seal.json','evidence_hashes')]:
        entries=load(ROOT/rel)[key]
        mismatch=[p for p,h in entries.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
        old[label]={'checked':len(entries),'mismatches':mismatch,'pass':not mismatch}
    assert all(v['pass'] for v in old.values()),old
    cases={}
    for name,directory in [('baseline','baseline_analysis_v3'),('planar','planar_analysis_v1'),('instrumented_original','diagnostic_analysis_v1'),('first_point_fix','first_point_analysis_v1'),('declared_noise_model','noise_model_analysis_v1')]:
        a=load(OUT/directory/'analysis.json');r=a['replay']
        assert a['integrity_pass']
        cases[name]={'analysis':str((OUT/directory/'analysis.json').relative_to(ROOT)),
            'integrity_pass':a['integrity_pass'],'completion_time_LIO_GT_xy_m':r['at_completion'][3],
            'max_LIO_GT_xy_m':r['max_xy_error_m'],'terminal_time_LIO_GT_xy_m':r['at_terminal'][3],
            'max_height_deviation_m':r['max_height_deviation_from_declared_flat_base_m'],
            'parameter_overrides':a['parameter_overrides']}
    tests=load(OUT/'parameter_entry_tests_01/verification.json');assert tests['all_pass']
    progress=load(OUT/'model_full_01/progress.json')
    assert progress['launches']==1 and progress['goals_requested']==0
    incomplete=load(OUT/'model_full_01/independent_audit_v2.json');assert not incomplete['all_pass']
    modeldeps=['src/uav_bringup/config/spark_provincial_noise_model_sim.yaml','src/uav_bringup/launch/provincial_stage2_lio_noise_model.launch.py']
    sync={p:sha(ROOT/p)==sha(ROOT/'install/uav_bringup/share/uav_bringup'/Path(p).relative_to('src/uav_bringup')) for p in modeldeps}
    assert all(sync.values())
    ast_files=list((ROOT/'tools/stage2/lio_replay').glob('*.py'))+list((ROOT/'tools/stage2/lio_model_validation').glob('*.py'))
    for p in ast_files:ast.parse(p.read_text(),filename=str(p))
    summary={'status':'OFFLINE LIO NOISE MODEL IMPROVED / CLOSED-LOOP ACCEPTANCE PENDING',
        'round_fully_completed':False,'stage4_fully_accepted':False,
        'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,
        'contact':'CONTACT DATA UNAVAILABLE','prior_seals':old,'offline_replays':cases,
        'parameter_entry_tests':tests,'production_source_install_match':sync,
        'new_Full_attempt':progress,'new_Full_audit':incomplete,
        'production_changes':'New optional simulation-only LIO YAML and launch; no existing navigation, control, planner or sensor/world source modified.',
        'limitations':['Offline same-sensor replay is not new closed-loop navigation or goal accuracy.',
            'One preliminary instrumented replay used the wrong component and is excluded (diagnostic_01).',
            'analyze_replay corrected_cloud_pose_matches describes original output only; use separate point-boundary audit for candidate output.',
            'Covariance comes from synthetic white-noise model, not hardware calibration; discretization convention explicitly follows this filter Q.',
            'No statistical repeatability, continuous-time clearance, contact detection or real MID-360 scan fidelity proof.'],
        'next_step':'One fresh Full with repaired parameter entry, unchanged strict gates, then independent raw evidence audit; do not start hardware step5.'}
    write(OUT/'round_review.json',summary)
    report=ROOT/'docs/stage2_step4_lio_replay_and_noise_model_20261002.md'
    lines=['# Stage 2 第4项：离线定位诊断与仿真噪声模型修正','',
        '**本轮未全部完成；第4项尚未通过完整闭环验收。**',
        'TRAINING-ONLY / PROVISIONAL · COMPETITION ARENA NOT VERIFIED · CONTACT DATA UNAVAILABLE','',
        '## 完成的工作','',
        '在 DDS 域73/74独立回放旧 Full 的原始三维点云、IMU及仿真时钟。没有启动导航、发布目标或速度，也没有向 LIO 发布 GT。原始 CDR 内容不变，回放以0.5倍墙钟速度执行，消息自身时间戳保持原值。每次完整回放交付22688条IMU、908帧点云、22689条时钟，观察到905帧初始化后的点云校正。GT仅供离线误差计算；最近样本配对限25ms，不插值。','',
        '| 隔离实验 | 旧完成时刻 LIO–GT xy误差 | 全程最大xy误差 | 结论 |',
        '|---|---:|---:|---|']
    conclusions={'baseline':'原误差复现','planar':'未解决，未采用','instrumented_original':'诊断日志不改变完成时结果','first_point_fix':'点集缺陷修复，但定位误差未改善；未并入生产','declared_noise_model':'离线改善，新增可选仿真入口'}
    for name,c in cases.items():lines.append(f"| {name} | {c['completion_time_LIO_GT_xy_m']*100:.4f} cm | {c['max_LIO_GT_xy_m']*100:.4f} cm | {conclusions[name]} |")
    lines += ['',
        '上述误差是同一旧物理轨迹上的定位偏差，不是新导航到点误差。旧 Full 的 GT 到点误差5.7957cm超出完成事件5cm阈值，FAIL继续保留。','',
        '## 有依据的最小修正','',
        '22688组原生IMU与物理插件诊断配对，实测加速度标准差约0.001、角速度标准差约0.0001，均在声明值10%以内。旧滤波配置Q分别为0.01/0.001，且设置0.0001的偏置随机游走；本仿真没有注入偏置随机游走。滤波源码将参数直接放入Q对角线，以(dt·f_w)Q(dt·f_w)^T传播。','',
        '新增仿真专用配置acc_cov=1e-6、gyr_cov=1e-8、b_acc_cov=b_gyr_cov=0，保持3D模式。没有修改旧默认配置、滤波算法、导航、控制、车体、地图或传感器频率。该设置不应直接移植真实IMU。','',
        '原始去畸变循环存在首点重复处理：内层break没有退出外层IMU区间循环。隔离副本改为return后，76个检查帧中每帧1个不一致点变为0；但终点定位误差未改善，不能把它认定为本次Full失败的唯一原因。生产二进制未修改。','',
        '## 唯一一次新 Gazebo 尝试','',
        'model_full_01只启动一个独立Gazebo，参数预检阶段失败，外部目标发布次数为0。新运行器遗漏同目录dump_parameters.py入口，实际调用无法打开文件；不是导航运行后失败。保存输入快照175项及原始数据后关闭进程。独立审计为INCOMPLETE_EVIDENCE / NOT_EXERCISED；未经过boundary_7，没有新的终点、间距或2+5秒通过结论。','',
        '缺失入口反例已从运行时快照复现。现新增入口复用封存的参数读取器；查询保存命令、返回码、stdout/stderr及单调起止时间，启动前检查入口存在，运行器异常退出码为2。DDS域75真实ROS参数服务及缺失服务、超时、历史文件保护等6项检查全部通过。没有再次启动Gazebo。','',
        '## 文件和证据','',
        '- 新工具：tools/stage2/lio_replay/ 与 tools/stage2/lio_model_validation/。',
        '- 可选入口：src/uav_bringup/launch/provincial_stage2_lio_noise_model.launch.py。',
        '- 可选配置：src/uav_bringup/config/spark_provincial_noise_model_sim.yaml。',
        '- 本轮汇总：tools/results/stage2_step4_lio_replay_20261001/round_review.json。',
        '- 比较图：noise_model_analysis_v1/replay_comparison.png。',
        '- 原始/候选点集核验：original_point_boundary.json、fixed_point_boundary.json。',
        '- IMU诊断：imu_model_audit_v1.json。',
        '- 预检失败证据：model_full_01/progress.json、input_snapshot/、independent_audit_v2.json。',
        '- 入口修复反例及6项测试：parameter_entry_tests_01/。',
        '- 初始diagnostic_01误加载旧组件，随后发现并停止；保留原记录，不纳入有效实验。后续通过/proc/pid/maps核验实际动态加载组件路径和哈希。',
        '- baseline_analysis_v1为空的失败尝试、v2旧几何分析均保留；采用v3。其他分析内corrected_cloud_pose_matches仅描述原试验点云，不代表候选输出；候选用独立point-boundary核验。','',
        '## 基线保护与剩余工作','']
    for label,v in old.items():lines.append(f"- {label}：{v['checked']}项哈希核验，无差异。")
    lines += ['',
        '新配置及launch的source/install内容一致。系统依赖没有封存为完整系统镜像。接收序号不能证明发布端绝无丢包；回放不能提供新的接触检测、连续时间安全或统计重复性证据。','',
        '下一步仍是第4项闭环验证：使用修复后的可选入口运行一个新的Full，保持唯一外部目标、现有安全阈值及独立完整2+5秒/waypoint/原生采样审计。通过前不切换默认基线、不宣布第4项完成、不进入真实硬件第5项。']
    with report.open('x') as f:f.write('\n'.join(lines)+'\n')
    git_status=subprocess.run(['git','status','--short'],cwd=ROOT,capture_output=True,text=True,check=True).stdout
    with (OUT/'git_status_final.txt').open('x') as f:f.write(git_status)
    diff=subprocess.run(['git','diff','--binary'],cwd=ROOT,capture_output=True,check=True).stdout
    with (OUT/'git_diff_final.patch').open('xb') as f:f.write(diff)
    sources={}
    old4=load(ROOT/'tools/results/stage2_step4_lio_20261001/final_evidence_seal.json')['source_hashes']
    for p in old4:sources[p]=sha(ROOT/p)
    additions=ast_files+[ROOT/p for p in modeldeps]+[report]
    for d in ['diagnostic_build','first_point_build']:
        additions += [OUT/d/'libspark_lio_component.so',OUT/d/'spark_lio_mapping']
    for p in additions:
        if p.is_file():sources[str(p.relative_to(ROOT))]=sha(p)
    evidence={}
    for p in OUT.rglob('*'):
        if not p.is_file() or any(x in ['diagnostic_build','first_point_build','__pycache__'] for x in p.parts):continue
        evidence[str(p.relative_to(ROOT))]=sha(p)
    write(OUT/'final_evidence_seal.json',{'source_hashes':sources,'evidence_hashes':evidence,
        'round_fully_completed':False,'stage4_fully_accepted':False,'prior_seals_verified':old,
        'excluded_ephemeral_build_trees':['diagnostic_build','first_point_build'],
        'note':'Executed isolated binaries separately hashed; build trees retained but not asserted immutable.'})
    print(json.dumps({'offline_integrity_pass':True,'parameter_tests':tests['all_pass'],
        'stage4_fully_accepted':False,'old_seals':old,'source_count':len(sources),'evidence_count':len(evidence),'report':str(report)},indent=2))

if __name__=='__main__':main()
