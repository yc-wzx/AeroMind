#!/usr/bin/env python3
"""Archive evaluator repair and un-deployed guard analysis without changing baseline."""
import hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'tools/results/stage2_step4_lio_source_and_guard_20261002'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def load(p):return json.loads(p.read_text())
def write(p,data):
    with p.open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False)
def main():
    previous=load(ROOT/'tools/results/stage2_step4_lio_live_noise_model_v2_20261002/final_evidence_seal.json');frozen={}
    for key in ['source_hashes','evidence_hashes']:
        bad=[p for p,h in previous[key].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
        frozen[key]={'checked':len(previous[key]),'mismatches':bad};assert not bad
    source=load(OUT/'input_history_audit_v2.json');full=load(OUT/'independent_full_reaudit_v2.json')
    old_full=load(OUT/'historical_full_source_v2.json');old_short=load(OUT/'historical_short_source_v3.json')
    budget=load(OUT/'clearance_budget_v2.json');tests=(OUT/'mutation_tests.log').read_text()
    assert source['pass'] and old_full['pass'] and not old_short['pass']
    assert not full['all_pass'] and full['checks']['actual_LIO_and_physical_IMU_source_recomputed']
    assert 'Ran 9 tests' in tests and '\nOK\n' in tests
    summary={'round_read_only_audit_and_budget_work_complete':True,'stage4_fully_accepted':False,
        'status':'INPUT HISTORY AUDIT REPAIRED / SAFETY BUDGET COORDINATION REQUIRED',
        'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,'contact':'CONTACT DATA UNAVAILABLE',
        'Gazebo_launches_this_round':0,'goals_sent_this_round':0,'navigation_output_changes':False,
        'new_input_history_pass':source['pass'],'max_velocity_residual':source['max_velocity_residual'],
        'initialization_inputs_reconstructed':source['initialization_inputs_retained'],
        'mutation_tests_pass':True,'mutation_test_count':9,
        'original_full_input_history_pass':old_full['pass'],
        'historical_short_full_callback_history':'INCOMPLETE: raw7852/dispositions7851; no missing evidence filled',
        'strict_latest_Full_pass':full['all_pass'],'strict_failed_checks':[k for k,v in full['checks'].items() if not v],
        'guard_deployed':False,'budget_scenarios':[{k:v for k,v in s.items() if k!='records'} for s in budget['scenarios']],
        'frozen_preserved':frozen,
        'limitations':['Input ledger validates recorded input/output history, not independently recorded callback clock/freshness for two rejected callbacks.',
            'Hypothetical spatial/temporal uncertainty budgets are not calibrated or guaranteed Full bounds.',
            'No guard-only rejections on recorded traces can prove changed closed-loop safety, stopping or recovery.',
            'Original reports remain sealed; new scope exposes an incomplete historical short callback record.',
            'No contact, continuous-time safety, statistical repeatability or verified competition arena.'],
        'next_minimal_step':'Coordinate a separate LIO planning safety budget with final guard and explicit time/source observability, keeping the frozen baseline. Validate offline and in isolation before any new Full; do not simply raise a stop threshold or steer around rejected plans.'}
    write(OUT/'final_review.json',summary)
    report=ROOT/'docs/stage2_step4_lio_source_audit_and_safety_budget_20261002.md'
    lines=['# Stage 2 第4项：启动历史审计修复与安全预算评估','',
        '**本轮只读审计修复及预算分析完成；第4项尚未通过，安全守卫没有部署。**',
        'TRAINING-ONLY / PROVISIONAL · COMPETITION ARENA NOT VERIFIED · CONTACT DATA UNAVAILABLE','',
        '## 启动期审计修复','',
        '旧审计仅从已发布输出重建速度窗，漏掉实际生产回调中已入历史、尚未产生速度输出的初始化输入。新审计按原始/lio/odometry与/adapter_diagnostics逐次序列重建；期望位置与速度独立由原始位姿、固定外参、窗口公式计算，不使用诊断自报的body_twist作为期望值。诊断仅标明接受、初始化、重复/旧样本和拒绝分支，并校验序号关系、时间、窗口样本数和持续时间。','',
        f"上轮中止Full：原始输入10658、回调处置10658，保留66个初始化输入，独立复核10181个实际输出；最大速度残差{source['max_velocity_residual']:.12g}，小于原1e-7阈值。所有启动期样本参与，不按目标下发时间豁免、不补零、不删异常。位置阈值1e-9、历史0.20s、初始化至少0.10s均保持。",'',
        '9项检查通过：合法记录、缺初始化输入、初始化位置篡改、初始化NaN、输出速度篡改、缺回调处置、处置时间错序、缺输出、输出时间NaN。反例只改内存副本，原始文件不变。','',
        '完整Full独立汇总重新执行原任务审计，再用新的输入历史审计替换有缺陷的输出历史重算。IMU物理链等其他源检查仍执行，不放宽阈值。新增independent_full_reaudit_v2.json中速度源检查通过；全部任务总PASS仍false，航点未完成、终点窗口不存在、GT采样间距越界等原失败保持。','',
        '## 历史复核与证据限制','',
        '最初旧Full的完整输入历史也通过，最大速度残差约8.43e-11；旧Full的终点GT误差失败不因此改变。','',
        '历史short_05在新增加的完整回调历史核验下证据不完整：7852个原始输入、7851个处置事件，最后原始输入与输出为30.532s，但缺相应处置事件。原封存短距离报告及其已有终点通过结论保留；不能声称这个新检查也通过。没有补写事件、豁免或重跑历史试验。','',
        '两个拒绝事件缺原输入stamp和内部回调clock，无法独立证明其确切新鲜度判断；重建只遵循记录的拒绝分支并校验其他完整序列。此检查证明记录中的源代数及历史一致性，不证明发布端绝无丢包，也不替代原新鲜度、连续性和终点审计。','',
        '## 安全裕量离线评估','',
        '保留0.08m物理验收门槛。用1/2/4cm位置预算、0.01rad姿态预算、1cm跟踪预算和20ms位姿年龄假设，分析178条已记录allowed非零指令。以上全部为显式设计假设，不是已校准误差界或新的运行参数。','',
        '几何敏感性按位置误差 + 2r·sin(yaw_error/2)计算；若进一步讨论采样间运动，还需要速度、角速度、时序同步和运动模型假设。旧录制中观察者clock与独立odom到达顺序可能导致负年龄代理，原符号保留，不补零；20ms假设与这些代理值分开，不冒充导航内部年龄。','',
        '| 位置预算（假设） | 点式候选所需间距 | 拒绝/178 | W01拒绝/42 | 再加轨迹采样运动预算所需间距 |','|---|---:|---:|---:|---:|']
    for s in budget['scenarios']:
        records=s['records'];lines.append(f"| {s['assumed_xy_budget_m']*100:.0f}cm | {records[0]['pointwise_assumed_required_gap_m']:.6f}m | {s['pointwise_rejections']} | {s['W01_pointwise_rejections']} | {records[0]['sampled_sweep_assumed_required_plan_gap_m']:.6f}m |")
    lines += ['',
        '所有候选在已记录W01第二段都拒绝42/42。1/2cm候选首次会在37.268s拒绝整条局部计划，较GT越界41.94s早4.672s；这只是回放数据的判定差异，不证明改变闭环后实际能安全停下或恢复。4cm预算点式候选甚至拒绝部分第一段；叠加假设所需间距约0.1424m，已超过中心线名义0.14m间距。','',
        '不能为让路线通过随意选1/2cm：已封存噪声模型离线回放全程最大xy偏差约3.388cm，超过2cm；而4cm也不是未来误差的保证界。当前身体/运动/定位预算若都要求保守覆盖，路径、输入时序与守卫需要共同设计。','',
        '结论：单独提高停车门槛会反复拒绝相同不安全计划，可能形成永久等待；不能把“更早拒绝”写成Full恢复。当前不部署该补丁、不降低物理门槛、不手工改场地，也不通过削减指令某一轴绕过unsafe_plan。','',
        '## 文件与执行入口','',
        '- 新只读工具：tools/stage2/lio_source_guard_audit/。',
        '- 本轮数据：tools/results/stage2_step4_lio_source_and_guard_20261002/。',
        '- 原输入历史：input_history_audit_v2.json；Full重新汇总：independent_full_reaudit_v2.json。',
        '- 历史源检查：historical_full_source_v2.json、historical_short_source_v3.json。',
        '- 反例测试：mutation_tests.log；预算分析：clearance_budget_v2.json。',
        '- 最初年龄代理分析因负值退出，没有生成成功报告；失败工具版本保留于tool_versions/。历史CLI在证据不足分支出现打印KeyError的报告也保留，v3已修复打印并保持拒绝。',
        '- 新汇总入口（须source ROS/install）：python3 -B tools/stage2/lio_source_guard_audit/reaudit_full.py --result-dir <原试验目录> --output <全新报告路径>。',
        '- final_review.json与final_evidence_seal.json封存本轮结论；旧205项源文件和237项证据哈希无变化。','',
        '本轮未启动Gazebo、未发送目标；没有修改任何运行导航、控制、planner、LIO算法/配置、最终速度、安全参数、车体、场地、物理步长或传感器频率。','',
        '## 下一步','',
        '单独设计LIO训练模式的规划安全预算与最终守卫一致性，并补齐导航内部位姿/时序和回调源观测。先确保安全拒绝后能够得到满足相同裕量的新计划；若原规划配置无法表达该预算，应记录具体约束冲突，不能盲重跑或增加路线专用控制。旧基线保留独立入口。完成离线和隔离验证后再考虑一次有界Full。','',
        'Stage4未完成，不进入真实硬件第5项，不宣称正式省赛验收、接触检测、连续时间安全或统计重复性。']
    with report.open('x') as f:f.write('\n'.join(lines)+'\n')
    with (OUT/'git_status_final.txt').open('x') as f:f.write(subprocess.run(['git','status','--short'],cwd=ROOT,capture_output=True,text=True,check=True).stdout)
    sources=dict(previous['source_hashes'])
    for p in list((ROOT/'tools/stage2/lio_source_guard_audit').glob('*.py'))+[report]:sources[str(p.relative_to(ROOT))]=sha(p)
    evidence={str(p.relative_to(ROOT)):sha(p) for p in OUT.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    write(OUT/'final_evidence_seal.json',{'source_hashes':sources,'evidence_hashes':evidence,
        'old_baseline_verified':frozen,'stage4_fully_accepted':False,'guard_deployed':False,
        'note':'Read-only evaluator repair and candidate budgets only. Full FAIL and short new-check incomplete evidence retained.'})
    print(json.dumps({'report':str(report),'source_items':len(sources),'evidence_items':len(evidence),
        'source_audit_fixed':True,'Full_pass':False,'guard_deployed':False},indent=2))

if __name__=='__main__':main()
