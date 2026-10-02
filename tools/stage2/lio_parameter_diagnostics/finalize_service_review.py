#!/usr/bin/env python3
"""Seal the one preflight failure and read-only tool improvement separately."""
import ast,collections,hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'tools/results/stage2_step4_lio_live_noise_model_20261002'

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def load(p):return json.loads(p.read_text())
def write(p,data):
    with p.open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False)

def main():
    previous=load(ROOT/'tools/results/stage2_step4_lio_replay_20261001/final_evidence_seal.json')
    frozen={}
    for key in ['source_hashes','evidence_hashes']:
        bad=[p for p,h in previous[key].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
        frozen[key]={'checked':len(previous[key]),'mismatches':bad,'pass':not bad};assert not bad
    trial=OUT/'full_01';progress=load(trial/'progress.json')
    assert progress['launches']==1 and progress['goals_requested']==0 and not progress['remaining_gazebo_servers']
    tests=load(OUT/'parameter_reader_v2_tests_01/verification.json');assert tests['all_pass']
    hash_tests=(OUT/'parameter_hash_guard_tests.log').read_text();assert 'Ran 4 tests' in hash_tests and '\nOK\n' in hash_tests
    raw_hashes=load(trial/'raw_data_sha256.json');assert all(sha(trial/p)==h for p,h in raw_hashes.items())
    manifest=load(trial/'input_manifest.json');after=load(trial/'input_sha256_after.json')
    inputs_ok=all(sha(trial/'input_snapshot'/p)==v['sha256']==after[p]['before']==after[p]['after']
        and v['resolved_path']==after[p]['resolved_path_after'] for p,v in manifest['inputs'].items());assert inputs_ok
    counts=collections.Counter();commands=[];poses=[];goals=[]
    for line in (trial/'lio_full_01.native.jsonl').open():
        row=json.loads(line);counts[row['topic']]+=1
        if row['topic']=='/model/omni_robot/cmd_vel':commands.append(row['data'])
        if row['topic']=='/goal_pose':goals.append(row)
        if row['topic']=='/gazebo/odometry':poses.append(row['data']['pose']['pose']['position'])
    sensor_counts=collections.Counter(json.loads(line)['topic'] for line in (trial/'lio_full_01.sensors.jsonl').open())
    native_health=load(trial/'lio_full_01.native.health.json');sensor_health=load(trial/'lio_full_01.sensors.health.json')
    writer_ok=native_health['pass'] and sensor_health['pass'] and dict(counts)==native_health['received_counts']==native_health['written_counts'] and dict(sensor_counts)==sensor_health['received_counts']==sensor_health['written_counts']
    assert writer_ok and len(goals)==0
    max_command=max(abs(m[k][axis]) for m in commands for k in ['linear','angular'] for axis in ['x','y','z'])
    max_spawn_deviation=max(((p['x']-4.7)**2+(p['y']-.5)**2)**.5 for p in poses)
    query=load(trial/'runtime_parameters/lio_sensor_adapter.query.json')
    assert query['returncode']!=0 and 'parameter request timed out list_parameters' in query['stderr']
    summary={'classification':'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,
        'contact':'CONTACT DATA UNAVAILABLE','round_fully_completed':False,'stage4_fully_accepted':False,
        'status':'PARAMETER PREFLIGHT FAILED / READ-ONLY RETRY TOOL VERIFIED / FULL NOT_EXERCISED',
        'runtime_progress':progress,'independent_audit':load(trial/'independent_audit_v1.json'),
        'actual_input_items':len(manifest['inputs']),'input_snapshots_unchanged':inputs_ok,
        'raw_hashed_files':len(raw_hashes),'raw_hashes_match':True,'writer_counts_match_actual_lines':writer_ok,
        'native_counts':dict(counts),'sensor_counts':dict(sensor_counts),'max_observed_final_command':max_command,
        'max_GT_spawn_xy_deviation_m':max_spawn_deviation,'receive_statistics':load(OUT/'pregoal_native_summary.json'),
        'old_reader_unloaded_probe_successes':sum(r['returncode']==0 for r in load(OUT/'parameter_service_probe_01/old_reader_probe.json')),
        'read_only_v2_checks':tests['checks'],'parameter_hash_guard_tests':4,'prior_seal':frozen,
        'causality':'Live response-send timeout and client request timeout observed; unloaded node probe did not reproduce. Injected delayed-response test does not establish unique live root cause.',
        'next_runner':'tools/stage2/lio_model_validation_v2/run_model_experiment.py',
        'next_auditor':'tools/stage2/lio_model_validation_v2/audit_model_navigation.py',
        'limitations':['No Full goal sent, no waypoint/terminal/corridor validation this round.',
            'No new navigation or filter algorithm change; v2 alters read-only observation only.',
            'Runtime query files were not in original root-only raw hash list; separately preserved by this round final seal.',
            'Next runner captures runtime parameter files in a pre-goal hash manifest; no backfilling past runtime proof.',
            'No complete system image, contact detection, continuous-time safety or statistical repeatability proof.']}
    write(OUT/'round_review.json',summary)
    report=ROOT/'docs/stage2_step4_lio_parameter_preflight_review_20261002.md'
    text=f'''# Stage 2 第4项：闭环预检失败与参数读取工具修复

**本轮闭环验收未完成，第4项仍未完全通过。**
TRAINING-ONLY / PROVISIONAL · COMPETITION ARENA NOT VERIFIED · CONTACT DATA UNAVAILABLE

## 本轮实际执行

使用上轮修复的模型验证运行器，启动且仅启动一个独立Gazebo。离线预检通过：参考长度7.75m、GridRoute clearance=0.40m、yaw90矩形采样间距0.14m。后者为离线参考检查，不是新Full轨迹间距。

运行期成功读取/lio_mapping参数，确认3D模式、acc_cov=1e-6、gyr_cov=1e-8、两项偏置随机游走为0。随后/lio_sensor_adapter/list_parameters请求8秒超时，运行器保留错误并结束。launch.log同时记录服务端“failed to send response (timeout)”。没有重发请求以跳过本轮预检，没有启动第二次Gazebo。

外部目标次数为0，没有route接受或完成事件。独立审计为INCOMPLETE_EVIDENCE。导航Full、boundary_7、内部航点及完整2+5秒停车窗口均NOT_EXERCISED，不记为planner失败，也不能宣称已通过。

运行时实际输入快照{len(manifest['inputs'])}项，结束后内容及符号链接目标无变化。原始哈希清单{len(raw_hashes)}文件一致，native/sensor健康通过，接收、写入计数与实际文件行数一致。当前原始哈希清单未递归纳入runtime_parameters；本轮最终封存另外包含参数响应与query文件，不能追认为原始运行器已经封存这些文件。

## 本轮记录的启动期事实

GT494条，原生时间间隔最大约0.020s，单调接收最大间隔0.022391s，墙钟接收约50.01Hz；最终Twist494条，最大单调接收间隔0.038529s，约50.00Hz。最终Twist没有自身时间戳，不伪造消息生成时间。

记录范围约sim=0.02至9.88s，最终指令各轴最大绝对值{max_command}，GT距出生点最大偏差{max_spawn_deviation}m。这只支持“未下发目标时保持停车”，不能代替终点或全程安全审计。IMU原始2470条、适配2448条等差异保留，不由序号声称绝无丢包。

## 排查和最小工具改进

1. 在DDS域76隔离启动实际生产SensorAdapter，不输入传感器、GT或导航数据；旧读取器连续6次成功，现场问题未被该条件复现。不能宣称发现唯一根因。
2. 新增独立tools/stage2/lio_model_validation_v2/，旧运行器、工具和输入保持原样。新参数读取器预创建只读服务客户端，等待服务发现并推进响应发现，核对唯一节点身份；请求8秒响应时限保持，单服务最多3次、整个节点读取总预算20秒，失败请求取消，不复用陈旧future。
3. 每次查询保存服务名、次数、单调时间及结果；持续超时仍拒绝，参数缺失、NaN或数量不一致不补零、不输出成功YAML。重试仅针对只读参数服务，不重发导航目标，不set参数。
4. 同目录v2运行器维持唯一外部目标和全部已有门槛，增加实际参数及查询文件的发送目标前哈希留档；v2审计新增参数证据哈希、参数门槛检查，原严格审计不放宽。

在DDS域77，刻意延迟首个响应8.3秒：旧读取器失败，新读取器第二次收到正确匹配响应。持续延迟、NaN、缺失值均不能成功；真实SensorAdapter读取成功。共11项检查通过。该延迟是工具鲁棒性反例，不是对现场DDS超时原因的证明。

另有4项实际v2审计入口的参数哈希变异测试通过：正常证据、参数文件篡改、缺少manifest、重算哈希后的失败gate。原始导航审计在这4项测试中由fixture替代，只验证新增证据守卫，不冒称完整任务审计测试。

## 修改与归档

- 仅新增工具：tools/stage2/lio_model_validation_v2/、tools/stage2/lio_parameter_diagnostics/。
- 没有修改导航、最终速度、planner、LIO算法、LIO配置、车体、速度、安全距离、地图、物理步长或传感器频率。
- 本轮证据：tools/results/stage2_step4_lio_live_noise_model_20261002/。
- Full启动失败证据：full_01/progress.json、runtime_parameters/lio_sensor_adapter.query.json、launch.log、independent_audit_v1.json、input_snapshot/及全部native/sensors原始记录。
- 旧读取器隔离结果：parameter_service_probe_01/。
- 11项查询检查：parameter_reader_v2_tests_01/verification.json。
- 4项哈希变异检查：parameter_hash_guard_tests.log。
- 结构化结论：round_review.json；封存：final_evidence_seal.json。
- 上轮192项源文件、455项证据哈希复核无差异；其中原Stage1.5、步骤1–3和初版步骤4基线继续保留。

## 下一步

第4项仍需一次新的真实Full。使用v2工具入口和新的目录，先成功封存所有运行参数、确认唯一导航接收端、真实GT离线预检与传感器新鲜度，再只发(8.7,4.25), yaw90一个目标。仍采用全部内部waypoint、GT/odom、完整2+5秒及原生最小间距独立审计。此工具在真实Gazebo负载下是否解决参数响应问题尚未验证。

不将上轮离线4.75mm定位偏差当作新导航到点误差；保留旧Full完成事件5cm判据FAIL。没有接触、连续时间安全或统计重复性证据，不进入真实硬件第5项。
'''
    with report.open('x') as f:f.write(text)
    with (OUT/'git_status_final.txt').open('x') as f:f.write(subprocess.run(['git','status','--short'],cwd=ROOT,capture_output=True,text=True,check=True).stdout)
    additions=list((ROOT/'tools/stage2/lio_model_validation_v2').glob('*.py'))+list((ROOT/'tools/stage2/lio_parameter_diagnostics').glob('*.py'))
    for p in additions:ast.parse(p.read_text(),filename=str(p))
    sources=dict(previous['source_hashes'])
    for p in additions+[report]:sources[str(p.relative_to(ROOT))]=sha(p)
    evidence={str(p.relative_to(ROOT)):sha(p) for p in OUT.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    write(OUT/'final_evidence_seal.json',{'source_hashes':sources,'evidence_hashes':evidence,
        'prior_seal_verified':frozen,'stage4_fully_accepted':False,'runtime_goals':0,
        'note':'Runtime parameter retries only, not navigation retries; current Full NOT_EXERCISED.'})
    print(json.dumps({'report':str(report),'source_items':len(sources),'evidence_items':len(evidence),
        'actual_input_items':len(manifest['inputs']),'new_tool_checks':11,'hash_guard_tests':4,'stage4_fully_accepted':False},indent=2))

if __name__=='__main__':main()
