# Stage 2 单次短距离扫描断流、停车与恢复核验

状态：TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED。

本轮指定的一次实跑、独立审计及失败分析已执行完毕；**严格验收未全部通过，不能归档为整体 PASS**。实际输入门控停车、恢复原任务、终点完整 2+5 秒的证据通过，但内部重试次数为 1，违反保留的零重试判据。没有自动重跑，没有启动 Full/Return/ABC，没有接入 MID-360/IMU/LIO。

## 本轮改动与输入

新增试验节点 `src/uav_planning/scripts/stage2_scan_relay.py`，并在 `src/uav_planning/CMakeLists.txt` 增加一个安装条目。新增独立 launch `src/uav_bringup/launch/provincial_stage2_scan_resume.launch.py`，只有该 launch 的 Stage2 定位接口将 `/scan` 重映射为 `/guarded_scan`。真实传感器、world、发布频率、物理步长均未修改。中继暂停时丢弃消息，恢复后只转发新接收的原消息，未缓存回放、改写消息或补零。

新增工具：

- `tools/stage2/run_scan_resume_experiment.py`：单次启动、输入封存、真实起点预检及节点图核对。
- `tools/stage2/run_scan_resume_trial.py`：唯一短目标、异步中继服务、逐消息订阅及刺激记录。
- `tools/stage2/audit_scan_resume.py`、`audit_scan_resume_v2.py`：独立断流证据和终点审计。
- `tools/stage2/test_scan_resume.py`、`test_scan_resume_v2.py`：生产中继/触发逻辑及反例测试。
- `tools/stage2/analyze_scan_resume.py`：只读统一时间轴、重试上下文和计划几何分析。

没有修改定位校正数学、Stage2 定位接口生产代码、最终速度输出逻辑、planner、executor、任何导航参数或场地。中继用于本次试验主动制造输入缺失，正常消息内容不变；GT 仅用于试验刺激、离线几何评估和不安全时终止试验，未反馈到定位或控制。

前后检查 Stage1.5 的 92 项封存输入全部匹配，原封存结论保留；上一轮 source/evidence seal 的 26 项检查也匹配。source/install 内容一致。36 个针对性/相关测试方法通过（15 个中继/刺激/断流审计、8 个已有输入保护、9 个定位、4 个真实记录短目标反例），子测试不另计数。

本次发送前封存实际本地依赖闭包 109 项，包含内容、解析路径/符号链接、SHA-256、Git 状态/diff、版本、启动命令；启动后记录实际参数及 SDF 雷达约定。运行后原封存内容和解析路径均匹配，原始文件关闭后数据哈希匹配。未封存完整系统镜像或所有动态加载系统依赖，不作完整系统可重建保证。v2 评估器及离线分析工具为运行结束后的新工具，不能冒充运行当时输入；运行时 v1 已保留在快照中。

固定参数：imperfect_sensors=false，GridRoute=.40m，EGO inflation=.22m，矩形间距门槛=.08m，zero 误差 profile；现有速度与车体不变。

## 一次实跑的原始结果

独立 Gazebo 数量 1，唯一外部目标数量 1。起点 GT=(4.7,.5)、yaw≈90°；目标=(4.7,1.15)、yaw≈90°，路线长 .65m，离线最小 grid clearance=.40m，90°矩形最小间距=.190m。接受 route=R0001 / waypoint=R0001:W00，确认延迟约 1.96ms（运行器单调接收时钟）。

| 事件/指标 | 原始证据复核结果 |
|---|---|
| 实际运动后暂停中继 | sim=16.14s，GT y≈.60121m，GT/最终指令约 .12410m/s |
| 最后接受扫描 | stamp=16.10s |
| 最终指令首次持续精确为零 | 接收关联 sim=16.62s；距扫描 stamp .520s；距接受诊断接收 .49807s |
| GT 达到停车门槛 | stamp=16.66s；距最后扫描 .560s |
| 进入暂停到停车最大采样位移 | .06973196m |
| 停车后观察 | 至 sim=18.66s，完整 2.00s；最大采样漂移 0m |
| 恢复中继 | sim=18.66s；丢弃 25 条扫描 |
| 新扫描接受/重新运动 | scan stamp=18.70s；非零最终运动指令接收关联 sim≈18.832s |
| 任务完成 | 原 R0001:W00 完成，GT 完成事件误差≈.01791m |
| 最终 GT / odom 到点误差 | .0171480m / .0162867m |
| 最终 yaw | 1.56870485rad（≈89.880°） |
| 完整 2+5 秒终点窗口 | [24.52,26.52] + [26.52,31.52]；CSV 与原生证据均通过窗口判据 |
| 总任务及观察耗时 | 17.08 仿真秒 |
| GT 原生采样 | 50Hz；仿真最大间隔 .020s；任务期最大单调接收间隔≈.02102s |
| 最终速度最大接收间隔 | ≈.04073s，小于 .2s |
| ground odom 最大接收间隔 | ≈.04070s，小于 .5s |
| 全程车体—墙采样最小间距 | .188905667m，sim=21.84s，boundary_5，GT≈(4.700551,.858133) |
| 卡死 / false success | 0 / 未观测到 |
| 内部重试 | **1，严格判据 FAIL** |

定位入口节点图确认：`gazebo_navigation_interface` 只订阅 `/guarded_scan`，没有订阅原始 `/scan`；`/guarded_scan` 唯一发布端为中继，原始 `/scan` 为真实 bridge。记录端身份/GID/QoS及稳定观察。GT 导航隔离图也通过。

原生扫描与中继扫描按时间戳、全部载荷比较一致；guarded stamp 单调无重复，暂停区间确实没有新消息；真实传感器继续输出。全部接受校正均有对应中继扫描，离线重新运行匹配与记录一致。恢复前最终输出保持精确零，恢复后有新的接受扫描，原任务原 waypoint 保留。无等待期间到达事件。

记录器 health/timing、接收计数、写入计数及实际文件行数匹配，无溢出或关闭失败。连续接收序号不证明发布端绝无丢包。没有监控全部 reset/set_pose 服务，未重置结论的范围是本轮运行器代码、命令和现有记录；没有声称完整服务调用捕获。

## 严格 FAIL 的核验与分类

v1 审计有三项失败。其一是本轮新入口调用通用 `native_audit()` 时未传 `expected_goal`，继承了 Full 的默认坐标 (8.7,4.25)。v2 按已封存的本次唯一目标 (4.7,1.15) 调用原有参数化接口，真实目标核验通过。错误、缺失、重复目标仍拒绝，四个原始记录临时副本测试通过。未改原始数据、旧报告或旧审计器。

v2 仍有两项失败：`full_2_plus_5_csv` 与 `no_retry_stuck_false_success`。二者来自同一个 **retry_count=1**，不是终点 7 秒停车窗口失败。实际完整窗口通过，但整体评估器还包括零重试门槛，因此不放宽门槛、不去掉该失败。

独立原始证据：sim≈18.56s，仍处于中继 hold（resume 为18.66s），最终动作是 stale_stop / vx=vy=wz=0，原 waypoint 保持 R0001:W00，日志为 `Retrying RMUC stage 1: 0.48 m from waypoint`，同一内部目标再次发布。

生产源代码中继承的 `update()` 在 sensor stale_stop 后仍执行阶段进度/重试分支：stage_sent_at 超过4s、低速、无进展超过2s、retry_count<4 即可触发。该分支未以 localization freshness 阻止“人为受控等待”被算成无进展。`observe_guard_resume()` 只在重新允许运动时重置进度时钟，不能阻止恢复前已发生的重试。源代码与原始事件共同支持这一具体机制；本轮没有修改它。

恢复后也记录到 unsafe_plan。sim=19.58s 的未来 Marker 轨迹延伸至 y≈1.947m，按最近接收 yaw 对车体矩形做只读采样，在 y≈1.940m 处与 boundary_7 间距为0；生产守卫同样记录 planned_path_min_gap=0 并输出零。实际 GT 全程间距≥.1889m。该计划风险被保护逻辑拒绝，不能把计划碰墙写成实际车体碰墙，也不能因此放行该计划。本轮的上下文 yaw 复算不是对生产未来 yaw 模型的完整重放，不单凭一次试验断言其规划成因。

最小下一步：先在隔离生产逻辑上复现 Stage2 定位缺失期间内部重试问题；只在 Stage2 层做最小暂停/恢复计时处理，保留 Stage1.5 的92项输入和所有阈值，明确有效定位恢复后的真实无进展仍可重试。先做隔离证据，不立即新增实跑，不改 planner。

## 可复核入口及证据

```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/audit_scan_resume_v2.py \
  --result-dir tools/results/stage2_scan_resume_20261001 \
  --output /tmp/scan_resume_review_NEW.json
```

输出路径必须不存在；返回非零是保留的零重试失败，不是审计崩溃。`independent_audit_v2_repeat.json` 复核同一证据，判定一致。

主目录：`tools/results/stage2_scan_resume_20261001/`。包括 `progress.json`、`input_manifest.json`、`input_snapshot/`、`input_sha256_after.json`、`runtime_parameters/`、`runtime_parameter_sha256.json`、`raw_data_sha256.json`、`odometry_graph.json`、`scan_graph.json`、`lidar_contract.json`、`hold_stimulus.json`、三版/复核审计及 trial 的 CSV/native JSONL/health/timing/events/plans/actuation/published_commands/日志。

离线诊断：`offline_analysis_v2/analysis.json`、`native_gt_timeline.csv`、`stop_resume_timeline.png`。v1 分析器的墙体容器适配错误日志和空结果目录保留，v2 为修正后的新增诊断，没有改动运行记录。

工具/检查日志：`tools/results/stage2_scan_resume_tools_20261001/`。

CONTACT DATA UNAVAILABLE。采样几何不重叠不等于独立接触传感器通过；本次短路线未覆盖 Full 的 boundary_7 最窄实际行驶区，不推导连续时间安全、毫米余量稳健、统计重复性或正式省赛验收。错误 frame/硬格式错误的新现场覆盖仍未做，仅保留已有隔离测试证据。
