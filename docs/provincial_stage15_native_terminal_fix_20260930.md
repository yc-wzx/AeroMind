# Stage 1.5 原生终点证据修复与基线冻结（2026-09-30）

本轮指令全部完成。状态：`STAGE 1.5 PROVISIONAL FORWARD BASELINE AUDIT CLOSED`。

范围仍为 `TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`。本轮只做离线评估器修复、反例测试和归档；未启动 Gazebo，未发布生产目标。没有修改导航输出、控制、planner、记录器、场地、速度、频率或验收阈值。保留 imperfect_sensors=false、GridRoute clearance=0.40 m、EGO inflation=0.22 m。

## 修复与反例

旧 Full 独立汇总虽然检查了 native 连续性，但终点速度与漂移主要依据降采样 CSV，可能漏掉 native 中单条速度超限。已保存修复前反例：在临时副本中把 sim_t=76.28 s 的一条原生 GT 平移速度改为 0.1 m/s，仅更新副本数据哈希，CSV 不变。旧评估器 all_pass=true；本轮生产评估器 all_pass=false，明确失败在前 2 秒 gt_stopped。历史原始数据没有被修改。

实际修改的生产工具是 `tools/summarize_provincial_full_observed.py`。新增 `native_terminal_audit()` 并接入总 PASS，逐条检查所声明的前 2 秒、后 5 秒及完整 7 秒窗口，不寻找更晚窗口挽救失败。核验原生 GT/odom/最终指令有限性、时序、覆盖、新鲜度、时钟关联、速度、目标误差、漂移及快照一致性。独立解析接受事件、计划和所有内部 waypoint 的顺序与坐标，匹配本次完成事件。Twist 仍使用接收时间；仅保留目标发布前、第一条 clock 前尚无仿真时间的启动记录，不把它填零或豁免行驶阶段数据。

停车阈值维持 GT/最终平移速度 <=0.02 m/s、角速度 <=0.03 rad/s、目标误差 <=0.10 m、停车漂移 <=0.03 m；完成事件阈值独立保持 0.05 m。接收连续性 GT/最终指令 <=0.2 s，odom 新鲜度 <=0.5 s；原有其他门槛均保留。

新增 `tools/test_provincial_native_terminal.py`，调用真实生产汇总入口，以临时副本覆盖速度/角速度超限、NaN、字段缺失、断流、乱序、时钟错配、漂移/快照冲突、缺失或错误 route/waypoint 事件、计划不一致、健康/哈希门槛及不覆盖旧报告。最终 69 项测试全部通过，包括 30 项本轮针对性测试和 39 项相关已有回归。反例更新副本哈希，断流反例同时校正副本计数，防止仅凭哈希或计数失败掩盖被测漏洞。

## 原 Full 的新版本复核

原试验 `trial_full_capture_01`：一个独立 Gazebo、一个外部目标，(4.7,0.5), yaw=90° → (8.7,4.25), yaw=90°。本轮没有重新运行。原生外部目标记录为 1；R0001:W00、W01、W02 全部完成事件核验通过。原有终点/任务审计与新增 native 终点审计全部通过。

| 项目 | 原生重新计算结果 |
| --- | --- |
| 前 2 秒窗口 | 76.28–78.28 s，GT 101 条、最终指令 101 条，PASS |
| 后 5 秒窗口 | 78.28–83.28 s，GT 251 条、最终指令 250 条，PASS |
| 完整 7 秒窗口 | GT 351 条、最终指令 350 条（共享边界 GT 仅计一次），PASS |
| 最大 GT 平移速度 | 0.0190720623 m/s；后 5 秒为 0 |
| 最大 GT 角速度 | 0 rad/s |
| 最大最终平移/角速度 | 0 m/s、0 rad/s |
| 窗口最大 GT 到点误差 | 0.0178260992 m |
| 最大停车漂移 | 0.0000759606 m，约 0.076 mm |
| 最终 GT/odom 误差 | 均为 0.0177501386 m |
| 窗口最大 GT/最终指令接收间隔 | 0.0219659870 s / 0.0295902140 s |

全程原生 GT 共 3829 条，消息时间戳间隔约 0.02 s（50 Hz），最大消息间隔 0.0200000000 s、最大单调接收间隔 0.022536364 s。最终指令全程最大接收间隔 0.174222194 s；odom 为 0.176191884 s，均满足各自既有门槛。

全程原生采样车体—墙最小间距仍为 0.08414363861055252 m：sim_t=28.24 s，GT=(5.4575604284,1.8558563614)，yaw≈90°，最近墙 boundary_7。没有原始采样低于 0.08 m，采样裕量约 4.144 mm。此次评估器修复没有增加新运动样本，不改变旧几何分析，不构成连续时间安全或统计重复性证明。

## 溯源与可重复执行

原 Full 运行时封存的 42 项本地依赖、运行参数与 11 项原始数据哈希继续保留。本轮对原 Full 与前轮复核目录中预先登记的 153 个文件核对，均未改动。原 v1 和历史反例审计保留，新版本审计另存为 `mission_verification_capture_v2.json`。

当前工作区与原运行时快照的差异仅为本轮事后修改的汇总评估器，不能把当前评估器当作原试验运行时输入。新增 `evaluation_code_snapshot/` 与 `evaluation_code_manifest.json` 保存本轮 10 项本地评估依赖，明确标记 POST-RUN EVALUATION ONLY。系统依赖仅有版本记录，不是完整系统镜像。

复核入口（不覆盖现有报告）：

```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/summarize_provincial_full_observed.py \
  --result-dir /home/rmnav/AeroMind/tools/results/provincial_stage15_full_capture_20260929 \
  --verify-only
```

本轮写入 v2 时使用同一入口的 `--output-name mission_verification_capture_v2.json`，完整命令与评估器哈希保存在新审计。已有输出不可覆盖。

## 新增/修改文件与证据

- 修改：`tools/summarize_provincial_full_observed.py`（只影响离线判定，不影响导航最终输出）。
- 新增：`tools/test_provincial_native_terminal.py`。
- 新增冻结清单：`tools/config/provincial_stage15_forward_baseline_20260930.yaml`，含路线、固定参数、验收门槛、运行时及事后评估输入、原始数据、审计/测试哈希、限制与恢复触发条件。
- 新审计：`tools/results/provincial_stage15_full_capture_20260929/mission_verification_capture_v2.json`。
- 本轮完整修复证据：`tools/results/provincial_stage15_native_terminal_fix_20260930/`，含 before、changes.patch、原始文件哈希、修复前后反例、69 项 tests_final.log、事后评估快照及 fix_verification.json。
- 本报告：`docs/provincial_stage15_native_terminal_fix_20260930.md`。

## 冻结范围与下一步

仅冻结当前暂定训练场的单目标正向 Full 可复核基线。`CONTACT DATA UNAVAILABLE`，几何采样无重叠不是接触检测通过。没有证明所有 reset/set_pose 服务调用均被独立录制，没有证明发布端绝无丢包，没有证明旧接收停顿的唯一根因。没有新增 Return、障碍矩阵、正式省赛任务或重复性验收。

暂停扩大测试矩阵，不进入 Stage 2。恢复 Stage 1.5 的触发条件仍是本届省赛可信场地信息：起点、射击区、墙体边界、转角/连接通道宽度、固定障碍尺寸和位置。取得后先更新 arena/PGM/collision 并离线核验连通性、clearance，再决定实跑。当前暂定 0.8 m 通道和虚拟边界不升级为正式赛场结论。
