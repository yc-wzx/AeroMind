# Stage 1.5 动态等待停车与恢复验证（2026-09-28）

**STAGE 1.5 HOLD / RESUME VALIDATION COMPLETE**。本结论只针对当前省赛暂定训练场（`TRAINING-ONLY / PROVISIONAL`）；`COMPETITION ARENA NOT VERIFIED`。本轮保持 `imperfect_sensors=false`、GridRoute clearance 0.40 m、EGO inflation 0.22 m、现有场地、车体与速度配置。未运行 Full、Return、连续 A→B→C，未进入 Stage 2。

## 修改与修复前反例

`src/uav_planning/scripts/gazebo_navigation_interface.py` 的省赛模式现在在没有 active waypoint 时明确区分初始空闲、路线完成和待发 waypoint 动态占用。三种状态在最终发布前强制输出零速度；有 active waypoint 时沿用原有新鲜轨迹、watchdog 和矩形安全检查。只读 actuation diagnostic 增加 `navigation_state` 与 `pending_waypoints`。

修复前对生产 `update()` 函数的隔离执行记录位于 `tools/results/provincial_stage15_hold_resume_20260928/pre_fix_reproduction.json`：当上游指令为 `vx=0.1 m/s`，初始空闲和待发目标被占用时最终指令均为 `0.1 m/s`，路线完成则为零。修复后 `post_fix_isolated.json` 中这三种状态均为零；新目标缺少安全轨迹时为零，有新鲜安全轨迹时允许 `0.1 m/s`，过期 odom 仍立即停车。六项隔离测试均通过。

`tools/summarize_provincial_stage15_terminal_v3.py` 现在从原始 CSV、最终输出 JSONL 和事件日志独立重建完整的 2+5 秒停车窗口，检查覆盖、单调性、间隙、速度、目标误差、漂移、事件匹配、快照和新鲜度。过去 v2 漏检的“前 2 秒 GT 速度被改为 0.1 m/s”反例在 v3 被拒绝。对数据副本的七项测试覆盖此前反例、后 5 秒非零输出、GT/指令缺口、缺少完成事件及快照不一致；均通过。

历史 A1/A2/B21/C28 原始数据按 v3 完整窗口重新核验，四次通过。原始运行时的文件哈希曾由 v2 保存并核对；当前导航文件经本轮修复，故与历史 manifest 的哈希不同。历史核验文件明确记录了这个差异，并使用原有 v2 哈希核验记录追溯历史输入；未改动原始试验数据或旧 v2 结论。

## Gazebo 定向场景

离线确认 `(4.7,0.5) → (4.7,1.15)` 静态连通，车体—墙采样最小间距约 0.19 m。临时障碍为目标处 `0.10 × 0.10 × 0.60 m` 静态实体，生成前与机器人车体间距约 0.34 m，与墙间距约 0.35 m；没有修改基础 SDF/PGM。先发送可达且空闲的目标，再生成障碍。临时实体已删除，Gazebo 进程已退出。

Gazebo 实际报告 `navigation_state=waiting_for_clear_waypoint`、`pending_waypoints=1`、`source=blocked_waypoint_hold`。此状态保持 3.00 仿真秒，53 个 GT 样本、159 个最终指令样本：上游 `/cmd_vel` 最大平移速度约 0.25 m/s，最终输出最大值为 **0**；GT 最大速度 0.00087 m/s，最大位移 0.00019 m。等待诊断出现后约 0.020 秒观察到下一条零指令。等待期机器人与障碍的最小采样几何间距约 0.339 m，无几何接触。障碍移除后约 0.332 秒首次恢复非零最终输出，原任务完成，GT 到点误差 0.01756 m；v3 完整终点审计通过。完成事件在移除障碍之后，未误报到达。

运行器在 `hold_trial_summary.json` 中留下一个未赋值的 `pass=false` 字段，但同文件的 `runner_returncode=0`、`status=RECOVERY_COMPLETED`、实际状态及原始数据均表明试验完成。该记账错误已在运行器源文件修正，不覆盖原记录；独立汇总 `hold_validation_final.json` 根据原始证据给出最终判定。

## 限定回归

| 试验 | Gazebo GT 到点误差 | 最小采样车体—墙间距 | v3 完整终点审计 |
| --- | ---: | ---: | --- |
| A1，同一 Gazebo 第一目标 | 0.01990 m | 约 0.190 m | PASS |
| A2，同一 Gazebo 第二目标 | 0.00739 m | 约 0.190 m | PASS |
| B90，独立 Gazebo | 0.01772 m | 0.13277 m | PASS |
| C90，独立 Gazebo | 0.01783 m | 0.14000 m | PASS |

A1/A2 的完成事件依次匹配 `R0001:W00` / `R0002:W00`，两目标间最终输出为零，第二目标发出后恢复最大 0.25 m/s 指令。B/C 各有一条最初 GT 样本早于最终指令回调，导致 v2 的“全程任何空值即失败”检查为假；v3 对完整终点窗口内的所有关键样本严格检查，两个终点窗口没有缺失，均通过。四次无重试、卡住或采样几何重叠。

## 证据和限制

- 修复前后隔离证据、总判定：`tools/results/provincial_stage15_hold_resume_20260928/`。
- 临时障碍的模型、原始 GT/odom/上游及最终指令、诊断、场景汇总：其下 `hold_trial_01/`。
- A 双目标回归：其下 `a_two_goal_regression/`。
- B/C 回归：其下 `bc90_regression/`，以 `terminal_evidence_v3_fullwindow.json` 为最终审计；初次 `terminal_evidence_v3.json` 保留了启动期缺值导致的旧 v2 判据失败。
- 历史四次完整窗口复核：原有 `provincial_stage15_goal_lifecycle_20260928/` 和 `provincial_stage15_yaw90_20260928/` 下各自的 `terminal_evidence_v3_historical.json`。

**CONTACT DATA UNAVAILABLE**：碰撞评估依赖采样车体与静态墙/临时障碍的几何间距，不能代替独立接触传感器。正式省赛场地尺寸、边界和固定障碍尚未核验，当前结果不能升级为正式赛场 benchmark。
