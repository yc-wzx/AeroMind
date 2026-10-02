# Stage 1.5：终点证据、连续目标生命周期与 90° 朝向复核（2026-09-28）

状态：**本轮指令已完成；四个指定训练试验 PASS。** 数据仅适用于当前省赛暂定训练场，正式赛场尚未核验。本轮没有改导航算法、场地、GridRoute clearance（0.40 m）、EGO inflation（0.22 m）或速度参数，`imperfect_sensors=false`；未进入 Stage 2，也未运行 Full、Return、连续 A→B→C。

## 评估口径

评估器现在同时记录上游 `/cmd_vel`、自报 actuation diagnostic 和直接订阅的最终 `/model/omni_robot/cmd_vel`。终点 PASS 要求目标被接受、最终 waypoint 的完成事件匹配、Gazebo 真值接近目标、真值及最终输出连续稳定至少 2 个仿真秒，并继续观察 5 个仿真秒。GT 接收间隙或时钟跳变会重置停车窗口并使该次结果不能 PASS；缺少数据按缺失处理，不补零。保存后的独立复核重新读取 CSV、最终输出 JSONL 和 SDF 墙体，核对采样连续性、输出上限、终点快照及机器人矩形与墙的间距。

`tools/test_provincial_terminal_gate.py` 覆盖假完成、非零最终指令、GT 断流、缺少完成事件和快照不一致。它与原有几何测试共 9 项通过；相关 Python 文件编译检查通过。所有新试验的输入哈希与复核时文件一致。

## 指定试验结果

| 试验 | 起点 → 目标（m） | 初始/目标 yaw | Gazebo GT 到点误差 | odom 到点误差 | 最小采样车体-墙间距 | 仿真耗时 | 结论 |
| --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| A1 | (4.70, 0.50) → (4.70, 0.90) | 90° / 90° | 0.0182 m | 0.0182 m | 0.1900 m | 9.36 s | PASS |
| A2，同一 Gazebo、不重置 | 实测 (4.70, 0.882) 附近 → (4.70, 1.15) | 90° / 90° | 0.0090 m | 0.0090 m | 0.1900 m | 8.90 s | PASS |
| B90，独立 Gazebo | (4.70, 1.15) → (5.80, 1.80) | 90° / 90° | 0.0178 m | 0.0178 m | 0.1327 m | 24.66 s | PASS |
| C90，独立 Gazebo | (5.80, 1.80) → (8.70, 4.25) | 90° / 90° | 0.0178 m | 0.0178 m | 0.1400 m | 53.62 s | PASS |

四次均没有几何重叠采样、重试、卡住、false success 或完成后离开目标。每次 5 秒观察窗内的最大 GT 平移速度及直接订阅的最终指令平移速度均为 0。B/C 采样的最大绝对 yaw 偏差为 0；这是当前理想仿真模型的观测，不外推为实车转向性能。C90 曾出现 1 次 `stale_stop`，随后恢复，未造成重试或到点失败。接触传感器数据不可用，因此“无碰撞”仅以车体-墙几何不重叠及最小间距为依据，不能声称有独立接触检测。

A 双目标的最终 waypoint 完成事件依次为 `R0001:W00` 和 `R0002:W00`，与各自发出的目标匹配。独立生命周期观察器测得初始空闲期和两目标间最终指令最大平移速度均为 0；第二目标发出后最大平移指令为 0.25 m/s，说明完成停车状态允许新目标重新驱动。两次目标在同一个 Gazebo 进程中执行，未调用 set_pose。

与旧 yaw=0 的独立 B20/C27 相比，B90 的 GT 误差增加约 0.7 mm、最小间距减少约 7.2 mm；C90 的 GT 误差增加约 0.4 mm、最小间距增加约 30 mm。旧数据只作历史对照；它没有本轮直接订阅最终输出的证据，不以新判据追认 PASS。

## 代码审查的边界

当前导航接口在“路线全部完成且没有剩余 waypoint”时显式把最终指令置零；A 双目标已验证这一状态及新目标重新启动。若所有待发 waypoint 被动态占用，代码可进入“`active_waypoint is None` 但 `waypoints` 非空”状态；此状态未在本轮试验触发，旧 EGO 指令可能在 watchdog 超时前继续通过。该风险尚无实测复现，本轮没有为它更改导航链路或宣称已验证。后续若安排临时障碍/目标占用试验，应专门观测这个状态和最终输出，再决定是否需要最小修复。

## 证据位置与下一步

- A 双目标：`tools/results/provincial_stage15_goal_lifecycle_20260928/`，重点查看 `two_goal_summary.json`、`terminal_evidence_v2.json`、两次 `*.csv`、`*.published_commands.jsonl`、观察器日志和 `input_manifest.json`。
- B90/C90：`tools/results/provincial_stage15_yaw90_20260928/`，重点查看 `terminal_evidence_v2.json`、各次 `*.summary.json`、原始记录和 `input_manifest.json`。
- 历史对照：`tools/results/provincial_stage15_terminal_verified_20260928/`，未覆盖或改写。

指定 Stage 1.5 训练验证已经完成。正式省赛赛场仍需以可信尺寸更新 arena、PGM 和 collision 后重做离线连通/间距审计；此前不把这些训练路线升级为正式 benchmark，也不自动进入 Stage 2。
