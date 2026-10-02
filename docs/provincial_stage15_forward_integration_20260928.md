# Stage 1.5 暂定训练场正向任务集成（2026-09-28）

**STAGE 1.5 TRAINING FORWARD INTEGRATION VERIFIED**，仅适用于当前 `TRAINING-ONLY / PROVISIONAL` 场地；`COMPETITION ARENA NOT VERIFIED`。本轮未修改导航算法、planner 参数、clearance、EGO inflation、车体尺寸、限速、PGM 或 collision，未进入 Stage 2，未运行 Return。保持 `imperfect_sensors=false`、GridRoute clearance `0.40 m`、EGO inflation `0.22 m`。

## 评估器缺值修正与历史证据

修复前 `tools/summarize_provincial_stage15_terminal_v3.py` 排除了全程关键样本有限性检查。`tools/results/provincial_stage15_forward_integration_20260928/pre_fix_missing_evidence_reproduction.json` 保存了反例：在 B22 原始数据的临时副本中，将 `sim_t=5.04 s` 的 `odom_x`、`gt_speed`、`published_vx` 分别置为 NaN，旧 v3 对三种情况都给出 PASS。原始证据没有修改。

新评估器从原始 GT/odom CSV 和最终指令 JSONL 逐条核验全程有限性、单调性与 `0.2 s` 连续性。唯一豁免为第一条有效最终指令到达前连续的 `published_vx/vy/wz` 全 NaN 前缀，且须由原始最终指令接收记录证明并落在 `0.2 s` 内；GT/odom 从不豁免，后续缺值不豁免。完整的终点前 2 秒及后 5 秒继续由原始记录独立重建，不放宽原有停车、误差、漂移或几何判据。新运行器发目标前等待新鲜有效的 GT、odom 和最终速度话题。

临时副本负面测试证实：行驶中 odom/GT/最终指令 NaN、后续再次缺最终指令、终点窗口速度超限、GT/指令断流、完成事件缺失和终点快照不一致均无法 PASS。相关 27 项测试通过。历史 hold、A1/A2、B22/C29 共五条原始记录经收紧后的 v3 全部通过；B22/C29 各明确豁免一条位于首次最终指令接收前的启动样本，其他三条为零条。历史审计使用当时保存且哈希已匹配的审计文件作 capture-time attestation，同时明确列出当前工作区与历史输入哈希的差异；不声称历史记录与今天的评估器或运行器文件相同。新的审计文件名均为 `terminal_evidence_v3_strict.json`，分别位于原有 `hold_trial_01/`、`a_two_goal_regression/`、`bc90_regression/` 目录，旧审计和原始数据保留。

## 离线预检及实跑

标称路线 A `(4.7,0.5)→(4.7,1.15)`、B `(4.7,1.15)→(5.8,1.8)`、C `(5.8,1.8)→(8.7,4.25)` 和 Full `(4.7,0.5)→(8.7,4.25)` 均经 GridRoute 连通性、各参考段 `0.40 m` clearance、90° 车体静态采样间距检查。Full 内部参考转角为 `(4.7,1.8)` 和 `(8.7,1.8)`，标称长度 `7.75 m`。每次发目标前，又以实际 Gazebo GT 起点重复预检。保留 Goal Guard，没有目标被拒绝、发到墙里或绕过安全检查。

同一 Gazebo 中按 A→B→C 顺序发送，期间未 set_pose、重置、重启 Gazebo 或导航节点；前一段完整 2+5 秒停车及原始证据审计通过后才发送下一段。A/B/C 的 route ID 依次为 `R0001`、`R0002`、`R0003`，仿真时间和 GT 完成位姿连续。Full 在另一个干净的 Gazebo 中仅发送一次最终目标，由原导航接口生成并完成 `R0001:W00/W01/W02` 三个内部 waypoint；无外部中间目标。

| 试验 | 实际 GT 起点 (m) | GT/odom 到点误差 (m) | 最小采样车体—墙间距 (m) | 最终事件 ID | 仿真耗时 (s) |
| --- | --- | ---: | ---: | --- | ---: |
| A | (4.700, 0.500) | 0.01768 / 0.01768 | 0.18992 | R0001:W00 | 12.96 |
| B | (4.700, 1.132) | 0.01737 / 0.01737 | 0.13270 | R0002:W01 | 25.60 |
| C | (5.783, 1.800) | 0.01770 / 0.01770 | 0.13996 | R0003:W01 | 53.40 |
| Full | (4.700, 0.500) | 0.01782 / 0.01782 | 0.08449 | R0001:W02 | 75.36 |

A/B/C 各段纯运行时间合计 `91.96 s` 仿真时间；Full 为 `75.36 s`，两次运行使用独立 Gazebo，不能把时长差解释为严格性能提升。四段最终 yaw 均保持 90°，没有重试、卡住、采样几何重叠或 false success。沿训练场参考线投影的 GT 进度没有回退；局部计划会正常更新，但原始事件中没有路线级异常重规划。Full 的采样车体—墙间距仅比当前 `0.08 m` 判据高约 `4.5 mm`，后续更换正式场地或扰动定位时必须重新验证，不能把这次 PASS 当作安全余量充足。

## 可追溯性与判定

两次运行分别保存运行前 Git 状态、导航及配置/地图/评估器的 SHA-256、非 install 输入快照、运行后原始文件 SHA-256、Gazebo/ROS 日志、GT/odom/上游及最终指令、事件、计划、离线与每段实际起点预检。`tools/summarize_provincial_forward_integration.py` 是独立总判定入口，重新核对原始数据哈希、输入快照与当前文件、同一 Gazebo 连续性、唯一 route/完成事件及每段 v3 原始证据；总 PASS 不取自 runner 返回码或人工布尔值。独立报告为：

实跑后另行核验了 source/install：导航接口、EGO 轨迹执行器、启动文件、world、PGM、地图 YAML、EGO 配置及训练场 JSON 的对应文件内容相同；其中 install 下的配置、地图和 world 链接到当前 source。EGO 二进制的运行时哈希已在两次输入 manifest 留存。

- `tools/results/provincial_stage15_forward_integration_20260928/continuous_abc/mission_verification.json`：`all_pass=true`，一个 Gazebo，三段全部 PASS。
- `tools/results/provincial_stage15_forward_integration_20260928/single_full/mission_verification.json`：`all_pass=true`，独立 Gazebo，单目标 Full PASS。

两份审计均记录了可重复执行的汇总命令，运行所用的输入和原始数据哈希均匹配；分别记录一个 Gazebo server 及退出后的空进程列表。旧 `hold_trial_summary.json` 的未赋值 `pass=false` 保留，仍由该轮独立 `hold_validation_final.json` 及本轮严格历史审计解释；没有覆盖旧记录。两次试验均为单次尝试，没有失败后自动重跑。

**CONTACT DATA UNAVAILABLE**。上述“无碰撞”只表示采样位姿下车体与静态墙模型未几何重叠，不能替代接触传感器判定。正式省赛场地的尺寸、边界和固定障碍尚未核验；本结论不是正式赛场验收，也不授权进入 Stage 2。
