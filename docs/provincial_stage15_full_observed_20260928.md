# Stage 1.5 原生采样 Full：单次试验报告（2026-09-28）

**本轮未达到正向可复核基线验收条件。** 保持 `TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`、`CONTACT DATA UNAVAILABLE`。没有修改导航、planner、控制、场地、物理步长、传感器频率或验收阈值；没有重跑 Full 或执行其他路线。

## 执行边界与输入

首先核对了旧 `continuous_abc/mission_verification_v4.json` 和 `single_full/mission_verification_v4.json`，两份当时报告均为通过。对旧原始数据重新执行 `--verify-only` 时，每段任务与 waypoint 证据仍通过，但总汇总中的“**当前工作区文件仍与旧运行时相同**”因本轮记录器变化而失败；不能把这个当前文件哈希差异解释成历史导航失败，也不能把旧报告当作当前输入证明。

第一轮 Gazebo **只进行了启动预检**，保存于 `tools/results/provincial_stage15_full_observed_20260928/`。参数留档过早读取 ROS 节点列表，仅见 `/gazebo_bridge`，故在发送目标前停止。`goals_requested=0`，保留该目录，不计为 Full 导航试验。

第二轮在 `tools/results/provincial_stage15_full_observed_20260928_preflight2/` 进行了唯一一次 Full。启动前冻结 37 个输入的实际路径、符号链接解析目标、SHA-256 与内容副本，另存 Git HEAD、dirty diff、状态、完整启动命令、runner 命令、ROS/Gazebo 版本。启动后、发送目标前保存四个核心节点的实时参数 dump 及哈希。运行前后 37 个输入哈希一致，运行参数的五个文件哈希复核一致。实时参数确认 `imperfect_sensors=false`、`grid_route_clearance=0.4`、`grid_map/obstacles_inflation=0.22`。ROS 为 Humble，Gazebo Sim 为 6.18.0。源代码和 install 中使用的软链接目标均在清单中。

标称路线 `(4.7,0.5) → (8.7,4.25)` 的参考段长 7.75 m，GridRoute 所需 clearance 为 0.40 m；按 90° 车体静态采样的最小墙距 0.14 m。真实起点 `(4.7,0.5,90°)` 再次通过预检；未调用 set_pose、reset，外部仅请求一个目标。记录到 `/goal_pose` 的一条消息：`(8.7,4.25,90°)`。该订阅证据证明目标出现在 ROS 总线上，**不证明导航节点的回调已处理该目标**；没有独立监控所有 reset/set_pose 服务调用。

输入留档仍有一个明确缺口：新独立评估器直接导入的 `tools/analyze_provincial_forward_clearance.py` 未列入运行前 37 项快照。事后可以核对其**当前** SHA-256 为 `4c7690fac5021f3d771c269fa64b04dc66048a7fa5678f7e3d54e40f3d6cd1b9`，但不能将其称为运行前留档。该模块仅提供离线 SDF 墙体解析，不参与导航；评估器结果保留这一追溯限制，不能宣称所有直接依赖均已封存。

## 运行与独立审计

本次 Full 的 runner 返回码为 2，结果为 `timeout`。240.0 仿真秒里 GT 保持 `(4.7,0.5)`，最终 GT 与 odom 到点误差均为 **5.482928 m**；最终 yaw 约 90°。没有 route 接受、内部 waypoint 发送/完成、局部计划或 `/cmd_vel` 记录，也没有完整 2+5 秒终点窗口。独立汇总为 **FAIL**。虽然启动器正常收尾并退出 0，这只代表启动器进程结束，不能代表导航通过；报告以 runner 返回码、原始事件和独立审计为准。

逐消息文件 `trial_full_observed_01.native.jsonl` 为约 40 MB，记录了所有本记录器实际收到的 `/clock`、Gazebo GT、ground odom、原始/最终速度、目标、诊断及可用轨迹消息。未对 GT 做 0.05 s 抽样；保留每话题接收序号、单调接收时间、接收时仿真时间、可用消息时间戳及原始字段。Twist 无消息自身时间戳，使用接收时间并标为 `none_receive_time_only`。在任务窗口收到 **12,000 条 GT**，平均 **50 Hz**，最大相邻仿真时间间隔 **0.0200000000 s**；未见关键话题的非有限字段、接收序号错误或时间戳乱序。`/clock` 共 60,029 条。接收序号不是发布端序号，因此无法据此证明传输层绝无丢包；实际间隔和缺失话题已在审计中列出。

本次采样最小车体—墙间距 **0.190000 m**，在 `sim_t=9.26 s`、GT `(4.7,0.5)`，最近为 `boundary_4`。**没有采样值低于 0.08 m**，但机器人没有进入旧 Full 的 `boundary_7` 窄处，所以不能用本次结果复核旧最小值 0.0844864394 m，也不能说通道安全已被重复证明。旧 Full 的 75.36 仿真秒、GT 到点误差 0.017819 m、采样最小间距 0.084486 m 仍只是原历史结果。单个新样本没有统计重复性意义；更密集采样本身也不构成连续时间间距下界。无接触传感器数据。

## 失败分类与下一步

本次失败发生在**目标总线可见之后、导航 route 接受事件之前**，归类为目标接收/任务分发未确认，尚不能归因于 planner、控制或 `boundary_7` 贴墙。运行期间导航节点仍在发布 ground odom、执行最终零速度输出，没有 route 接受或拒绝日志。无法仅凭现有数据区分 `/goal_pose` 在发送时尚未与导航订阅端匹配、回调排队/饥饿或其他未记录的分发问题。新增 `/goal_pose` 只读订阅让原 `get_subscription_count() > 0` 的就绪检查可以被**记录器自身**满足；这是需要优先核查的具体缺口，但不是已证实的丢目标原因。

最小下一步是先只改运行器的就绪/确认机制：发送前明确验证导航节点的订阅端已匹配，发送后以限定时间等待本 route 的接受或拒绝事件，未确认则保存现场并停止；同时独立捕获目标消息与导航接收时序。完成隔离测试后，须另获新一轮单次 Full 授权，才能重新验证 `boundary_7`。本轮不修改导航或参数，也不再启动 Gazebo。

## 证据与复核

- 运行输入：`tools/results/provincial_stage15_full_observed_20260928_preflight2/input_manifest.json`、`input_snapshot/`、`runtime_parameters/`、`runtime_parameter_sha256.json`、`input_sha256_after.json`。
- 原始记录：同目录的 `trial_full_observed_01.native.jsonl`、`.csv`、`.events.jsonl`、`.plans.jsonl`、`.actuation.jsonl`、`.published_commands.jsonl`、`.summary.json`、`launch.log`、`progress.json` 和 `raw_data_sha256.json`。
- 独立报告：`mission_verification_observed.json`，已列入清单的 37 项输入快照及 9 项已关闭原始数据文件哈希均匹配；另有上述一项离线评估器直接依赖未在运行前封存。可重复审计时指定新的 `--output-name`，不能覆盖当前报告。
- 测试：新追踪器隔离序列化测试确认 `/clock`、Odometry、无时间戳 Twist 及 NaN 原样记录；现有历史审计入口仍能运行。新 Full 的失败路径亦被独立审计判为 FAIL。
