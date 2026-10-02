# Stage 1.5 单次 Full 与原生 GT 复核（2026-09-29）

**本轮指定执行与审计均已完成；验收结论为 FAIL。** 机器人实际完成了正向 Full 的全部 waypoint 并到点，最小采样间距也高于 0.08 m；但整段任务的接收连续性证据出现一次超过现有 0.2 s 阈值的中断。保持 `TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`、`CONTACT DATA UNAVAILABLE`。没有第二次 Gazebo、第二个外部目标或其他测试路线。

## 运行与输入

唯一运行目录为 `tools/results/provincial_stage15_full_delivery_20260929/`，trial ID 为 `trial_full_delivery_01`。单次启动器记录一个 Gazebo server、一个目标请求，退出后无残留 server；runner 返回码 2，启动器正确向外传播失败。源代码中未调用 set_pose/reset；本次未独立监控全部相关服务调用，因此“无重置”限于运行器代码和记录到的证据。

目标发送前留档 42 项递归本地依赖的内容、路径、解析后的符号链接目标与 SHA-256，包括导航基类、scan matcher、planner 二进制、地图/场地和评估器。保存 Git HEAD、dirty diff、启动命令、ROS/Gazebo 版本与四个节点实际参数。运行前后文件内容及链接目标均一致，原始数据关闭后的 9 项文件哈希匹配。系统级 ROS/Gazebo/Python 库只记录已取得的版本，并非完整系统镜像。

实际起点 GT 为 `(4.7,0.5,90°)`。离线参考路线 7.75 m，GridRoute clearance 0.40 m，yaw=90° 时参考线采样最小车体—墙距约 0.14 m。运行参数为 `imperfect_sensors=false`、GridRoute clearance 0.40 m、EGO inflation 0.22 m；未修改导航算法、控制、速度限制、geometry、PGM、collision、物理步长、传感器频率或验收阈值。

## 导航结果与证据失败

原生记录中仅有一个外部 `/goal_pose`：`(8.7,4.25,90°)`。发送前记录到唯一的生产导航订阅端、实际 DDS QoS 匹配和持续约 0.5 s 的稳定端点。发送后约 **0.0054 s** 收到生产导航 logger 的匹配 route 接受事件。`R0001:W00 → W01 → W02` 三个 waypoint 按序完成，没有重试、卡住、静态重叠或 false success。

最终 waypoint 约在 `sim_t=75.92 s` 完成。最终 GT 和 odom 到点误差均为 **0.017582 m**，实际 yaw 90°。原始数据从 `sim_t=75.94` 至 `82.94 s` 可重建完整的 **2+5 秒停车窗口**；该窗口本身的 GT、最终输出、漂移和误差检查通过。导航在物理意义上到达并稳定停车。

但原生逐消息数据表明：在 `sim_t=9.00 → 9.02 s` 两条相邻 GT 之间，记录器的**单调接收时间**相隔 **0.343483 s**，超过原有的 0.2 s 全程连续性阈值。相同时间附近，ground odom、最终速度和 `/clock` 在这个记录器中的最大接收间隔分别约 **0.361 / 0.341 / 0.336 s**。GT 的消息仿真时间戳仍以 0.02 s 递增，未见仿真时间跳跃、GT 数据乱序或非有限值。现有证据只能确认记录器观察到了共同的接收停顿，不能区分发布端停顿、ROS 调度或单线程记录器处理阻塞。

终点 gate 因这次缺口保留 `GT reception gap` 无效标记，结果为 `evidence_gap`；完整窗口可单独重建，不会抹掉全程缺口。独立报告中 14 项顶层检查有 3 项失败：runner 成功退出、导航的 `terminal_pass` 状态和完整任务终点审计。全程 GT/最终指令连续性检查也失败。**因此不升级为暂定训练场可复核正向基线。**

## 原生 GT 与窄处

任务窗口收到 **3,791 条原生 GT**，平均 50 Hz，最大相邻**仿真时间**间隔 0.020 s；全文件共 3,826 条 GT。所有原生采样重算的最小车体—墙距为 **0.0843167037 m**，位于 `sim_t=27.94 s`、GT `(5.457234,1.855683)`、yaw 90°，最近墙体为 `boundary_7`。所有已采样点均高于 0.08 m，最窄余量约 **4.317 mm**。没有接触传感器数据，也不能从离散采样直接推出连续时间间距下界。

旧 Full 的采样最小值为 0.0844864394 m，新值低约 **0.170 mm**。新局部计划在相同 x 的 y 约为 1.855495 m；新 GT 向 `y=1.8 m` 参考线偏约 0.055683 m，setpoint y 约 1.855492 m。新旧两次都呈现计划靠近 `boundary_7`、GT 基本跟随计划的形状；只有一次新运行，不能据此声明统计重复性或找到造成计划偏移的算法原因。图中最终速度取机体系平移速度模，不把机体坐标误读为世界方向。

局部俯视图和时间曲线见 `tools/results/provincial_stage15_full_delivery_20260929/offline_clearance_review_v1/`。脚本 `tools/analyze_provincial_full_delivery.py` 是**运行后的只读分析工具**，未被称作运行前输入快照的一部分；分析报告另存原始数据、旧对照数据、当次 SDF 和分析脚本的哈希。

## 最小下一步

先只读排查 `sim_t≈9 s` 的共同接收停顿，重点对照记录器单线程回调耗时、ROS 进程调度与 WSL/Gazebo 宿主负载。现有数据没有发布端时间戳或调度追踪来唯一归因。若确认记录器自身造成阻塞，应仅最小修改采集/评估方式，并证明不放宽现有 0.2 s 阈值、2+5 秒及 0.08 m 判据；若真实传输发生中断，应按证据缺口处理。任何修复后新的 Full 都需另设独立试验，不在本轮自动重跑。

核心证据：`tools/results/provincial_stage15_full_delivery_20260929/mission_verification_delivery_v1.json`、`trial_full_delivery_01.native.jsonl`、`trial_full_delivery_01.summary.json`、`raw_data_sha256.json`、`input_manifest.json`、`input_sha256_after.json`、`runtime_parameters/`、`offline_clearance_review_v1/offline_review.json`。历史失败和成功记录均未覆盖。
