# Stage 1.5 省赛暂定通道低速验证（2026-09-27）

状态：**STAGE 1.5 LOW-SPEED PASSAGE MARGINAL**。本轮使用 2025 暂定规则推导的 **0.8 m 训练通道假设**，`final_arena_verified=false`；0.8 m 不是 PDF 单独标注的自动通道宽度。本轮没有修改 clearance、EGO inflation、地图、footprint 或导航恢复逻辑，也没有进入 Stage 2。

启动前确认运行 world 为 `provincial_2025_training`，PGM 为 `provincial_2025_provisional.pgm`，GridRoute clearance=0.40 m，EGO inflation=0.22 m，`imperfect_sensors=false`。场景无临时障碍；运行的是本仓库 `/home/rmnav/AeroMind/install`。限速通过省赛 launch 的已有参数接口覆盖：`vx_max=vy_max=0.25 m/s`，`wz_max=0.30 rad/s`，EGO 规划 `max_vel=0.25 m/s`、`max_acc=0.35 m/s²`。两轴各自限速时合成平面速度可超过 0.25 m/s；实测 GT 最大合成速度 A/B/C 分别约 0.336/0.278/0.295 m/s。EGO inflation 未调整。

| Test | Runs | Success | GT 终点误差 | 中心线 CTE P95 / max | 最小车体墙距 | 最大相对 yaw | 最大有效横向宽度 | Retry / Stuck | Contact / 几何重叠 | 判定 |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| A 起点 (4.7,0.5)→(4.7,1.15) | 1 | 到点 | 0.067 m | 0.003 / 0.004 m | 0.062 m | 89.6° | 0.668 m（相对 yaw 约 51°） | 0 / 0 | contact unavailable / 0 overlap samples | MARGINAL |
| B 转角 (4.7,1.15)→(5.8,1.8) | 1 | 到点 | 0.014 m | 0.056 / 0.099 m | 0.092 m | 90.0° | 0.520 m | 1 / 0 | contact unavailable / 0 overlap samples | MARGINAL |
| C 射击区 (5.8,1.8)→(8.7,4.25) | 1 | 到点 | 0.047 m | 0.075 / 0.118 m | 0.051 m | 90.0° | 0.520 m | 3 / 0 | contact unavailable / 0 overlap samples | MARGINAL |
| D 完整路线 | 0 | 未跑 | — | — | — | — | — | — | — | A/B/C 安全门槛未满足 |
| E 返回路线 | 0 | 未跑 | — | — | — | — | — | — | — | D 未通过准入门槛 |

到点定义为 Gazebo GT 与目标相距不超过 0.10 m、GT 速度不超过 0.10 m/s 且维持 1.5 s；A/B/C 耗时分别为 7.42/12.14/51.80 s。最终 odom 误差分别约 0.067/0.014/0.047 m。三次合计重试 4 次、stuck 0 次、目标已接受、未见 EGO `endpoint occupied`、`initial control point occupied`、`A* error` 或 `Emergency stop` 相关告警。Gazebo 未提供 contact 话题，因此**不能宣称“0 次物理碰撞”**；以 GT 位姿和 SDF 墙体盒做定向矩形几何检测，1191 个采样中无几何重叠。瞬时采样之间的接触仍不可排除。

最小墙距是根据 **MEASURED** Gazebo GT 位姿、0.52×0.42 m 定向车体盒与 SDF 墙体 collision 盒离线计算的 **INFERRED** 物理间隙。中心线 CTE 是 GT 到暂定通道中心线的距离，不等同于纯控制器误差；轨迹点误差还包含沿轨迹的时间滞后。B 最贴墙点在约 (4.785,1.684)，CTE 0.085 m，而 GT 到当时下发轨迹点仅 0.004 m。C 最贴墙点在约 (8.603,1.962)，CTE 0.097 m、轨迹点误差仅 0.008 m。因此这两处的贴墙主要与转角附近局部轨迹切角有关，不能归为控制器明显跟踪失效。A 的最小墙距发生在直道自然转向时：车体相对通道约 51°，有效横向宽度达到 0.668 m，显示目标 yaw 与矩形 footprint 在窄道中相互作用。全局 GridRoute 预检和 Goal Guard 正常通过，尚无全局规划断路证据。

三条路线合并的中心线 CTE P95 为 0.073 m、最大 0.118 m。原 0.030 m 预算**不能覆盖实际通道占用偏差**；它也不能直接用作纯控制器横向误差结论，因为局部轨迹允许偏离中心线。最小实际墙距只有 0.051 m，而暂定规则允许尺寸偏差和本届赛场未知，故 0.40 m 当前评价为 **MARGINAL**，不能据此升到正常比赛速度。下一轮应先保留现有配置并分析转角处局部轨迹与 yaw 的安全余量；不得自动将 clearance 降到 0.35 m。

逐次原始数据、ROS 事件及摘要位于 `tools/results/provincial_low_speed/`。`test_a_start.csv`、`test_b_turn.csv`、`test_c_shoot.csv` 是采样数据；对应 `*.events.jsonl` 和 `*.summary.json` 保存日志与统计，`launch.log` / `launch_restart.log` 保存启动与规划日志。`aggregate.json` 是本表的机器可读摘要。测试完毕后已正常停止 Gazebo，未留下运行中的导航进程。
