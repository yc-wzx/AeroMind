# Stage 2 第 4 项：瞬时点云处理优化与独立核验（2026-10-02）

本轮完成。状态为 `STAGE 2 STEP 4 INSTANTANEOUS CLOUD PROCESSING FIX VERIFIED`。完成的是当前仿真测量处理的正确性修复、隔离回放和一次独立 Full 验证。没有进入真实硬件第 5 项。

继续标记：`TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`、`CONTACT DATA UNAVAILABLE`。

## 实际采用的修改

当前 Gazebo 3D 雷达为同一时刻的一组射线，所有逐点时间为零。原 FAST-LIO 逆向去畸变循环只退出内层循环，首点会在更早 IMU 区间再次处理；同时，在 IMU 区间边界，整帧瞬时点云也可能被执行不应有的运动补偿。

对同一份封存数据逐帧核验：原算法 1004 个已记录输出帧中，829 帧恰有一个不一致点，另有 44.3 s 的一帧整帧 1885 点不一致。仅修首点后仍保留该整帧问题。完整修复后，1004 帧全部通过 20 μm 诊断容差。该容差用于点集合刚性一致性，完全不同于导航间距阈值。

生产副本只有 `include/imu_processing.hpp` 内容改变：首点处理结束后退出函数；完成 IMU 传播和最后雷达时间维护后，若所有点时间都为零，则跳过去畸变。保留 IMU 传播、3D 滤波、地图匹配、原观测方差和真实非零逐点时间的原补偿分支。没有改传感器数据或生成虚假逐点时间。真实扫描时序分支尚未完成硬件验证。

原第三方源文件、原安装库和旧导航入口均保留。新增可选入口：

- `/home/rmnav/AeroMind/src/stage2_lio_sim/scripts/lio_point_boundary_mapping.py`
- `/home/rmnav/AeroMind/src/uav_bringup/launch/provincial_stage2_lio_point_boundary.launch.py`
- `/home/rmnav/AeroMind/install/stage2_lio_sim/lib/lio_point_boundary/`
- 修复源副本及准确补丁：`/home/rmnav/AeroMind/tools/results/stage2_step4_lio_optimization_20261002/instant_cloud_source/`

入口名称保留 `point_boundary`，实际安装的是包含上述两项修复的 `instant_cloud_build`。运行时 `/proc` 映射证明实际加载组件 SHA-256 为 `1489cdb0e10c9cc5f87cab9a5018cfa776ec977c91b1ecb7a2916b1a68b015ed`，不是原库或仅修首点的实验库。

导航算法、最终速度输出、安全守卫、GridRoute 0.40 m、EGO inflation 0.22 m、车体 0.52×0.42 m、速度限制、场地和物理步长均未修改；`imperfect_sensors=false`。修复会改变 LIO 测量处理和估计输出，因此经过了一次真实闭环 Full 验证。

## 没有采用的优化方向

同一份原始 IMU/点云分别在隔离 DDS domain 73 回放，未输入 GT、导航目标或速度；GT 只在事后对照误差。回放时序可能不同，不能当成逐位相同的重复运行。

| 候选 | 最大 XY 误差 m | XY RMS m | 最后 XY 误差 m | 决定 |
| --- | ---: | ---: | ---: | --- |
| 原算法诊断回放 | 0.054053 | 0.027248 | 0.049423 | 对照 |
| 点观测方差 0.001→0.0001 | 0.068411 | 0.032982 | 0.058149 | 拒绝采用 |
| 仅修首点重复处理 | 0.057111 | 0.028946 | 0.051481 | 不作为完整修复 |
| 现有 planar_mode 投影 | 84.261183 | 34.532643 | 84.261183 | 拒绝采用 |
| 首点修复＋瞬时点云免补偿 | 0.055855 | 0.027593 | 0.050097 | 采用其测量正确性修复 |

降低点观测方差并未改善结果，未部署该参数。现有 planar_mode 发散至约 84.26 m，未接入导航；其代码对状态作平面投影但没有同步约束协方差是值得调查的机制，不能据此宣称已证明唯一根因。保留全 3D 模式。完整点云修复的回放误差仍约 5.59 cm，所以不能声称点云缺陷是全部累计误差的根因。

## 唯一新增 Full 的结果

独立 Gazebo 只启动一次，唯一外部目标 `(8.7,4.25)`，起点 `(4.7,0.5)`，yaw=90°。接受确认约 9.05 ms。内部 `R0001:W00/W01/W02` 的计划坐标、归属、顺序及完成事件均通过审计。

| 指标 | 本次结果 |
| --- | --- |
| 独立审计 | 34/34 项通过，`independent_audit_v2.json` |
| 完成事件 GT 误差 | 0.045184523 m |
| 最终 GT 到点误差 | 0.045157707 m |
| 最终 odom 到点误差 | 0.009461159 m |
| 实际终点 yaw | 1.581999888 rad，目标 π/2 |
| 完整停车窗口 | [102.7, 104.7]＋[104.7, 109.7] s，通过 |
| 7 秒 GT / 最终指令样本 | 351 / 350 |
| 停车窗口最大 GT 平移速度 | 0.014040136 m/s |
| 停车窗口最大最终平移/角速度 | 0 / 0 |
| 停车漂移 | 0.000054637 m |
| 含停车观察的总仿真耗时 | 87.58 s |
| retry / stuck / false success | 0 / 0 / 未观察到 |
| 所有原生 GT 最小采样车体—墙距 | 0.123611415 m |
| 最窄位置 | sim=78.32 s，(8.511503,1.817503)，boundary_7 |
| GT 原生频率 / 最大时间戳间隔 | 50.00 Hz / 0.020000 s |
| GT 任务期最大单调接收间隔 | 0.023958271 s |
| 低于 0.08 m 的采样数 | 0 |
| LIO 对最近原始 GT 的最大 XY 偏差 | 0.047486655 m，关联差≤25 ms，无插值 |
| 本次点云输出刚性一致性 | 1094 帧、2190055 点全部通过；最大残差约 0.523 μm |

本次 1097 个输入帧中，初始化期 0.008/0.1/0.2 s 未有对应匹配输出，明确列出，没有补造。其余所有已记录输出均匹配原始输入及同时间戳的实际 LIO 输出位姿，核验未使用 GT。

与旧 Full 相比，最终误差 4.71→4.52 cm、采样最小间距 12.267→12.361 cm，任务期最大 LIO 偏差 5.49→4.75 cm；含观察耗时 76.50→87.58 s。新旧闭环运动和回调时序不同，每种仅一例，不能归因或宣称统计精度、速度、重复性提升。

## 独立评估器修正，保留首次 FAIL

本次 `independent_audit_v1.json` 保留 `all_pass=false`：旧评估器把包含运动过程的 IMU specific-force 模长均值 10.015458 m/s² 与重力 9.81 比较，略超原 0.2 容差。运动时 specific force 包含动态加速度，其模长均值不等于重力，这不是可靠的运动期重力判据。

新增只读 v2 使用固定 10..15 s、目标前的停车窗口。必须同时证明原始 GT 在出生位置静止、物理速度和加速度静止、三路记录连续且有限；每个静止 IMU 样本都必须满足原 0.2 m/s² 容差，同时保持原有全程动态 IMU 与物理运动/声明噪声的逐样本核验。本次 251 GT、1251 物理样本及 1251 IMU 样本通过，静止均值 9.809990186，范围 9.806754439..9.813296043 m/s²。

原旧评估器、旧 FAIL、全程均值和原始数据不变；没有放宽重力或导航阈值。v2 评估器是在运行结束后新增的，哈希、命令和源码由新封存记录，不能伪称已包含在目标下发前的 260 项运行时快照内。

20 项针对性测试通过：参数实际执行检查 5 项、点集合证据反例 6 项、静止重力证据反例 9 项。其中点证据覆盖异常点、NaN、缺输入/输出/位姿；重力证据覆盖去掉重力、单点异常、NaN、缺段/边界、GT 或物理运动及窗口内下发目标。隔离测试没有向导航注入速度。早期工具测试失败、预检拒绝和日志中的 ResourceWarning 均保留，不伪称所有开发尝试首次通过。

## 留档、入口和限制

全部新结果：`/home/rmnav/AeroMind/tools/results/stage2_step4_lio_optimization_20261002/`。

- 本次原始数据及目标、内部事件、参数、输入和数据哈希：该目录 `full_instant_cloud_01/`。
- 正式新独立审计：`full_instant_cloud_01/independent_audit_v2.json`。
- 点云逐帧审计：`full_instant_cloud_01/point_set_audit_v2.json`。
- 旧报告/原始数据未覆盖；新回放分析为 `*_analysis_final.json`，瞬时点云分析为 `instant_cloud_replay_analysis.json`，均留存评估器源码和哈希。
- 全原生 GT 几何分析及图：`instant_cloud_full_geometry_review.json`、`instant_cloud_full_verified.png`。
- 本轮工具：`/home/rmnav/AeroMind/tools/stage2/lio_optimization/`。
- 总汇总及新封存：`optimization_completion_audit.json`、`final_evidence_seal.json`。

旧基线的 233 个源文件和 1338 个证据文件哈希已再次核验未变。现有 tracked dirty diff 未改变，未清理其他工作。仿真进程已正常收尾，无残留 Gazebo；导航节点在统一 SIGINT 关闭时日志的退出码 -2 不作为运行期任务失败。

复核封存（不会启动仿真或发送目标）：

```bash
cd /home/rmnav/AeroMind
python3 -B tools/stage2/lio_optimization/finalize_optimization.py --verify-existing
```

新入口已经安装，可显式选择；原 safe 入口保留。后续需要界面时可使用：

```bash
source /opt/ros/humble/setup.bash
source /home/rmnav/AeroMind/install/setup.bash
ros2 launch uav_bringup provincial_stage2_lio_point_boundary.launch.py navigation:=true gui:=true rviz:=true auto_goal:=false
```

该命令只写入说明，本轮结束未再次启动或发送目标。重新编译安装可用 `tools/stage2/lio_optimization/install_point_boundary_profile.py`，不同现有候选库会被拒绝覆盖。

仍存在的边界：4 cm 定位余量是工程预算，本次最大偏差约 4.75 cm，尚不是经标定的误差上界；实际物理速度离散差分有加速度尖峰，仿真运动/噪声模型不能替代真机标定。采样几何不等于连续时间安全或接触传感器证据。服务调用没有全量独立监控，“无重置”只在运行器、代码/日志和连续记录的观测范围内成立。原生序号不证明发布端绝无丢包，系统库没有封存为完整机器镜像。本次传感器的全记录接收连续性也通过，但 /clock 启动期仍有约 0.445 s 接收间断，任务期正常；不能抹掉初始化证据。

本轮结论：可采用新入口的瞬时点云处理修复；保留其他候选的拒绝结论。没有正式省赛场地验收，没有真实 MID-360 时序/真机精度结论，没有进入第 5 项。
