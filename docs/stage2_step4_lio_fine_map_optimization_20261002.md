# Stage 2 第 4 项：LIO 体素分辨率优化核验（2026-10-02）

本轮完成，结论为 `STAGE 2 STEP 4 LIO FINE MAP OPTIMIZATION VERIFIED`。

保持：`TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`、`CONTACT DATA UNAVAILABLE`。没有进入真实硬件第 5 项。

## 实际修改及采用依据

新增可选入口 `provincial_stage2_lio_fine_map.launch.py`，只将 LIO 参数 `filter_size_map` 从 0.10 m 改为 0.05 m。该参数同时控制 LIO 的输入特征体素滤波和内部地图采样；全局导航 PGM、场地 JSON、collision、world 均不变。沿用上一轮已经修复的瞬时点云库，其二进制没有再次修改。

保持全 3D、固定外参、不估计时间偏移以及原模拟噪声方差；没有采用之前被拒绝的 planar_mode 或降低观测方差。GridRoute clearance=0.40 m、EGO inflation=0.22 m、0.52×0.42 m 车体、0.25 m/s 速度限制、planner、controller、最终输出守卫和所有终点/间距阈值不变；imperfect_sensors=false。

此修改影响 LIO 估计输出，间接影响闭环运动；导航控制算法本身未改。

预先保存 `experiment_plan.json`：在两份不同历史 Full 的完整原始传感器数据上，各运行一次 10 cm 对照和一次 5 cm 候选。要求两份数据均最大 XY 误差改善至少 10%，RMS 和最后误差不恶化，原始输入/库身份/记录健康/逐帧点集合检查通过，之后才允许一次 Full 闭环。

四次回放顺序执行，DDS domain=73，不输入 GT、导航目标或速度。GT 仅供事后误差对照，采用最近原始 GT 时间戳且相差≤25 ms，不插值。实际 ROS 参数 dump、加载的 .so 路径及 SHA-256、相同传感器 CDR 哈希均独立核验。

## 两份固定输入对照

以下单位均为 cm：

| 数据 | LIO 体素 | 最大 XY 误差 | XY RMS | 最后 XY 误差 |
| --- | --- | ---: | ---: | ---: |
| 记录1（旧 Full） | 10 cm | 5.5855 | 2.7595 | 5.0097 |
| 记录1（旧 Full） | 5 cm | 2.3914 | 0.4068 | 0.5533 |
| 记录2（上轮 Full） | 10 cm | 4.7485 | 2.4887 | 4.7484 |
| 记录2（上轮 Full） | 5 cm | 2.9709 | 0.5396 | 0.4966 |

两份数据的最大误差分别改善约 57.2% 和 37.4%；RMS 分别改善约 85.3% 和 78.3%。最后误差均约 5 mm。四次回放全部已记录输出帧通过刚性点集合检查（1004/1004/1094/1094 帧），没有用缺失输出、插值或补零制造通过。

这些结果证明该单一参数变化在本轮两份数据中有效，支持采用新仿真配置。更细体素改变平面拟合的采样和地图保留，可能减轻粗体素的配准偏差；本轮未逐一审计所有邻域和平面残差，不能断言累计误差的唯一根因。回放回调时序仍可有差异，不是逐位重复，也不是统计重复性或真机精度保证。

## 唯一新增闭环 Full

只启动一个独立 Gazebo，只发送一次 `(8.7,4.25)` 外部目标，起点约 `(4.7,0.5)`、yaw=90°，无外部中间目标。实际参数比较只有 `filter_size_map:0.1→0.05`；launch 文本比较也只有此处覆盖。沿用原 LIO、物理 PositionImu、等待停车及恢复逻辑。

| 项目 | 本次结果 |
| --- | --- |
| 独立审计 | 36/36 项通过 |
| 接受确认延迟 | 7.949 ms |
| 内部航点 | R0001:W00、W01、W02 顺序及坐标完整 |
| 完成事件 GT 误差 | 0.030956066 m |
| 最终 GT 到点误差 | 0.030585985 m |
| 最终 odom 到点误差 | 0.014071636 m |
| 实际终点 yaw | 1.566037824 rad，目标 π/2 |
| 完整 2＋5 秒停车窗口 | [94.68, 96.68]＋[96.68, 101.68] s，通过 |
| 完整窗口原生 GT / 最终指令样本 | 351 / 348 |
| 最大停车 GT 速度 | 0.017575285 m/s，阈值 0.02 |
| 停车窗口最大最终平移/角速度 | 0 / 0 |
| 停车漂移 | 0.000069447 m |
| 含 7 秒观察的总仿真耗时 | 76.46 s |
| retry / stuck / false success | 0 / 0 / 未观察到 |
| 任务期间 LIO 对最近原始 GT 最大 XY 偏差 | 0.025384173 m |
| 原生 GT 最小采样矩形车体—墙距 | 0.128285365 m |
| 最窄位置 | sim=43.60 s，(4.890550,1.787940)，boundary_6 |
| 原生 GT 频率 / 最大时间戳间隔 | 50 Hz / 0.020 s |
| 任务期 GT 最大接收间隔 | 0.024633272 s |
| 任务期最终指令最大接收间隔 | 0.070464284 s |
| 低于 0.08 m 的采样数 / 几何重叠采样数 | 0 / 0 |
| 本次逐帧点集合核验 | 1014 个已记录输出帧全部通过 |

对比上一轮单次 Full：GT 到点误差 4.52→3.06 cm，最大 LIO 偏差 4.75→2.54 cm，最小采样墙距 12.36→12.83 cm，含观察耗时 87.58→76.46 s。两次闭环的实际运动、轨迹和回调时序不同，这组比较不能证明统计改善或全部差异均由体素参数造成；采用依据主要是两份固定输入对照及本次闭环完整验收。

## 连续性、工具与追溯

本次 277 项目标前输入快照包含旧库、新入口、运行器、评估器、局部依赖、地图及运行版本/命令；实际加载库身份、参数、输入运行后哈希与原始数据哈希通过。系统依赖不是完整机器镜像，未全量监控所有 reset/set_pose 服务调用。

没有新放宽评估器。沿用上一轮固定 10..15 s 停车重力核验和全程动态 specific force 核验；相关评估器本次在目标前已封存。静止 IMU 均值约 9.810018 m/s²，原 0.2 m/s² 容差保持，逐样本和静止/连续性证据通过。运动期模长均值 10.120240 仍原样保留，不能当成重力大小。GT 和传感器的任务接收连续性、写入计数、健康状态均通过；没有从序号推断发布端绝无丢包。

本轮新增 8 项门槛及参数序列化测试通过，覆盖微小改善不能接受、RMS/最后误差恶化、证据缺失、NaN、字段缺失和 ROS 嵌套/扁平参数转换。最初读嵌套 YAML 导致的错误审计保留为 `old_full_voxel_010_analysis.json`，正确复核为 `_v2.json`；相对路径工具异常保留在 `gate_path_failure.log`。两者发生在候选采纳/仿真之前，没有覆盖原证据或重复仿真找成功。

GNU time 的整个回放任务及子进程统计：第二份记录 10/5 cm 的总 CPU 时间约 96.44/96.67 s，峰值 RSS 约 190.3/191.6 MiB。本轮没有观察到明显资源放大，但它们包含 Python、记录器和相关子进程，不是独立 LIO 的 CPU/RAM 成本。文件缓存/调度影响仍存在，不能宣称提速，不能外推真实 MID-360 更高点数的性能。

## 新入口、文件和封存

后续仿真可显式选择已验证的新入口：

```bash
source /opt/ros/humble/setup.bash
source /home/rmnav/AeroMind/install/setup.bash
ros2 launch uav_bringup provincial_stage2_lio_fine_map.launch.py navigation:=true gui:=true rviz:=true auto_goal:=false
```

本轮已结束 Gazebo，此命令只作为使用说明。旧 safe/point_boundary 入口及其 10 cm 配置保留，不替换原基线。

- 新源入口：`/home/rmnav/AeroMind/src/uav_bringup/launch/provincial_stage2_lio_fine_map.launch.py`。
- 新安装入口：`/home/rmnav/AeroMind/install/uav_bringup/share/uav_bringup/launch/provincial_stage2_lio_fine_map.launch.py`，符号链接到新源文件。
- 本轮工具：`/home/rmnav/AeroMind/tools/stage2/lio_voxel_review/`。
- 所有新结果：`/home/rmnav/AeroMind/tools/results/stage2_step4_lio_voxel_review_20261002/`。
- 预设门槛和对照汇总：该目录 `experiment_plan.json`、`offline_candidate_gate.json`。
- 新 Full 原始 CSV/JSONL、输入快照、参数及哈希：`full_fine_map_01/`。
- 正式独立审计及逐帧核验：`full_fine_map_01/independent_audit_v1.json`、`point_set_audit_v1.json`。
- 图及分析：`voxel_replay_comparison.png`、`fine_map_full_verified.png`、`fine_map_full_geometry_review.json`、`replay_resource_review.json`。
- 新汇总/封存：`optimization_completion_audit.json`、`final_evidence_seal.json`。

复核新封存不会启动仿真或下发目标：

```bash
cd /home/rmnav/AeroMind
python3 -B tools/stage2/lio_voxel_review/finalize_voxel_review.py --verify-existing
```

上一轮 317 个源文件和 2184 个证据文件保持不变，原 tracked dirty diff 保留。本轮只新增入口、工具及证据，未清理其他工作或覆盖历史失败。

仍未证明：连续时间最小间距、独立接触检测、统计重复性、真实 MID-360 扫描时序和硬件适配、正式省赛场地正确性。当前样本最大偏差低于 4 cm 工程余量，并不把该余量升级为经过标定的误差上界。终点仍存在控制残差和定位偏差，尚未达到零误差；本轮没有改变终点阈值或控制参数掩盖它们。

当前建议采用新可选仿真入口继续项目，保留所有适用范围和旧基线。没有进入第 5 项。
