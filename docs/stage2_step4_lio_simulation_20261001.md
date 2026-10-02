# Stage 2 第 4 项：三维雷达、IMU、LIO 仿真接入与验证

更新：2026-10-01 23:41（Asia/Shanghai）。

**本轮接入和测试已执行；第 4 项的严格闭环验收尚未全部完成。**

状态：`TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`、`CONTACT DATA UNAVAILABLE`。

原步骤 4 是“接入三维点云、IMU 和 LIO，先验证定位，再闭环导航”。本轮新增独立入口，复用仓库现有 SPARK FAST-LIO 二进制。默认 Stage 1.5 和已封存的步骤 1–3 没有替换、删除或回滚。实物接入不在本轮范围。

## 已实现的链路

```text
Gazebo 三维 lidar → /simulation/lidar3d/points
  → lio_sensor_adapter → /lio/lidar ┐
Gazebo 物理 IMU → /simulation/imu3d │
  → lio_sensor_adapter → /lio/imu   ├→ 原 SPARK FAST-LIO
                                  └→ /lio/odometry + /lio/cloud_registered
  → 固定出生位姿对齐、IMU/车体安装偏移转换、因果速度估计
  → /localization/lio_navigation_odometry
  → LIO 专用导航入口 → 原 planner / executor / 最终安全守卫
```

导航、LIO、两个适配器均不订阅 `/gazebo/odometry`。GT 由记录器只读订阅，用于审计和结束不安全试验；它不修正 LIO、轨迹或导航速度。2D `/scan` 保留为障碍传感器，未在这条链路做 2D 位姿匹配。步骤 3 的人工误差里程计保留在其独立入口，本入口没有叠加它。

保持 `imperfect_sensors=false`、GridRoute 0.40 m、EGO inflation 0.22 m、原车体 0.52×0.42 m、速度/加速度/完成/停车/间距阈值。没有修改原 planner、executor、导航最终速度实现或场地几何。新 SDF 去掉新增传感器与插件后，与冻结世界的 XML 语义完全一致，包括机器人、碰撞和物理步长。

## 最终实际结果

| 检查 | 结果 | 证据 |
|---|---|---|
| 最终代码静止定位（short_05 下目标前，固定 sim 10–15 s） | PASS | 1251 对原始位姿；最大平面误差 0.00185913 m；最大估计速度 0.00816963 m/s |
| 短目标 `(4.7,0.5)→(4.7,1.15)` | PASS | 最新独立审计 20 个检查通过；GT 误差 0.03815313 m；odom 误差 0.03327924 m |
| 短目标完整停车 | PASS | 原始 CSV 和逐消息记录分别核验完整 2+5 秒 |
| 一次 Full `(4.7,0.5)→(8.7,4.25)` | **严格审计 FAIL** | 一个 Gazebo、一个外部目标，内部 W00/W01/W02 顺序完整；总 75.92 仿真秒 |
| Full 完成事件 | **FAIL** | sim 83.74 s，GT 到点误差 **0.05795730 m > 0.05 m** |
| Full 最终停车快照 | 记录保留 | GT 误差 0.05792150 m；odom 误差 0.00391004 m |
| Full 停车速度、漂移和完整 2+5 秒覆盖 | PASS | sim 83.76–90.76 s；351 个 GT、350 个最终指令；最终输出为零，漂移为零 |
| Full 原生采样最小车体—墙间距 | PASS（仅采样几何） | **0.09593036 m**，sim 34.64 s，GT `(5.322340,1.840848)`，yaw 1.555308 rad，boundary_7 |
| Full 全程来源、消息、输入和数据哈希 | PASS | LIO 位姿转换和速度独立重算；IMU 与物理运动测量配对；输入和原始哈希匹配 |
| 重试 / 卡住 | 均为 0 | short_05、full_01 |

Full 运行器退出码为 0、summary 为 `terminal_pass`、assessment 为 `MARGINAL`，**都不能替代独立任务审计**。停车观察的目标误差阈值 0.10 m 与完成事件的 GT 阈值 0.05 m 不同；本次确实满足停车窗口，但不满足完成事件。因此不得归档为新的合格 Full 基线，也不得宣布步骤 4 全部验收通过。

原生 GT 50 Hz，最大消息时间间隔 0.020 s；全文件最大单调接收间隔 0.041008 s。对齐后 LIO 250 Hz，IMU 250 Hz，三维 cloud 约 10 Hz。Full 任务期原始 IMU 最大接收间隔 0.009332 s，三维 cloud 0.108192 s，已注册 cloud 0.111602 s。全文件启动前缀有约 0.499377 s 的 IMU 接收间断，报告保留，**并未宣称全文件连续性通过**。任务期边界由唯一外部目标与最终快照原始记录确定，输入发送前已有新鲜、稳定定位。

Full 收到原始 IMU 22688 条、转发 IMU 22688 条、原始 cloud 908 条、转发 cloud 908 条、注册 cloud 905 条。健康文件的接收/写入计数与行数核验通过。接收序号连续不证明发布端绝无丢包。

## Full 失败的已证实部分

使用最近原始 GT（时间差不超过 25 ms，不插值）配对：

| 内部完成点 | sim s | LIO−GT 平面偏差 |
|---|---:|---:|
| W00 `(4.7,1.8)` | 29.58 | 0.01090067 m |
| W01 `(8.7,1.8)` | 61.452 | 0.04626223 m |
| W02 最终目标 | 83.74 | 0.05205372 m |

全程最大平面偏差为 **0.08132413 m**。最终完成附近的 LIO−GT 约为 `(+0.029534,+0.042864) m`，与导航认为已到点、实际仍偏离目标的现象一致。独立代数复核通过，没有发现固定出生位姿或 0.25 m 安装偏移转换造成的错误，也没有发现本次原始来源匹配缺失。

LIO 估计高度也出现变化，提示需要检查三维定位的高度/姿态可观测性和 IMU/点云处理。**当前不能把稀疏点云、低矮场地、瞬时点时间或高度漂移中的任何一项宣布为唯一根因。** GT odometry 插件是 2D，z 为 0；图中的 LIO z 与声明物理车体高度的差值不是独立录制的 3D GT 误差。

本次 boundary_7 的采样间距大于历史 2D 定位 Full 的 0.0844864394 m / 0.0843167037 m，但定位方案和试验不同，不能据此断言安全性能提高或获得统计重复性结论。图使用原始位姿，不把插值当成新增样本。没有连续时间间距下界或接触传感器证据。

## 本轮修复与保留的失败

1. 初版用相邻 4 ms LIO 位姿差分估速，静止时产生约 0.354 m/s 的虚假速度。新适配器改为保留真实时间的 0.20 s 因果最小二乘速度窗口，至少 0.10 s 有效数据后才输出，不补零、不削峰、不伪造时间。
2. 当前 VelocityControl 场景中，原生 IMU 横向加速度近似仅噪声，无法反映加减速。新增 **物理层** IMU 插件根据实际安装点世界速度按原 4 ms 物理步长求导，再由 Gazebo ImuSensor 生成重力和声明噪声；不读 ROS GT，不驱动机器人。其可观测诊断与原始 IMU 在独立审计中配对通过。速度指令阶跃产生的加速度可达约 22 m/s²，属于理想驱动模型限制，并非真实底盘标定。
3. 两个工具启动/动态库链接错误修正；第一次 CLI 参数读取超时改为批量 ROS 参数服务读取；所有这些启动失败没有发布目标。
4. short_04 到点和停车有效，但记录器漏收一个任务期原始 IMU，严格审计保留 FAIL。只增大记录器订阅缓存，未改传感器频率，short_05 的来源匹配通过。
5. full_01 没有因为失败自动重发目标、重启 Gazebo、调整阈值或重跑。

历史目录 `stationary_01/02/03`、`short_01/02/03/04/05` 和 `full_01` 均保留。早期静止原生 IMU 结果不是最终 IMU 插件的运行证明；最终静止核验引用 short_05 目标前固定 10–15 s。Full 的 10–15 s 静止复核因目标已在该窗口末端前发送而标记不合格，不用它替代静止证据。

## 工具与针对性测试

12 个传感器/时戳/速度契约测试和 6 个直接覆盖生产 update() 的停车安全测试通过。后者覆盖：仅 IMU 预测新鲜但点云修正过期、LIO odom 过期、安全轨迹未就绪、恢复、安全轨迹就绪和指令过期。

另有 4 个一次性证据反例测试通过。在临时副本中删除原始 LIO、删除完成事件、修改停车 GT 速度为 0.1 m/s、把行驶导航位置改为 NaN；**更新副本哈希后**独立审计仍拒绝全部反例。原始证据哈希保持不变。

主证据根目录：`/home/rmnav/AeroMind/tools/results/stage2_step4_lio_20261001/`。

- `short_05/independent_audit_v3.json`：最终短路线审计。
- `full_01/independent_audit_v1.json`：Full 严格 FAIL，原始输入和逐消息数据在同目录。
- `short_review_v1/review.json`：最终代码目标前静止检查。
- `full_review_v2/review.json`、`lio_forward_review.png`：Full 定位偏差与最窄处图。
- `mutation_tests_v1.json`：四个更新哈希后的反例。
- `full_nominal_preflight.json`：场地语义一致性和 7.75 m 路线预检。
- `final_review.json`、`final_evidence_seal.json`：本轮最终状态与封存。

可重复只读审计（输出必须为新文件，已有文件不覆盖）：

```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/lio/audit_navigation.py \
  --result-dir tools/results/stage2_step4_lio_20261001/full_01 \
  --output /tmp/stage2_lio_full_reaudit.json
```

退出码 2 是本次 Full 的预期严格失败结果。不要再启动 Gazebo来重现同一报告。

## 代码与追溯

新增生产包 `/home/rmnav/AeroMind/src/stage2_lio_sim/`：物理 IMU、传感器格式/时戳契约、传感器适配、LIO 里程计适配和可选导航入口。新增 bringup 的 `provincial_stage2_lio.launch.py`、`provincial_stage2_lio.sdf`、`spark_provincial_sim.yaml`。新增工具位于 `tools/stage2/lio/`。

新导航入口会通过 LIO 位姿与新鲜度影响新入口中的最终运动输出；原输出逻辑没有修改。IMU 修复和里程计适配会影响新入口的定位输入。原默认 launch、原 SDF/PGM/collision、planner 和已封存的步骤 1–3 保持不变。

Full 发送目标前封存 **169 项本地输入**：实际执行文件、源文件、符号链接目标、Python 递归导入、原 SPARK 源码和二进制、插件、YAML、launch/world/map、运行器和当时评估器。保存实际 ROS 参数和参数门槛、LIO/IMU 动态库 ldd、版本、启动命令、Git HEAD 和 dirty diff。结束后输入与原始数据哈希复查通过。

`review_evidence.py`、`test_evidence.py` 及本轮报告是运行后新增的离线工具/文档，由最终封存记录当前版本，不能冒充 Full 运行前输入。169 项留档也不是完整系统镜像或全部运行时动态依赖封存。

结束时复核：Stage 1.5 的 **92 项**输入未变，步骤 1–3 的 **28 项**源代码及 **532 项**历史证据未变。没有清理其他 dirty 修改或历史结果。

## 仿真模型边界与下一步

三维扫描采用规则 360×16 射线、10 Hz、每帧瞬时采样；MID360 点结构仅做接口兼容，line/intensity 为明确的合成字段。所有点时间来自真实 cloud header，没有伪造 100 ms 扫描时差。SPARK 持续打印点时间跨度 0 的警告；代码中初始平均扫描时长为 0，本次真实即时帧的结束时间为 header 时间，未通过假时间消除警告。**这没有验证真实 Livox 扫描模式、运动畸变补偿或点密度。** 官方实物规格参考：[Livox MID-360](https://www.livoxtech.com/mid-360/specs)。

使用固定、声明的出生位姿对齐 LIO 局部坐标，尚未验证任意出生位姿、全局重定位或与现场地图匹配。速度窗口和协方差未做实物统计标定；协方差不能用作可信误差上界。只有一个 Full 样本，没有统计可靠性结论。

**最小下一步是先用本次封存点云/IMU做离线重放，检查 LIO 的时间配对、三维约束与高度/姿态漂移，再基于复现证据确定传感器或 LIO 的最小修复。** 不修改省赛通道、GridRoute、EGO 或终点阈值，不把 GT 接回定位链路。未修复且未取得新严格证据前，维持 `LIO INTEGRATION READY / FULL ACCEPTANCE NOT PASSED`。不进入实物步骤 5。
