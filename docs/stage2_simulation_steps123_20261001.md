# Stage 2 仿真步骤 1–3（2026-10-01）

**本轮指定步骤 1–3 已完成；带误差导航并非全部通过。已在第 3 步结束。**

状态：TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED。
未接 MID-360、IMU、LIO 或实机。本轮是人为里程计故障注入，不是硬件定位验收。

## 文件整理与基线保留

- 增加 `docs/PROJECT_INDEX.md`，把当前入口、封存配置、历史交接和 Stage 2 工具明确列出。
- 新工具集中于 `tools/stage2/`；本轮结果独立保存在 `tools/results/stage2_steps123_20261001/`。
- 删除源代码与工具目录中 3 个可再生成的 `__pycache__`，共 487,152 字节。ROS 运行可能重新生成缓存，收尾再清理并单独记账。
- 没有删除历史报告、CSV/JSONL、快照、失败现场、旧版本评估器、第三方许可或源码。
- 没有删除 `build` / `install`：安装符号链接可能依赖其中内容，编译缓存也用于后续开发。
- Stage 1.5 的 92 项封存输入全部未变；原完整独立汇总再次通过；194 个已登记历史证据文件哈希再次核验通过。
- 未改 Stage 1.5 启动入口、world/PGM/collision、导航算法、planner 参数、最终速度守卫。

## 步骤 1：独立 Stage 2 入口

新增 `src/uav_bringup/launch/provincial_stage2_odometry.launch.py`。
沿用原平面场地、二维激光雷达、规划和控制配置，只增加独立里程计节点及输入适配器。
`auto_goal=false`，默认零误差。GUI/RViz 可通过已有开关显示。

固定参数：imperfect_sensors=false、GridRoute clearance=0.40 m、EGO inflation=0.22 m、车体 0.52×0.42 m、平移速度上限 0.25 m/s、角速度上限 0.30 rad/s、原 0.08 m 采样间距门槛及完整 2+5 秒终点条件。

之前的 imperfect_sensors 实现没有删除，也没有打开。其旧开关会同时引入里程计误差和底盘执行动态，本轮独立节点只注入里程计误差。

## 步骤 2：真值与导航输入分离

```text
Gazebo /gazebo/odometry
        ├── Stage 2 模拟里程计 ── /simulation/navigation_odometry
        │                           └── Stage2NavigationInterface
        │                                 └── /ground/odometry → 原导航链路
        └── 只读记录 / 独立评估
```

`stage2_navigation_interface.py` 继承原导航接口，仅覆盖输入合法性检查。通过 remapping 将继承的真值订阅改为独立导航里程计。若没有正确 remap，或打开旧 imperfect_sensors，节点拒绝启动。
运行前端点身份稳定至少 1 秒：导航接口不订阅真值，只订阅 `/simulation/navigation_odometry`；该话题唯一发布者为模拟里程计节点。端点 GID/QoS 和图快照有留档。

独立话题采用 odom/base_link、消息原始时间戳及机体系 twist。原内部 `/ground/odometry` 的世界系速度兼容约定保留，原控制输出未改。此阶段 map→odom 仍为恒等、初始出生位置已知，没有全局重定位或误差纠正功能。

非零误差模式初始化一次位置，之后累计运动增量，不随每条真值重置估计。零误差模式严格复制原始位姿与速度，便于与原基线比较。
非有限值、错误四元数、过期/未来数据、重复时间戳会拒绝；源时间倒退或超过 0.2 s 的消息时间间隔锁定故障，不能自动跳回真值。原传感器/轨迹 watchdog 及停车逻辑保留。

## 步骤 3：独立累计误差试验

每组一个独立 Gazebo、一个正向 Full 外部目标，未发中间外部目标、未 reset/set_pose，未重跑误差组。
起点约 (4.7,0.5)，目标 (8.7,4.25)，yaw=90°；由导航自行切换三段内部 waypoint。
各次输入内容、符号链接目标、实际 ROS 参数、启动命令、Git 状态/差异、系统版本及原始数据 SHA-256 均留档。系统依赖为版本/路径留档，不是完整系统镜像。

| 组别 | 人工误差参数 | 导航结果 | GT 到点误差 | odom 到点误差 | 原生 GT 最小采样间距 | 任务仿真耗时 |
|---|---|---|---:|---:|---:|---:|
| zero | 无误差 | PASS，完整 2+5 秒通过 | 0.017660 m | 0.017660 m | 0.084080 m | 75.18 s |
| scale | 机体系 x×1.005、y×0.995 | PASS，完整 2+5 秒通过 | 0.041273 m | 0.017307 m | 0.090521 m | 75.46 s |
| yaw_bias | +0.0005 rad/s | FAIL：真值间距低于 0.08 m，终止 | 未到点 | 未到点 | 0.079963 m | 42.70 s |
| random_walk | 位置 0.002 m/√s，航向 0.0003 rad/√s | FAIL：真值间距低于 0.08 m，终止 | 未到点 | 未到点 | 0.079986 m | 20.82 s |

所有非零参数均为训练假设，不是 MID-360、IMU 或轮式里程计的实测参数。
随机种子 20261001；误差从节点初始化时开始累积，发目标时不重置。各组初始化到发目标的时间不同，不能把相同 seed 当作行驶期间完全相同的噪声时序，也不能用单次试验推导统计可靠性。

### 零误差与比例组

零误差原生配对 3,758 个样本，与真值位姿/速度完全一致，全部 waypoint 和 CSV/native 的完整停车窗口通过。
比例组累计定位偏差最大 0.027367 m；真实到点误差与里程计报告误差已明显分离。
两组 retry/stuck/false success 均为 0。

### 航向偏置组失败现场

- sim_t=52.36 s，GT≈(8.282738,1.744337)，yaw≈1.549686 rad。
- 最近墙 boundary_6，间距 0.079962519 m，触发试验终止。
- 最近已记录的模型数据 sim_t=52.34 s：估计≈(8.271362,1.810447)，位置偏差 0.066871 m、航向偏差 0.026160 rad≈1.50°。
- 最近已记录的原导航诊断仍为 allowed，预测间距约 0.129375 m、计划间距约 0.128886 m。
- 估计位置上的守卫没有给出物理真值间距保证。人为航向偏置使位置增量方向长期偏转，同时控制按估计航向调节机器人实际方向。

### 随机游走组失败现场

- sim_t=37.72 s，GT≈(5.633153,1.859742)，最近墙 boundary_7。
- 原生最小间距 0.079985744 m，触发试验终止。
- 最近模型记录 sim_t=37.70 s，估计≈(5.632751,1.851398)。此时位置偏差约 0.008803 m，航向偏差约 0.002828 rad；全程位置偏差最大 0.015555 m。
- 最近原导航诊断为 allowed，预测间距约 0.088273 m、计划间距约 0.085237 m。
- 本场地原间距余量较小，厘米量级定位扰动就能消耗余量。这里没有验证真实硬件误差大小或 LIO 能否纠正误差。

两组失败均属于本次给定误差条件下的定位偏差/真值间距失效，不是消息断流、记录器失效或已证明的接触碰撞。
上面的诊断按最近一次消息时间关联，不是严格同步的生成时刻比较。
终点及完整 2+5 秒窗口未执行；未完成的后段不能判 PASS。试验截至终止前 retry/stuck/false success 为 0，不证明继续运行不会失败。

真值间距门槛仅作为试验终止条件：记录器不把真值送回导航、不纠正估计、不向速度话题发布指令。终止由运行器关闭本次 Gazebo 完成，不是新增可部署的真值安全守卫。终止响应时间未单独验收。

## 独立审计与反例

19 项模型/实际输入接口/独立递推审计工具测试通过。原生每条收到的消息保留，未抽样、补零或插值。各组原生消息、接收连续性、writer 健康及输入/原始文件哈希均通过。
独立审计从真值与模拟里程计同时间戳配对，重算每一步运动比例、航向偏置及固定种子的随机游走递推。记录器发现之前的前缀只按原始诊断计数跳过 RNG 调用，不伪造前缀原始数据；因此不宣称已录制节点完整生命期。

Stage 1.5 的原完成事件审计要求日志位置近似等于 GT。加入误差后，该日志位置实际上来自导航 odom，比例组旧 v1 因此被判证据不一致。
新增 Stage 2 完成事件审计：日志位置与原生 `/ground/odometry` 配对；GT 单独核验。日志位置一致性 0.02 m、完成事件距离 0.05 m、最终 GT 误差 0.10 m、停车速度/漂移/连续性、完整 2+5 秒条件全部保留；native 最终完成时 GT≤0.05 m 的原严格条件也保留。
原 v1 报告保留，新 v2/v3 与总汇总重新复核；没有覆盖或重跑原记录。

在临时副本上核验七个案例：合法比例原记录通过；删除首个完成、完成乱序、前 2 秒 GT 速度超限、后 5 秒非零最终输出、GT 终点位置偏移、停车窗缺失 GT 段均被拒绝。

两次启动预检失败也保留于 `attempts/`：首次新脚本执行权限不足；第二次 ROS 图端点身份尚未完成发现。均未发目标。已修补运行前执行权限检查与稳定端点等待。不是失败后重复导航目标寻找一次成功。

## 修改文件与运行影响

- 新增运行文件：`stage2_odometry_model.py`、`stage2_simulated_odometry.py`、`stage2_navigation_interface.py`、独立 launch 及四份 profile YAML。
- 修改 `src/uav_planning/CMakeLists.txt`：只增加三个新文件的安装声明，没有删除现有程序。
- 新增 `tools/stage2/` 的运行器、审计、反例、汇总、绘图与测试。
- 新增本报告及项目索引。
- 非零模式会改变 Stage 2 导航位置与速度估计，因此会间接改变导航输出。原 planner/executor/最终速度输出实现及参数未改；旧 Stage 1.5 入口完全不使用新节点。

可复核命令（须 source ROS 与 install）：

```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/summarize_steps123.py
```

手动打开独立零误差入口（不会自动发目标）：

```bash
ros2 launch uav_bringup provincial_stage2_odometry.launch.py gui:=true rviz:=true auto_goal:=false
```

原运行器拒绝覆盖已有组目录。当前四组已完成，不应直接重跑同名命令。

## 证据与下一步边界

- 总审计：`tools/results/stage2_steps123_20261001/steps123_verification_v2.json`；v1 保留。
- 分组目录：`zero/`、`scale/`、`yaw_bias/`、`random_walk/`，各有输入快照、参数、CSV/native JSONL、events/plans/actuation/final commands、health/timing、原始 SHA-256。
- 新版 zero/scale 审计：各组 `independent_audit_v3.json`；失败两组 `independent_audit_v1.json`。
- 反例：`terminal_counterexamples_v1.json`；失败关联数据：两组 `failure_context.json`。
- 图：`stage2_steps123_native_samples.png`。均为原始采样；曲线连线不构成新增原始数据或连续时间安全证明。

**请求的实现与有界实验范围完成，不代表三组误差导航均通过。** 本轮停在第 3 步，不调导航、不降低间距门槛、不扩大误差矩阵，不接 MID-360/IMU/LIO。
后续应以这两个失败现场明确定位误差预算与几何余量问题，再另行决定接入传感器定位链路的范围；不能仅通过把 0.08 m 判据调小来消除 FAIL。

CONTACT DATA UNAVAILABLE。未记录接触传感器，采样几何未重叠不能写成独立接触检测通过；不宣称连续时间安全、统计重复性或正式省赛场地验收。
