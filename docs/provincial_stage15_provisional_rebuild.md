# Stage 1.5 — 省赛暂定平面场地重建与离线验收

状态：**PROVINCIAL PROVISIONAL ARENA REBUILT**；**STAGE 1.5 OFFLINE BENCHMARKS READY**（仅限 2025 暂定图的名义几何）。**2026 最终赛场未验收；本轮没有运行 Gazebo 导航或发送目标。**

## 图纸证据与范围

2025 福建省赛暂定规则第 6、7、9、10 页直接标注：场地 9400 × 10000 mm、启动区 800 × 1000 mm、一处蓝色射击方块 800 × 800 mm、目标靶区域 2500 × 2500 mm。PDF 第 9 页没有独立指向绿色自动通道两侧边界的宽度尺寸线；图上的其他 800/600 mm 标注不可无条件挪用。当前暂定模型的 **AUTO PASSAGE WIDTH = 0.8 m** 是从 800 mm 蓝色射击区与右侧自动通道左右边界对齐这一图纸关系推得，横向连接段同宽属于连续通道建模假设，**不是“自动通道独立标注 800 mm”**。附录网盘图纸链接目前返回“链接不存在”，未能取得 CAD 验证该假设。

队长确认实际赛场没有旧暂定稿文字提到的起伏路段；本轮只做纯平面 2D 自动通道。2025 规则自身注明是暂定稿，场地允许 ±5% 尺寸误差并以赛前最新版本为准。因此 0.8 m 模型用于开发与离线训练，不能声称 2026 正式场地尺寸已确认。

## 统一场地几何

唯一输入为 [暂定场地源](/home/rmnav/AeroMind/tools/config/provincial_2025_provisional_source.json)。[生成器](/home/rmnav/AeroMind/tools/generate_provincial_provisional_arena.py) 从这份源派生：

| 资产 | 路径 | 状态 |
| --- | --- | --- |
| 导航场地 JSON | [provincial_2025_provisional.json](/home/rmnav/AeroMind/src/uav_planning/config/provincial_2025_provisional.json) | 0.8 m 平面通道，含边界段 |
| Gazebo visual + collision | [provincial_2025_training.sdf](/home/rmnav/AeroMind/src/uav_bringup/worlds/provincial_2025_training.sdf) | 同一通道矩形与边界墙；墙属暂定训练限制 |
| GridRoute ground map | [provincial_2025_provisional.pgm](/home/rmnav/AeroMind/src/uav_bringup/maps/provincial_2025_provisional.pgm) + 同名 YAML | 通道内 free，外部 occupied，0.05 m/格 |
| 独立启动配置 | [provincial_2025_provisional.launch.py](/home/rmnav/AeroMind/src/uav_bringup/launch/provincial_2025_provisional.launch.py) | 默认不自动发目标；imperfect_sensors=false |
| EGO 配置 | [ego_provincial_2025_provisional.yaml](/home/rmnav/AeroMind/src/uav_bringup/config/ego_provincial_2025_provisional.yaml) | inflation 保持 0.22 m |

旧 RMUC 全国赛 world、STL、PGM 和旧省赛训练图均未覆盖。新场地中固定墙体被建成训练用虚拟边界；旧规则图没有足够证据证明所有这些边界在实物场地中都是实体墙。机器检查确认：三个 visual 赛道块与源矩形一致，11 段 collision 边界与源一致，PGM 每一栅格与同一矩形并集一致。详见 [离线审计 JSON](/home/rmnav/AeroMind/tools/results/provincial_stage15_provisional_offline_audit.json)。

## 车体与 clearance

当前 Gazebo 底盘碰撞盒长 0.52 m、宽 0.42 m，外接圆半径 sqrt(0.26² + 0.21²) = **0.334 m**。GridRoute 当前默认 0.55 m 是车体中心到最近 occupied 栅格中心的欧氏距离门槛，近似半径 0.55 m 圆；它不读取矩形 footprint。在 0.8 m 通道中，该模型理论需要至少 1.10 m 通道，因而**不兼容**。

当机器人沿通道方向摆正，矩形单侧几何余量为 (0.8−0.42)/2 = **0.19 m**；在横向通道中车头仍朝北、全向横移时，横向占宽为 0.52 m，单侧余量 **0.14 m**。车身旋转时横向占宽为 0.52|sin(yaw)|+0.42|cos(yaw)|，极值约 0.668 m，最坏单侧余量约 **0.066 m**。离线连续中心线采样的矩形检测：固定 yaw=90° 最小实体边界余量 0.140 m，yaw=45° 为 0.068 m，均未碰撞；这不代替轨迹跟踪实测。

0.05 m 栅格的半对角误差预算为 0.035 m。外接圆 0.334 + 栅格预算 0.035 + **暂定**理想跟踪预算 0.030 ≈ **0.399 m**，离线候选取 0.40 m。该 0.030 m 尚无实跑证据，且 0.8 m 通道只允许约 0.030 m 的最大剩余预算；候选处于名义几何极限，不能称为已经验证的安全参数。几何推导范围是约 0.369–0.400 m（安全预算从 0 到约 0.030 m），不是靠跑通反向拍定数值。若 2025 暂定规则的 ±5% 宽度偏差落在较窄方向，0.40 m 圆形模型可能不再可行，需要基于真实场地与矩形姿态重新评估。

新启动配置仅给省赛场地传入 GridRoute clearance=0.40 m；GridRoute 其他场地默认值仍为 0.55 m，planner 算法未重写。导航接口中末段直线、转角切换和重试恢复原本固定采用 0.50 m 的安全检查，本轮将这三处限制为 `min(0.50 m, 地图 GridRoute clearance)`，避免新场地已经通过 0.40 m 路由却在执行检查中被固定 0.50 m 拒绝；旧 RMUC 场地仍取 0.50 m。EGO inflation=0.22 m 未作调优。它只略大于半车宽 0.21 m，却小于横移时半车长 0.26 m；局部避障保护余量应在低速通道验证中单独检查，本轮不同时改 EGO。

## 离线 benchmark 与阈值

[PROVISIONAL_STAGE15_BENCHMARKS](/home/rmnav/AeroMind/tools/config/provincial_stage15_provisional_benchmarks.yaml) 均标记 final_arena_verified=false、TRAINING-ONLY；Goal Guard 使用新 PGM 检查 start/goal safe、同连通分量及 A* 路径，静态不可达则不发布。

| Route | Start → Goal (m) | 0.40 m raw A* | raw 长度 | 路径最小占据格中心间隙 | 最大仍连通 clearance |
| --- | --- | --- | ---: | ---: | ---: |
| Start | (4.7,0.5) → (4.7,1.15) | PASS | 0.650 m | 0.400 m | 0.400 m |
| Turn | (4.7,1.15) → (5.8,1.8) | PASS | 1.627 m | 0.400 m | 0.400 m |
| Shoot | (5.8,1.8) → (8.7,4.25) | PASS | 5.233 m | 0.400 m | 0.400 m |
| Full Auto | (4.7,0.5) → (8.7,4.25) | PASS | 7.509 m | 0.400 m | 0.400 m |
| Return | (8.7,4.25) → (4.7,0.5) | PASS | 7.509 m | 0.400 m | 0.400 m |

离线 clearance sweep：0.25、0.30、0.35、0.40 m 时五路线均连通；0.45、0.50、0.55 m 时五路线均拒绝。0.25/0.30 m 小于车体外接圆半径，不能因连通就称安全。路径最小值是 PGM occupied **栅格中心**距离；与实体墙面实际空隙是不同指标。

## 实跑门槛与剩余限制

- 当前是**暂定训练场**，不是 2026 正式赛场。需待本届起点、射击区、通道宽度、固定墙与障碍资料到手，快速复核源参数并重生成 visual/collision/PGM；不继续维护全国赛中央高地为省赛验收条件。
- **可在下一轮启动低速通道验证；尚不具备宣称完整 Stage 1.5 压力测试通过的条件。** 低速验证应先检查车体碰撞、轨迹横向误差、转角 yaw、通道刮墙以及 0.22 m 局部膨胀的实际效果。本轮未运行机器人。
- 当前 Gazebo 是单碰撞盒、无真实车轮接地模型，只能做平面软件与碰撞近似验证。
- 现有扫描过滤规则为旧宽阔场地设置；在 0.8 m 通道内，临时障碍可能与静态墙被归在一起。动态障碍压力测试不能凭本轮静态离线通过就宣称已解决。
- 仍禁止 Stage 2 的 noisy odometry、MID-360、IMU、LIO、真实定位改造。下一轮若做实跑，先验证上述低速门槛，并保持 Goal Guard 生效。
