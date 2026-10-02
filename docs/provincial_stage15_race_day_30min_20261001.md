# 省赛赛前 30 分钟地图处理流程（2026-10-01）

队伍确认正式地图只会在赛前约 30 分钟提供，格式暂时未知。因此 Stage 1.5 的开发验收以暂定训练场和快速建模工具为范围，不能把提前得到正式地图作为开发结束条件。正式地图验收留在当天执行；TRAINING-ONLY / PROVISIONAL 状态继续保持。

## 当前工具能做什么

`tools/prepare_provincial_arena.py` 接收人工确认过的米制 JSON，输出独立目录内的 field.json、arena.sdf、arena.pgm、arena.yaml、arena_preview.png、arena.launch.py 和 offline_audit.json。它没有向 ROS 发布数据，没有安装或覆盖冻结训练图。启动文件默认 auto_goal=false，已有 Goal Guard 保留。

当前支持平面 XY、若干轴对齐可通矩形的并集、任意方向的线段墙盒；可用封闭线段表达矩形固定障碍。当前沿用训练参数：地图 0.05 m/格、墙厚 0.055 m、GridRoute 0.40 m、车体 0.52×0.42 m、EGO inflation 0.22 m。它不是任意 CAD/STEP 或坡道导入器，也没有把边界颜色自动判定为实体墙。不同墙厚、复杂多边形、坡道、缺少尺寸时应明确拒绝当前模型，人工确认/提前扩展导入器后再用；不能猜测尺寸、缩小 clearance 或把通道拓宽。

## 30 分钟时间分配（工作预算，非实测保证）

| 时间 | 任务与产物 |
| --- | --- |
| 0–5 分钟 | 两人核对资料版本、单位、坐标原点/方向、起点、射击区、通道宽度和固定障碍；确认实体墙与禁行边界语义。缺关键尺寸立即向裁判确认。 |
| 5–12 分钟 | 录入米制 JSON；一人录入，一人对图纸复核。保存原始资料和来源记录。|
| 12–15 分钟 | 生成独立候选 bundle，检查 preview、离线 connectivity、0.40 m route、固定姿态矩形余量、SDF/PGM 一致性。|
| 15–23 分钟 | 仅在前述检查通过且人工复核后，启动候选仿真，短段/转角/全程有序验证，保存失败；此时间不保证足够完成全部 2+5 秒审计。|
| 23–27 分钟 | 核对真实机器人部署的坐标、外形、限速、传感器/电控接口及实际场地；这些是后续实机阶段的责任，当前 Stage 1.5 没有实现。|
| 27–30 分钟 | 保存最终配置/哈希和可回退候选，停止临时调参，确认操作者动作。|

30 分钟可能不足以处理不完整图纸、复杂模型、实机故障或跑完验证。这是流程预算，不是已证明的端到端用时。机器计时单独记录，人工读图/录入目前未做盲测；不要将已有训练 JSON 的生成时间当作赛前总时间。

## 三种资料格式的处理

- 图片/PDF：先找明确尺寸线。至少保留 x/y 两方向的可靠尺寸或比例，检查透视/缩放；直接标注优先，人工量图结果须记录不确定性。自动通道宽度、实体墙不能根据旧图默认补齐。
- CAD/STEP：保留原文件、单位和坐标轴；使用现有 CAD 工具读取平面尺寸，再录入 JSON。当前脚本没有 CAD 自动导入能力，不能宣称已支持 STEP 一键转换。
- 照片/现场实测：斜视照片不能直接当正交地图。测主要边界、通道宽度、转角与固定障碍，用共同原点统一坐标；缺少测量时不启动自动导航。

## 操作命令

先复制模板到新的任务文件，替换全部坐标/区域与来源记录。模板内的旧训练值只能用于演练，不是当天真实场地的默认值；未知字段填 null 并列入 unresolved_dimensions，工具会拒绝。

```bash
cd /home/rmnav/AeroMind
cp tools/config/provincial_arena_input_template.json tools/config/race_day_candidate.json
# 编辑 race_day_candidate.json，人工复核单位、边界语义和尺寸。
python3 -B tools/prepare_provincial_arena.py \
  --source tools/config/race_day_candidate.json \
  --output-dir tools/results/race_day_candidate_01 \
  --competition-input
```

只有 scale/axes/start/shooting_zone/free_space/fixed_walls/fixed_obstacles 七项由操作者实际核对后，才可设置 confirmations=true。该标志只是输入确认记录，不会令 competition_arena_verified=true，也不替代独立场地/实机验收。

输出目录存在时拒绝覆盖。若生成审计失败，保留候选和失败原因，禁止启动。若通过，先人工检查 preview 和原图一致；再 source ROS/install，用新目录里的明确启动文件：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch /home/rmnav/AeroMind/tools/results/race_day_candidate_01/arena.launch.py \
  gui:=true rviz:=true auto_goal:=false
```

此处只启动显示，不自动发目标。目标下发仍须实际当前位姿的 GridRoute 与矩形检查；不应把旧 Full 的目标硬编码用于新场地。`run_provincial_low_speed_validation.py` 和本轮往返运行器仍是旧训练 benchmark 工具，不能直接套用新场地验收。

## 冻结与部署边界

准备工具已将硬编码楼板尺寸和出生点的旧生成器输出，仅在新候选目录内替换为输入值；机器人、物理步长和传感器频率不变。新候选 launch 直接引用新资产，不需要重新编译导航节点。静态输入复核通过不意味着任何新形状的轨迹都可通过，尤其更窄通道与改变航向必须重验。

当前没有 MID-360/IMU/LIO、真实定位或电控接入；本流程不能代替这些工作。本轮不自动进入 Stage 2。CONTACT DATA UNAVAILABLE，采样间距也不是连续时间安全证明。
