# Stage2 激光输入保护与窄通道间距复核 — 2026-10-01

本轮指令已完成：修复前反例留档、输入门控最小修复、36项隔离测试、历史三个Full的独立审计、窄点前后各2秒分析和新版本归档。
本轮 Gazebo 启动次数=0，导航目标发布次数=0。

仍为 TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED。
本轮不表示 Stage2 完成，也没有接入 MID-360、IMU 或 LIO。

## 修复结果

实际执行修复前生产 scan_callback 的隔离反例：
- angle_increment=NaN：matcher 已拒绝，但基类仍发布含360个非有限点的点云，并刷新scan_received_at。
- frame_id=camera：旧入口仍接受匹配、更新校正并发布点云。
- angle_max 与采样数量不一致：旧入口仍接受。

新入口在基类处理前检查坐标系、角度与采样数量、有限距离上下界、时间元数据及时间戳范围。无效输入不能更新点云、地图点、动态障碍或传感器新鲜度。

明确区分拒绝行为：
1. 硬格式或坐标系错误：设置输入故障，复用生产watchdog，在下一次最终输出update中三轴停车；只有新鲜合法且拟合通过的扫描才能清除此故障。
2. 陈旧、未来、重复/乱序、关联失败、内点不足或退化匹配：拒绝并不刷新有效扫描或匹配时间；原已接受定位最多保持现有0.5秒门槛，过期后停车。仿真时间和单调接收时间都检查，因此暂停/卡住的仿真时钟不能无限保持运动。
3. 正常无返回的inf或单束无效NaN：丢弃该束，不补零。若剩余有效证据足够，可以正常匹配；全部无返回时拒绝。
4. 恢复运动仍需原有轨迹及安全条件成立，不能靠恢复一条扫描绕过缺失/不安全计划的守卫。

没有放宽原有0.5秒、25ms关联、0.08m间距等门槛；没有修改planner、executor、速度、车体、地图或场地。

## 雷达坐标系与安装约定

从历史原始扫描确认 frame_id=omni_robot/base_link/lidar。
配置SDF里雷达相对base_link为(0,0,0.25,0,0,0)，平面共位且同向。
新入口启动时读取已安装的配置SDF，保存路径/解析目标和SHA-256；不支持的安装偏移、倾角、frame或topic拒绝启动，不能默认为零外参。
新增参数 localization_scan_frame_id 和 localization_world_sdf 的默认值对应当前隔离启动入口；并没有修改launch/world。
该检查证明配置文件约定，不是独立查询运行中Gazebo模型的完整证明。

当前只支持time_increment=0的瞬时仿真扫描，不支持滚动采集而未去畸变的扫描。LaserScan的scan_time表示两次扫描的时间间隔，不能冒充采集时长；依据[ROS2官方消息定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/LaserScan.msg)。未来真实雷达需要重新处理时间和外参。

## 验证与历史证据

36项工具/隔离测试通过。实际生产callback及update主体被执行，外部发布者仅为内存对象，没有向ROS导航链路注入速度。覆盖硬错误停车、有限/无返回束、时间异常、不刷新拒绝数据、定位超时、恢复后仍须安全计划、单调时间超时、SDF外参变化拒绝。

六种临时副本反例均不能PASS：错误frame、矛盾angle_max、NaN angle_increment、NaN range_min、删除原始扫描、篡改修正量。更新临时副本的数据哈希后仍拒绝，不是仅由哈希差异判失败。

零误差、yaw_bias和random_walk历史Full经v4/v5审计仍PASS。分别检查了761/749/761条被历史定位采用的扫描的静态元数据、frame及场地安装约定。

这些结果只证明旧数据与原任务通过，以及旧输入满足新静态门槛；不能证明历史仿真运行过本轮新门控，不能替代真实扫描断流/恢复试验。
历史runtime/current差异报告如实包含两份导航源及其安装链接；zero还保留上一轮审计修正产生的工具差异。旧运行快照和before/after哈希是旧运行的证据，不伪称当前文件与旧运行完全相同。

## 最窄点同时间戳分析

所有最窄点均为boundary_7。墙面靠通道侧y=2.2m，车体足迹朝墙方向支撑长度为0.26|sin(yaw)|+0.21|cos(yaw)|。
同一GT消息时间戳下，将校正后估计误差分解为：

```
估计间距 - 真值间距 = -(朝墙位置误差 + yaw导致的车体支撑长度误差)
```

本地4秒区间中足迹均远离墙端，因此该平面分解成立，最大代数残差约1e-15m。每个区间201个原始GT样本、间隔0.02仿真秒；没有插值生成GT。
正朝墙误差表示估计更靠上墙；负间距高估值表示估计较保守。

| 场景 | 最窄sim_t(s) | GT间距(mm) | 朝墙位置误差(mm) | yaw足迹误差(mm) | 估计间距-真值(mm) | 4秒内最大间距高估(mm) |
|---|---:|---:|---:|---:|---:|---:|
| 零误差 | 33.28 | 82.657 | +0.520 | +0.036 | -0.556 | 1.205 |
| 航向偏置 | 33.38 | 83.886 | +0.443 | +0.140 | -0.583 | 0.316 |
| 随机游走 | 34.58 | 84.394 | +0.047 | +0.016 | -0.063 | 2.771 |

最窄时三个估计都略保守约0.06–0.58mm；因此不能把全程最大定位误差直接从另一时刻的最小间距中相减，声称这里已违反门槛。
但邻近窗口仍有最高约2.77mm的间距高估。GT最小间距比门槛多约2.66–4.39mm，这个量级仍不足以宣称对定位误差稳健。

统一CSV保存GT、原始注入位姿、校正位姿、法向误差、yaw足迹影响、原始/最终机体系速度及换算的世界系速度、setpoint、最近收到的计划点、轨迹ID、active waypoint和守卫诊断。
没有自身时间戳的Twist/诊断采用前一个接收记录，附接收年龄；它们不是同一生成时刻。最近计划点只作空间参考，不能当成控制沿样条推进的状态。守卫预测与实际采样间距亦不同。

没有可靠连续时间下界；不把采样正间距或诊断预测写成连续安全或接触检测通过。

## 修改和归档

本轮修改三个已有文件：
- src/uav_planning/scripts/stage2_localized_interface.py：拒绝入口与输入故障门控，会影响独立Stage2模式是否停车/恢复。
- src/uav_planning/scripts/stage2_wall_localization.py：元数据及SDF约定验证；有效输入的拟合公式/阈值没有改变。
- tools/stage2/test_localization.py：测试输入与新增状态适配。

新增六个工具：scan_guard_fixture.py、reproduce_scan_guard.py、test_scan_guard.py、audit_scan_guard.py、analyze_scan_guard_clearance.py、check_scan_guard_counterexamples.py。

source/install内容一致，原安装文件都是指向源文件的链接，不需要改CMake或重新构建。
Stage1.5封存92项输入仍全部匹配，其完成汇总只读复核再次通过。上一轮全部封存证据/原始数据保留；旧源封存的三项差异是上述本轮有意改动，不能说它们仍匹配旧哈希。
保留所有旧dirty修改，未清理、回滚或覆盖历史工作。

本轮路径：/home/rmnav/AeroMind/tools/results/stage2_scan_guard_20261001/
- verification_v1.json：本轮汇总。
- counterexample_before.json / counterexample_after.json：生产回调修复前后。
- tests_final_v3.log：36项通过；较早测试失败日志保留（恢复测试时钟未推进到扫描时间，修正测试时序后通过，没有放宽生产判据）。
- <profile>_historical_reaudit_v4.json / <profile>_contract_audit_v5.json：新审计。
- scan_guard_evidence_counterexamples_v1.json：临时副本反例。
- <profile>_narrow_timeline_v2.csv / narrow_clearance_review_v2.json / .png：局部原始时间轴和图。
- changes.patch、input_before/、stage15_reverification.log、final_evidence_seal.json：增量修改与封存。

复核入口使用新的不存在的输出路径：

```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/audit_scan_guard.py \
  --result-dir tools/results/stage2_localization_20261001/yaw_bias \
  --output /tmp/stage2_scan_guard_reaudit_unique.json
```

## 下一步

需要一次受控、短距离的实际Gazebo扫描中断/恢复验证，确认新故障停车与原任务恢复，之后才扩大Stage2融合传感器或误差组合测试。
本轮没有新live结果；不调整clearance/EGO或拓宽地图，不用GT作为定位反馈，不接入真实传感器。

CONTACT DATA UNAVAILABLE；不证明统计重复性或连续时间安全；不代表正式省赛场地验收。
