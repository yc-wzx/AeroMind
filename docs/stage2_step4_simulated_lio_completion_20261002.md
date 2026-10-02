# Stage 2 第 4 项完成简报：仿真三维雷达、IMU、LIO 闭环

归档日期：2026-10-02，Asia/Shanghai。完成范围为当前暂定训练场的仿真定位及闭环导航，不包含真实传感器、真实比赛场地或第 5 项。

**STAGE 2 STEP 4 SIMULATED LIO CLOSED-LOOP VALIDATION COMPLETE**

状态继续保留：`TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`、`CONTACT DATA UNAVAILABLE`。

## 最终结果

最终版本完成一个独立 Gazebo 单目标 Full，随后完成另一个独立 Gazebo 的短目标回归。每个试验只发送一次外部目标，没有外发中间目标；导航自行切换内部 waypoint。两次原始证据各通过 **31 项顶层独立检查**，不是只依赖运行器退出码。完整 2 秒停车及后 5 秒观察同时由 CSV 和原生消息复核。

| 最终版本试验 | Full：`full_reserved_map_01` | 短距离：`short_reserved_map_01` |
|---|---:|---:|
| 目标，yaw 均为 90° | (8.7, 4.25) | (4.7, 1.15) |
| 实际初始 GT | (4.7, 0.5) | (4.7, 0.5) |
| 独立审计 | PASS，31/31 | PASS，31/31 |
| 最终 GT 到点误差 | 0.0470888069 m | 0.0107609021 m |
| 最终 odom 到点误差 | 0.0095688782 m | 0.0139136492 m |
| 所有原生 GT 的最小车体—墙采样间距 | **0.1226708279 m** | **0.1859401637 m** |
| 仿真耗时，包含停车观察 | 76.50 s | 13.08 s |
| 重试 / 卡住 | 0 / 0 | 0 / 0 |
| 内部完成事件 | R0001:W00 → W01 → W02 | R0001:W00 |
| 完整停车窗口，sim s | 93.82–95.82–100.82 | 27.08–29.08–34.08 |

Full 完成事件时 GT 误差约 0.0471283 m，满足原有 0.05 m 完成阈值，但仅有约 **2.87 mm** 余量。终点停车窗口误差阈值仍为 0.10 m，未混淆或放宽两个阈值。

最终 Full 原生 GT 为 50 Hz，最大消息时间间隔 0.020 s，任务期最大单调接收间隔 0.024205849 s；最终速度和 odom 的最大任务期接收间隔分别为 0.064513751 s、0.066149062 s，满足既有 0.2/0.5 s 判据。最小间距在 sim 69.44 s，GT `(8.5095517511, 1.8183214363)`、yaw `1.5755521173 rad`，最近墙为 `boundary_7`。低于 0.08 m 的原生 GT 样本数为零。CSV 较稀疏记录的最小值为 0.1226721925 m，应优先引用原生 GT 的数值。

Full 目标接受确认延迟为 0.007877605 s；短目标为 0.009354752 s。Full 下发目标前 sim 10–15 s 的静止前缀含 251 个 GT、1251 个 LIO 样本，最大平面偏差 0.001604296 m，最终速度指令均为零。

## 已解决的问题与修改范围

### 1. 物理 IMU 的缓存速度与实际运动不一致

先前插件对 `Link.WorldLinearVelocity(offset)` 求导。在本轮首个失败 Full 中，机器人已经停止，sim 55–56 s 的原始 GT 位置差分速度为零，但该接口仍返回约 0.25 m/s 的缓存世界速度，导致惯性输入与实际运动不一致、LIO 大幅漂移。这个证据直接比较世界位置差分与世界速度，不依赖机体系/world 系 Twist 混用。

新增 `tools/stage2/lio_safe_validation/imu_physics/position_imu.cpp`：由 Gazebo 实际传感器安装点世界位置、姿态和原有 4 ms 物理时间差生成速度、加速度及角速度，再由原 ImuSensor 添加重力及既有噪声。它不读 ROS GT、导航 odom 或速度指令，也不写机器人位姿/速度。前两条物理样本只初始化，不伪造零加速度消息。

最终 Full 的 25201 对物理位置/速度连续差分、5040 个物理/GT 配对由离线审计核验；最大位置→速度残差约 `5.83e-13 m/s`，速度→加速度残差约 `6.78e-11 m/s²`。GT 只用于这一离线复核。6 个真实记录的反例测试覆盖错误位置、错误速度、缺行、GT NaN 和乱序，均被正确拒绝。

新 world `src/uav_bringup/worlds/provincial_stage2_lio_position_imu.sdf` 与原 Stage 2 world 的差异仅为 IMU 插件名与库名；墙、碰撞、机器人、物理步长和传感器发布频率未改。原插件和旧入口全部保留。

### 2. 为 LIO 入口增加一致的规划留距与最终守卫

新增 `lio_safety_geometry.py`、`lio_safe_navigation_interface.py` 和独立启动入口 `provincial_stage2_lio_safe.launch.py`。新入口会影响导航行为：

- 将墙体提供给 planner 的虚拟障碍包络向外扩 0.075 m。只改变此入口的规划障碍输入，原物理场地、PGM、GridRoute、EGO inflation 及 planner 二进制/算法/参数保持不变。
- 计划轨迹的矩形留距需要满足原 0.08 m 加 0.04 m 工程裕量。
- 最终输出前使用最新 LIO 位姿，按原始物理墙体重新计算 0.25 s 的矩形运动预测；要求间距至少 `0.08 + 0.04 + motion × (pose_age + 0.02 + 0.0125)`。过期位姿、无效/过期扫描、非有限值或不满足留距时发布完整零速度。
- 初始空闲、任务完成及无 active waypoint 的等待状态继续停车；原云修正/传感器 watchdog、轨迹安全与恢复规则保留。

曾试用原始雷达最近点加 0.035 m 余量直接停车，第二个失败 Full 表明该方法会在噪声和点采样下反复拒绝规划允许的走廊。最终版本明确将这项距离计算设为 **`diagnostic_only`**：其 0.115 m 阈值只做影子诊断，不宣称直接距离停车已通过。雷达有效性停车、动态障碍输入及上述更严格地图矩形守卫仍然执行。没有降低原有 0.08 m 物理验收间距。

最终 Full 的 1531 个有运动决策诊断，由独立审计使用原始 LIO 位姿和 SDF 多边形重新计算，几何/裕量残差为零。诊断有约 20 Hz 节流，不能称每一次最终发布都具有独立守卫诊断；最终 Twist 逐消息记录完整保留。Full 记录到 51 次裕量停车诊断，但任务未卡住、未重试，也未绕过守卫。

### 3. 证据采集与来源审计

记录器停止前有限排空已收到的回调，按原始输入、处理 dispositions、实际输出核验观察到的来源账本。Full 为 26041 个输入、26041 个处理记录、24997 个有效输出；初始化、乱序/旧消息与拒绝记录没有删除或补零。原始 LIO 存在高频预测与迟到修正的时间乱序，适配器按原契约拒绝旧时间输入；导航输出时间单调。

记录器健康、接收/写入计数、原始文件行数、输入/原始数据 SHA-256、实际 ROS 参数、source/install 和加载的插件身份均通过审计。实际加载新插件由运行时 `/proc` 映射留档，不仅依据 SDF 声明。

Full 当时的安装库为指向归档 build 的符号链接；之后通过可复现安装工具改为同字节的安装副本，短目标使用该副本。两次库内容 SHA-256 相同：

`ca5b00c09d2eff8d095920a10059d133c5dd5f317bb37ba140d379df61ca49a1`

实际解析路径不同，不能把后来的安装路径伪称为 Full 当时的路径。所有运行时输入快照保留。

## 失败和历史结果均保留

本轮并非所有探索试验都通过：

| 目录 | 结论与原因 |
|---|---|
| `full_01` | FAIL：旧缓存速度 IMU 与实际停止运动不一致，大幅定位漂移、卡住；证据和独立回放保留 |
| `short_position_imu_01` | 当时范围内 PASS：新物理 IMU 短路线验证；不是最终守卫版本 |
| `full_position_imu_01` | FAIL：原始最近雷达点直接停车造成反复拦截/卡住，另有传感器连续性检查未通过 |
| `full_reserved_map_01` | 最终版本 PASS，独立审计 v2 |
| `short_reserved_map_01` | 最终版本 PASS，独立审计 v2 |

原 Stage 1.5、步骤 1–3、原 Step 4 的 FAIL/PASS 历史结论不覆盖。本轮封存前核验上一封存的 211 项源文件、16 项证据保持原哈希。工作区原有大量 dirty 修改仍然保留，未提交、清理或回滚；当前状态和 diff 另外留档。

工具验证包括最终生产安全逻辑 17 项、物理证据反例 6 项、原 LIO watchdog 6 项，全部通过。早期工具失败日志及不完整 `full_native_geometry_review.json` 保留，不用于最终结论；有效图形分析报告是 `full_native_geometry_review_v2.json`。初始速度诊断混用了运动时 body/world Twist 坐标，不能引用它的全程最大差异作为根因证据；停驻世界位置差分证据由 `stale_velocity_stationary_world_audit_v2.json` 替代。

## 未覆盖和边界

1. 最终 Full 的最大 LIO 平面偏差为 **0.0548821745 m**。它超过 0.04 m 工程裕量，因此 4 cm 不是已经证明的定位误差上界，不能以本次通过保证任意路线安全。终点完成距 5 cm 阈值也较接近。
2. 只有一个最终版本 Full 和一个短回归，不宣称统计重复性。没有继续运行 Return、连续目标或新增障碍矩阵。
3. Full 启动前缀存在约 0.33–0.34 s 的传感器接收间断，**全文件传感器连续性未通过**；目标前就绪检查和完整任务期连续性通过。原始启动间断、时间乱序和拒绝账本均保留，没有用仿真时间连续来豁免任务期接收要求。
4. 模拟三维雷达是瞬时 360×16 点云、10 Hz，提供 MID-360 兼容字段；没有伪造逐点顺序扫描时间，也未验证真实 MID-360 扫描图案、数据包、硬件时间同步或实物噪声。IMU 250 Hz。理想驱动中的速度阶跃仍可能产生较大物理差分加速度，不是实车动力学标定。
5. 原生 50 Hz 采样间距、预测间距和几何不重叠均不构成连续时间安全证明或独立接触检测。`CONTACT DATA UNAVAILABLE`。
6. “未重置”由执行器代码、日志和唯一外部目标原始订阅记录支持；没有监控所有 reset/set_pose 服务调用。接收序号/健康计数也不能证明发布端绝无丢包。
7. 本地运行文件内容/参数封存不是完整操作系统镜像。正式比赛场地尚未核验，未接入真实硬件，未进入第 5 项。

## 文件与复核方式

证据根目录：`/home/rmnav/AeroMind/tools/results/stage2_step4_lio_safe_profile_20261002/`。

最终报告：本文；图：`stage4_full_verified.png`；完整收尾结论：`stage4_completion_audit.json`；封存：`final_evidence_seal.json`。两个最终试验的原生 JSONL、传感器记录、CSV、健康、时间诊断、参数、启动日志、输入快照与哈希均在各自目录。`independent_audit_v2.json` 是最终任务审计；旧 v1 保留。

只读再次核验最终封存：

```bash
cd /home/rmnav/AeroMind
python3 -B tools/stage2/lio_safe_audit/finalize_stage4.py --verify-existing
```

重新计算任务审计时使用**新的输出文件名**，现有文件不可覆盖：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/lio_safe_audit/audit_safe_navigation.py \
  --result-dir tools/results/stage2_step4_lio_safe_profile_20261002/full_reserved_map_01 \
  --output /tmp/stage4_full_independent_new.json
```

新入口的安装文件只读验证：`python3 -B tools/stage2/lio_safe_validation/install_profile.py`。在独立的新 checkout 构建安装新 profile 时用 `--install`，已有不同字节文件会拒绝覆盖。为保持冻结入口，本轮没有改旧包 CMake 安装清单。

如需之后手动查看最终配置的 GUI，可在已 source 环境运行 `ros2 launch uav_bringup provincial_stage2_lio_safe.launch.py navigation:=true gui:=true rviz:=true auto_goal:=false`；此命令只启动，不自动发送目标。**本轮收尾没有再次启动 GUI 或导航试验，已结束试验进程。**

![最终 Full 原生轨迹、间距与定位偏差](/home/rmnav/AeroMind/tools/results/stage2_step4_lio_safe_profile_20261002/stage4_full_verified.png)
