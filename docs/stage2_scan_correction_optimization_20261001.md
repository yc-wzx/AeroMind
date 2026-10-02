# Stage 2 二维激光定位校正最小优化 — 2026-10-01

本轮完成：定位漂移失败现场复核、独立校正模式、隔离测试、三次限定 Full、原始证据独立复核和归档。

状态仍为 TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED。
这是当前仿真条件下的最小修复验证，不表示 Stage 2 全部完成。

## 问题与处理

上轮未校正的航向偏置在 boundary_6 处降至 0.0799625192 m，随机游走在 boundary_7 处降至 0.0799857440 m，均触发只读试验中止。估计位姿下的几何守卫仍判断允许运动：它不能独立识别估计位姿与真实位姿的偏差。历史 FAIL 原样保留。

本轮新增独立启动入口，使用当前场地已有二维 /scan 和相同 collision_segments 墙体的实际矩形墙面进行局部定位校正。先进行有界鲁棒拟合，再重关联墙面，检查内点、残差、三个自由度的可观测性和修正幅度。校正增益 0.5；扫描和原始里程计最大关联误差 25 ms。

链路为：

```
Gazebo GT → 原有误差注入 → 原始 /simulation/navigation_odometry
                         + /scan + 已知场地墙面
                         → 独立 Stage2 校正接口
                         → /localization/navigation_odometry
                         → 原导航 /ground/odometry → EGO / executor / 原最终守卫
```

校正代码不订阅 GT 或含 GT 的 /simulation/odometry_diagnostics。只有试验记录器订阅它们做只读评估，GT 安全检查只结束试验，不反馈校正或控制指令。

未经校正的误差仍累计：航向偏置新试验最大位置误差 127.070 mm，随机游走新试验 29.371 mm。校正后最大位置误差分别约 5.798 和 6.511 mm。上述误差只对本次收到的同时间戳原生样本统计。

当没有新鲜有效匹配时，新增接口通过原生产 sensor watchdog 停车；门槛同时检查仿真和单调时间，均为 0.5 s。接收目标前也必须有新鲜校正。隔离测试执行实际生产 update() 主体，验证缺失/过期匹配时最终三轴零输出，新鲜匹配且原安全条件满足时允许恢复。本轮未做真实 Gazebo 扫描断流/恢复试验。

## 实跑结果

每个 profile 只启动一次独立 Gazebo、只发一个外部目标：(4.7,0.5), yaw=90° → (8.7,4.25), yaw=90°。不 reset/set_pose，不外发中间目标。全部由导航切换 R0001:W00/W01/W02。

| 场景 | 独立审计 | GT 到点误差 | odom 到点误差 | 原生采样最小间距 | 校正后最大定位误差 | 含停车窗仿真耗时 |
| --- | --- | --- | --- | --- | --- | --- |
| 零误差 | PASS | 1.716 cm | 1.659 cm | 8.266 cm | 6.876 mm | 75.52 s |
| 航向偏置 | PASS | 1.800 cm | 1.819 cm | 8.389 cm | 5.798 mm | 74.30 s |
| 随机游走 | PASS | 1.551 cm | 1.513 cm | 8.439 cm | 6.511 mm | 75.50 s |

全部内部 waypoint、原生与 CSV 完整 2+5 秒停车窗通过；重试、卡住和 false success 记录均为 0。三个最窄点均为 boundary_7。门槛仍是 0.08 m，三个新样本没有低于门槛。

原生 GT 接收仿真频率均为 50 Hz，最大消息仿真间隔 0.020 s。接收连续性保持 GT/最终指令 0.2 s、odom 0.5 s，均通过。最大一次校正计算耗时分别约 5.820/7.252/5.166 ms；没有改变物理步长或发布频率。

零误差校正引入激光噪声，其最大定位偏差约 6.876 mm；新零误差最小间距 0.082657 m 小于旧未校正零误差的 0.084080 m。没有证据宣称所有指标单调改善。

## 工具验证和独立审计

28 项工具/隔离测试通过。原始零误差数据通过，五个临时副本反例均不能 PASS：删除扫描、改变扫描距离、篡改校正量、改变校正后位姿、删除校正后样本。副本哈希已按改变后的数据更新，拒绝不是仅靠哈希不匹配。原始记录没有修改。

v3 审计从原始扫描重新运行每次接受的任务期匹配，与保存修正量的最大代数差异为 0；同时复核原始误差模型逐增量递推和每个校正输出。全部通过。每次运行均封存 103 个实际本地依赖及链接目标/哈希、启动命令、版本、Git 状态、实际参数和原始文件哈希；未封存全部操作系统/系统库和所有动态加载插件。

零误差 v1 的定位证据审计 FAIL 保留：四条订阅发现前缀样本缺少原始配对或上一校正状态。v2 明确将任务定义在实际目标发布之后，并逐条验证所有任务期校正输出；不豁免行驶期缺值，不伪造前缀。v3 进一步核验扫描重算和每个原始里程计任务期输出的对应校正样本。历史 v1/v2 报告保留。

复核命令（新输出路径必须不存在）：

```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/audit_localized_replay.py \
  --result-dir tools/results/stage2_localization_20261001/yaw_bias \
  --output /tmp/stage2_yaw_bias_reaudit_unique.json
```

运行前必须自行确认新目录；不要重复本轮 Full。GUI 手动入口（不会自动发目标）：

```bash
ros2 launch uav_bringup provincial_stage2_localized.launch.py \
  odometry_profile:=/home/rmnav/AeroMind/src/uav_bringup/config/stage2_odometry_yaw_bias.yaml \
  gui:=true rviz:=true auto_goal:=false
```

启动前先检查残留进程和实际参数；不能与另一个 Gazebo/导航并行运行。

## 文件与基线保护

新增导航相关源：
- src/uav_planning/scripts/stage2_wall_localization.py：纯匹配与 SE(2) 校正数学。
- src/uav_planning/scripts/stage2_localized_interface.py：仅独立入口使用，改变送入原导航的定位估计；匹配过期时复用原 watchdog，因此会影响本入口的最终输出是否停车。
- src/uav_bringup/launch/provincial_stage2_localized.launch.py：仅替换独立 Stage2 的接口可执行文件。

新增工具：tools/stage2/run_localized_experiment.py、run_localized_trial.py、audit_localized.py、audit_localized_replay.py、test_localization.py、check_localization_counterexamples.py、plot_localization.py。

已有文件本轮仅改 src/uav_planning/CMakeLists.txt，新增两项安装条目。其他历史 dirty 修改均保留。Stage1.5 原导航、planner、executor、参数、world/PGM/collision 没有改变，92 个封存运行输入仍全部匹配，原 Stage1.5 只读完成汇总再次通过。上轮 Stage2 封存证据全部匹配，源文件中只有 CMake 新安装条目产生预期哈希差异；原未校正 Stage2 入口和模型保留。

构建使用 `colcon build --base-paths src --symlink-install --packages-select uav_planning uav_bringup`。第一次未限定搜索路径的构建遇到证据快照中 package.xml 导致重名；保留失败日志，限定 src 修复，不删除证据目录。扫描面配对第一版隔离测试失败也保留，修正后才启动仿真。

## 剩余优化与限制

1. 最小间距仅比门槛多约 2.7–4.4 mm；校正后的瞬时位置误差可达到 5.8–6.9 mm。两个数发生时刻不同，不能直接相减证明某次已经违反门槛；但这样的余量不足以宣称对定位不确定性稳健。下一轮应离线分析最窄点的法向误差、姿态误差、同步误差和守卫预测，不降低门槛。
2. 校正只适用于当前已知平面墙面与已知、共位且同向的二维雷达安装。需要补齐明确的扫描 frame/extrinsics、无效元数据、错误地图、离群/动态返回和断流恢复门控；本轮没有对这些情况作完整 live 验证。
3. 现有 covariance 仍是继承的原始输入，不能当成校正后的标定后验不确定度；没有完整 map→odom 与真实定位架构，也不是全局重定位。
4. 每种新场景只有一个样本；未验证混合误差、校正模式的 scale profile、新随机种子、Return 或统计重复性。不能据此扩展成功结论。
5. 未接入仿真 MID-360、IMU 或 LIO。当前二维激光校正是隔离的最小工程验证，不替代将来真实传感器标定和融合。
6. CONTACT DATA UNAVAILABLE。采样几何正间距不等于独立接触检测通过；没有连续时间安全证明，没有正式省赛场地验收。

## 证据路径

- 汇总：tools/results/stage2_localization_20261001/optimization_verification_v1.json
- 每个场景：zero/、yaw_bias/、random_walk/independent_audit_v3.json
- 输入：各场景 input_snapshot/、input_manifest.json、input_sha256_after.json、runtime_parameters/
- 原始：各场景 stage2_localized_<profile>_01.native.jsonl、native.health.json、native.timing.jsonl、CSV/events/plans/actuation/published_commands 及 launch.log
- 反例：localization_counterexamples_v1.json；测试：tests_final_v2.log
- 图：localization_comparison.png；本轮增量修改：incremental_changes.patch
- Stage1.5 复核：stage15_reverification.log

本轮三个指定修复样本与独立审计已完成。暂停进一步实跑，保留上述余量和传感器异常处理限制。
