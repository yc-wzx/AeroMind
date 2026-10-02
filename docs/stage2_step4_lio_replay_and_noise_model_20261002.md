# Stage 2 第4项：离线定位诊断与仿真噪声模型修正

**本轮未全部完成；第4项尚未通过完整闭环验收。**
TRAINING-ONLY / PROVISIONAL · COMPETITION ARENA NOT VERIFIED · CONTACT DATA UNAVAILABLE

## 完成的工作

在 DDS 域73/74独立回放旧 Full 的原始三维点云、IMU及仿真时钟。没有启动导航、发布目标或速度，也没有向 LIO 发布 GT。原始 CDR 内容不变，回放以0.5倍墙钟速度执行，消息自身时间戳保持原值。每次完整回放交付22688条IMU、908帧点云、22689条时钟，观察到905帧初始化后的点云校正。GT仅供离线误差计算；最近样本配对限25ms，不插值。

| 隔离实验 | 旧完成时刻 LIO–GT xy误差 | 全程最大xy误差 | 结论 |
|---|---:|---:|---|
| baseline | 5.2054 cm | 8.1240 cm | 原误差复现 |
| planar | 5.2555 cm | 6.4296 cm | 未解决，未采用 |
| instrumented_original | 5.2054 cm | 8.1328 cm | 诊断日志不改变完成时结果 |
| first_point_fix | 5.3313 cm | 8.4354 cm | 点集缺陷修复，但定位误差未改善；未并入生产 |
| declared_noise_model | 0.4745 cm | 3.3884 cm | 离线改善，新增可选仿真入口 |

上述误差是同一旧物理轨迹上的定位偏差，不是新导航到点误差。旧 Full 的 GT 到点误差5.7957cm超出完成事件5cm阈值，FAIL继续保留。

## 有依据的最小修正

22688组原生IMU与物理插件诊断配对，实测加速度标准差约0.001、角速度标准差约0.0001，均在声明值10%以内。旧滤波配置Q分别为0.01/0.001，且设置0.0001的偏置随机游走；本仿真没有注入偏置随机游走。滤波源码将参数直接放入Q对角线，以(dt·f_w)Q(dt·f_w)^T传播。

新增仿真专用配置acc_cov=1e-6、gyr_cov=1e-8、b_acc_cov=b_gyr_cov=0，保持3D模式。没有修改旧默认配置、滤波算法、导航、控制、车体、地图或传感器频率。该设置不应直接移植真实IMU。

原始去畸变循环存在首点重复处理：内层break没有退出外层IMU区间循环。隔离副本改为return后，76个检查帧中每帧1个不一致点变为0；但终点定位误差未改善，不能把它认定为本次Full失败的唯一原因。生产二进制未修改。

## 唯一一次新 Gazebo 尝试

model_full_01只启动一个独立Gazebo，参数预检阶段失败，外部目标发布次数为0。新运行器遗漏同目录dump_parameters.py入口，实际调用无法打开文件；不是导航运行后失败。保存输入快照175项及原始数据后关闭进程。独立审计为INCOMPLETE_EVIDENCE / NOT_EXERCISED；未经过boundary_7，没有新的终点、间距或2+5秒通过结论。

缺失入口反例已从运行时快照复现。现新增入口复用封存的参数读取器；查询保存命令、返回码、stdout/stderr及单调起止时间，启动前检查入口存在，运行器异常退出码为2。DDS域75真实ROS参数服务及缺失服务、超时、历史文件保护等6项检查全部通过。没有再次启动Gazebo。

## 文件和证据

- 新工具：tools/stage2/lio_replay/ 与 tools/stage2/lio_model_validation/。
- 可选入口：src/uav_bringup/launch/provincial_stage2_lio_noise_model.launch.py。
- 可选配置：src/uav_bringup/config/spark_provincial_noise_model_sim.yaml。
- 本轮汇总：tools/results/stage2_step4_lio_replay_20261001/round_review.json。
- 比较图：noise_model_analysis_v1/replay_comparison.png。
- 原始/候选点集核验：original_point_boundary.json、fixed_point_boundary.json。
- IMU诊断：imu_model_audit_v1.json。
- 预检失败证据：model_full_01/progress.json、input_snapshot/、independent_audit_v2.json。
- 入口修复反例及6项测试：parameter_entry_tests_01/。
- 初始diagnostic_01误加载旧组件，随后发现并停止；保留原记录，不纳入有效实验。后续通过/proc/pid/maps核验实际动态加载组件路径和哈希。
- baseline_analysis_v1为空的失败尝试、v2旧几何分析均保留；采用v3。其他分析内corrected_cloud_pose_matches仅描述原试验点云，不代表候选输出；候选用独立point-boundary核验。

## 基线保护与剩余工作

- stage15：92项哈希核验，无差异。
- steps123_source：28项哈希核验，无差异。
- steps123_evidence：532项哈希核验，无差异。
- step4_source：174项哈希核验，无差异。
- step4_evidence：1734项哈希核验，无差异。

新配置及launch的source/install内容一致。系统依赖没有封存为完整系统镜像。接收序号不能证明发布端绝无丢包；回放不能提供新的接触检测、连续时间安全或统计重复性证据。

下一步仍是第4项闭环验证：使用修复后的可选入口运行一个新的Full，保持唯一外部目标、现有安全阈值及独立完整2+5秒/waypoint/原生采样审计。通过前不切换默认基线、不宣布第4项完成、不进入真实硬件第5项。
