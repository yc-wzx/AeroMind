继续维护 /home/rmnav/AeroMind。

本轮目标：只补齐新增 Stage2 二维激光校正入口的输入有效性和失效保护，并离线分析窄通道安全余量。不启动 Gazebo，不发送目标，不接入 MID-360/IMU/LIO，不自动扩大测试。

先阅读 docs/stage2_scan_correction_optimization_20261001.md。
核对 tools/results/stage2_localization_20261001/ 下 optimization_verification_v1.json、final_evidence_seal.json、三个 independent_audit_v3.json、localization_counterexamples_v1.json、tests_final_v2.log、incremental_changes.patch。

保留历史原始数据和 FAIL。Stage1.5 92 项封存输入不动；原未校正 Stage2 入口/误差模型不动。当前只证明零误差、yaw_bias、random_walk 各一个新 Full 样本及独立审计通过，不证明统计重复性。

保持 imperfect_sensors=false、GridRoute clearance=0.40m、EGO inflation=0.22m、最小间距门槛0.08m，以及原 planner/executor/车体/速度/场地/物理步长/话题频率。
只允许改新增 stage2_localized_interface.py、stage2_wall_localization.py 及必要评估工具；不调 planner，不用 GT 校正，不降低门槛。

1. 检查 git/source/install/残留进程、实际历史输入与当前文件区别。只有 CMake 新安装条目是上一轮相对旧 Stage2 源封存的预期差异。

2. 先隔离复现输入验证缺口：目前 matcher 拒绝无效扫描元数据后，接口仍可能调用基类 scan_callback；未知 frame 和未经确认的外参也缺少完整门控。
   根据保存的实际 /scan frame 和当前 SDF 安装建立明确约定。
   无效 frame、非有限/错误角度元数据、错误 range_min/range_max、过期/未来/乱序消息，不能改变地图点云、动态障碍、校正状态或传感器新鲜度，不能继续作为有效定位输入。
   LaserScan 中正常无返回的 inf 与无效元数据要区别处理，不允许把缺值补零。
   对不足内点、退化或地图不匹配明确记录拒绝；保留有依据的过期停车机制，不伪造成功或强行恢复。

3. 隔离测试必须执行真实生产 callback 和 update()，只替换隔离的发布者/时钟等外部依赖。
   验证有效扫描不回归、无效扫描不进入基类处理、丢失定位后最终输出为零、新鲜合法扫描/轨迹恢复后才允许运动。
   不向运行中的 ROS 导航链路注入测试数据。

4. 只读分析三个新 Full 的全程最窄点，分别约0.082657、0.083886、0.084394m，均boundary_7。
   建立最窄点前后至少2秒时间轴，从原始同时间戳 GT/原始注入/校正后位姿分析朝墙法向误差、yaw误差对矩形的影响、计划/预测/实际间距及指令时序。
   不把全程最大位置误差直接从不同时间的最小间距中相减。
   区分50Hz原始采样、守卫预测和连续时间推断；证据不支持连续下界时写未证明。
   不把继承的原始 covariance 当成校正后标定后验。

5. 用临时副本制作无效 frame/元数据、缺失扫描、篡改修正量等反例，保存新审计版本，原始数据不动。
   重新独立审核历史三次样本，明确本轮输入门控改动尚未经新Gazebo实跑。离线/隔离通过不能替代新的live失效保护验证。

6. 输出中文聊天简报：本轮是否完成、修改文件及输出影响、反例结果、窄处同时刻误差和余量、source/install状态、历史复核、未覆盖项，以及下一轮是否需要一次受控短距离扫描断流/恢复试验。

继续 TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。
本轮不自动进入三维雷达/IMU融合或改规划器。
