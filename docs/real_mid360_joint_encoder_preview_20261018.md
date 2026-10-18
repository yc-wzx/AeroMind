# 真实 LIO 与云台编码器联合只读检查（2026-10-18）

## 当前完成情况

工具已完成，实机联合试验等待执行。导航、控制、planner、速度输出、安装表、原 raw/phased 文件及历史记录均保留；新增 `tools/hardware/mid360_joint_check/`，不替换旧工具。

最新实机编码器报告：`/home/rm/AeroMind_xuc_feedback_checks/raw_20261018_143723/`。用户粘贴的报告显示 5006 有效帧、主机接收约 200.2426 Hz、最大间隔 5.445 ms，39B CRC8408，左转 +55.5469°、返回 −52.3828°、回原位差 +3.1641°。据此观察到左正右负；没有独立角度真值，不能将回位差解释为编码器精度。仅收到报告，开发机未获取实机原始文件重新计算。

## 模型与边界

继续使用用户暂定测量：转轴在圆心上方，安装面中心在转轴正前约25 mm、离地约550 mm，倒装且原生+X与云台前向一致。未改变或拟合这些尺寸。官方标称内部偏移仍只组合一次；IMU到轴地面交点的固定杆臂候选 `[-0.036,-0.02329,0.54712] m`（IMU系）。

记 `R_L_I` 为原始 LIO 姿态，`R_G_I=Rz(alpha)Rx(pi)` 为理想倒装关系；初始云台与声明机械前向对齐，编码器中位数只作为本轮前向候选。`theta=encoder-encoder_initial_forward`，候选姿态为 `R_L_B=R_L_I R_G_I^T Rz(-theta)`，再相对初始姿态报告变化。先完成倒装变换，不直接拿原生 Euler yaw 减角度。

同轴条件下圆心位置候选 `p_L_C=p_L_I+R_L_I arm_I`。位置不依赖编码器，绝不强制圆心不动；朝向计算才使用实时反馈。任何真实底盘平移/转动仍应保留。若安装表不再声明同轴，工具拒绝该捷径。

原始位姿仍在 sensor-local LIO 坐标；这里没有地图、重力或绝对底盘朝向验收。所有位置距离为3D采样距离，非真实定位误差。粗测尺寸无误差界，长竖直杆臂会耦合姿态误差。

## 时间与证据

只读 ROS 记录器保存 corrected / predicted / mixed / TF / SI IMU 的逐消息原始CDR、单调接收和墙钟接收时刻；独立串口子进程保存所有原始字节与读取完成时间。只读取 GetParameters，记录前后来源、参数、工具内容快照、符号链接目标与哈希，核验 sealed raw/phased 输入。

无设备采样时戳的编码器不伪造时间。离线使用同一主机的最近接收时刻配对，50 ms只是配对筛选上限，不是同步精度或测量时延界。相邻相位不插值，匹配行保留编码器原始 frame offset/read time 及 LIO 原始 stamp/sequence。既有接收连续性判据保留：LIO .2 s、SI IMU .1 s、串口 .2 s。完整5阶段各约5秒，静止统计取每阶段最后2秒原始样本。

LIO计算/传输时延、编码器采样时刻未知，不拟合时延。动态残差不能单独归因于外参、LIO或编码器误差。只计算统计，不发明“稳定通过”的精度门槛；`motion_check_complete=true` 仅表示证据完整且两路均实际转出/返回，`compensation_accuracy_accepted=false`、`navigation_ready=false` 保持。

串口仍调用已验证的 O_RDONLY 采集，显式 CRC8408；无串口TX编码、运动指令或新的 ROS/TF 发布。ROS GetParameters 自带服务请求不属于导航控制输出。未监控 reset/set_pose 服务或物理电机状态。来源/参数预检不通过时不开始串口运动检查；检查期间输入过期会中断自己的串口子进程，保留失败数据并提示停止手动转动，不停止其他程序。

新目录默认 `~/AeroMind_mid360_joint_checks/turn_return_<timestamp>/`，已有目录/报告拒绝覆盖；不自动重试。`input_snapshot/` 留存本地硬件工具，sealed manifest 引用已验证构建内容；这是工具快照与构建追溯，不是完整操作系统镜像。

## 实机运行：三个终端

更新代码，无需重新编译。先确认雷达供电、网线和本轮主机IP，192.168.1.5能ping通192.168.1.173再启动。LivoxViewer不与ROS驱动同时占用雷达。

终端1：
```bash
cd ~/AeroMind_20261016
git pull --ff-only
MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360/run.sh
```

终端2（终端1保持运行）：
```bash
cd ~/AeroMind_20261016
MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_lio_phased/run.sh \
  /home/rm/AeroMind_mid360_captures/20261017_214624/summary.json
```

等待LIO初始化。底盘固定不动，云台对准并标记机械前向，终端3执行：
```bash
cd ~/AeroMind_20261016
MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_joint_check/run.sh
```

就绪后跟随25秒自动提示：静止5秒→左转约45°用5秒→静止5秒→回原标记用5秒→静止5秒。不按回车，不反复调整。异常停止时停止手转并提供输出，不通过重复尝试选一次成功。

独立重算到新文件：
```bash
MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_lio_phased/env.sh \
  python3 -B tools/hardware/mid360_joint_check/review.py \
  /home/rm/AeroMind_mid360_joint_checks/turn_return_实际时间 \
  --output-name independent_joint_review_v1.json
```

## 验证结果与限制

18项新测试、19项既有接收/提示测试通过，覆盖倒装、非零初值、±π跨越、实际底盘运动保留、错误反馈残差不被拟合、NaN、断流、顺序、原始哈希、重复输出拒绝及失败不能PASS。当前采集/审计/几何生产文件哈希与成功端到端采集快照一致。

隔离DDS域89的合成5话题+虚拟串口实际完成25秒：16192条ROS原始消息全部写入，无溢出/错误；独立重算相同；250条corrected任务样本；最近接收配对最大约1.138 ms。该波形理想无噪声，残差极小不代表实车表现。

域90无输入试验：拒绝开始，未创建串口子进程。域91合成IMU在初始静止2秒后断流：实际生产监控中断自己的采集子进程，只有baseline标记，未提示开始左转，证据不完整判定保留。

首次合成试验保留：导入测试辅助函数的通用模块名冲突已改成测试内显式构帧；随后采集审计通过但无TX检查发现虚拟终端恢复canonical/ECHO后回显。独立PTY实验在完全不写slave的情况下复现恢复后自动回显；只将测试PTY初始状态设为raw，未改生产termios恢复逻辑。成功fixture对端未收到TX payload。旧失败数据和日志仍保留。

证据：`docs/evidence/mid360_joint_check_20261018/`。合成原始数据留在开发机 `tools/results/mid360_joint_check_20261018/`，不把它冒充实机证据；测试中无Gazebo、导航或物理设备。

本轮完成：联合工具、测试、文档与旧结果归档。未完成：实机联合残差测量、精确尺寸复测、设备时间同步、地图对齐与真实电控闭环。没有据工具通过宣称真实导航可以开跑。
