# 实车手推直线往返诊断工具准备（2026-10-18）

## 完成情况

新增 `tools/hardware/mid360_translation_check/`，保留转云台工具及全部历史数据；未修改旧硬件/导航文件、planner、控制、速度、场地或安装表。工具和合成验证完成，真实手推试验尚未运行。A-B实测距离和能否沿地面保持朝向手推，需由操作者提供。

上一轮真实转云台诊断已归档于 docs/real_mid360_joint_physical_review_20261018.md。不能把它当成平移尺度验证，也不能将61°转动套成0.5m平移结果。

## 实物准备与交互

在平整空地沿底盘固定机械前方用卷尺标出A、B，建议约0.50m；输入必须为实际测量值，不使用建议值作为默认。全程使用底盘同一个固定标记对准A/B；保持底盘朝向和云台相对底盘角度，云台对准机械前向。不抬起底盘，沿地面慢推；返回时保持原朝向，沿原线倒推回A。

记录器先检查LIO/IMU/电控输入及实际参数/来源。随后5次回车分别标记：

1. A点就位、停稳 → 自动记录初始静止5秒；
2. 准备出发，按回车后才推到B；
3. B点停稳、同一标记对准B、松手后按回车 → 自动静止5秒；
4. 准备返回，先按回车再沿原线倒推；
5. A点停稳、对准原标记并松手后按回车 → 自动静止5秒。

移动时长由实际起止回车决定，不强迫固定5秒推完。整次就绪后的操作最多180秒。没有输入来源、有限性或新鲜度时结束并提示停止手推，保留失败数据，不重跑。没有电机/导航/TF输出。

## 记录与计算

新串口记录线程只以O_RDONLY打开460800/8N1，检查其他占用者并使用TIOCEXCL；不清空已有字节、不含TX编码，结束恢复原termios。复用旧串口异步Writer和只读解析器，原30秒/转动工具不改；新人工操作模式有独立生命周期。保存所有接收字节、读取偏移/顺序/接收时刻，不伪造设备采样时戳。未监测DTR/RTS的实际物理变化。

ROS沿用真实phased corrected/predicted/mixed/TF/SI IMU逐消息CDR记录，保留原始消息及两种接收时间。实际参数和来源前后读取，封存工具内容、路径、符号链接目标、sealed raw/phased manifest和输入/数据哈希。不是完整操作系统镜像。

`reference.json` 单独保存操作者报告的卷尺距离、同一标记和方向声明；不确定度未知写null。`--distance-m` 为必填参数，缺值、NaN、非正数或超过本工具2m短程范围拒绝。

独立审计重建人工阶段顺序和覆盖，静止取最后2秒原始corrected样本。连续性保留LIO .2s、SI IMU .1s、串口 .2s，主机最近接收时刻配对50ms只为诊断筛选，不是时间同步精度。没有插值、补零、丢弃坏记录来制造通过。

暂定圆心 `pC=pI+R_L_I arm_I`，初始车体系 `R_L_B0=mean(R_L_I_baseline) R_G_I^T`，位置变化 `delta_B0=R_L_B0^T (pC-pC_baseline)`。依声明机械前向和理想倒装定义X/Y/Z，未按实际轨迹拟合前方轴或比例，也未对齐正式地图/重力。实时相对角用于朝向抵消，不能把真实底盘平移/转动清掉。

输出：X前向估计距离、与输入实测值之差、比例、Y侧向/Z初始车体轴位移、往返闭合差、最大朝向/相对角变化、原始配对索引与时间差。若底盘实际转动，同一外壳标记的位移不再直接等于圆心位移；仍需由操作者和数据共同解释。安装尺寸是粗测，设备时间关联未知，不能将差值归因为唯一传感器误差。

`diagnostic_complete=true` 仅表示记录和独立审计完整，不是尺度/方向精度PASS；零位移、反向、侧移或错误比例直接保留为对应统计，不通过拟合修正。`forward_direction_consistent` 只检查去回X的符号。`accuracy_accepted=false`、`navigation_ready=false` 始终保持。

## 实机运行

保持真实雷达和phased LIO在域43运行。在NoMachine普通交互终端执行，distance_m替换为实际卷尺测量值：

```bash
cd ~/AeroMind_20261016
MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_translation_check/run.sh \
  --distance-m 实测米数
```

默认新目录：`~/AeroMind_mid360_translation_checks/push_return_<timestamp>/`，已有输出拒绝覆盖。

原始记录关闭后独立重算到新文件：

```bash
MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_lio_phased/env.sh \
  python3 -B tools/hardware/mid360_translation_check/translation_review.py \
  /home/rm/AeroMind_mid360_translation_checks/push_return_实际时间 \
  --output-name independent_translation_review_v1.json
```

## 工具验证

16项测试通过：倒装及任意LIO初始yaw、侧移不当作前进、错误尺度不拟合、反向保留、底盘yaw保留、相对角补偿、参考缺失/非法、阶段缺失/过短、原始NaN/断流、全部流身份及有限性、原始哈希、重复输出拒绝、失败不判完成，以及实际PTY的O_RDONLY采集/无TX观察与占用者拒绝。

隔离DDS域93的实际生产记录器与虚拟UART完整执行：14842条ROS记录完整写入，参考0.50m的合成去回保留为+0.50m/−0.50m，独立重算一致，0个发送payload被对端观察到。只注入虚拟串口路径和模拟操作回车时间，没有替换生产几何或审核判据。这不是实机精度。

域94合成IMU在初始静止2秒后断流：生产监控结束记录，未开始outward阶段，原始数据及失败结论保留。sealed原raw/phased148项核验仍通过。

证据：docs/evidence/mid360_translation_tool_20261018/。开发机合成原始文件：tools/results/mid360_translation_tool_20261018/。当前仍无本轮实物平移数据；不发布实车平移通过结论，不启用电机闭环。
