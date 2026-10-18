# 导航反馈只读接收准备（2026-10-18）

范围按用户本轮要求收紧：只关心导航反馈；不继续检查或修改自瞄。
本轮完成接收工具和开发机测试，真实电控反馈尚未采集，不称为实机通过。

## 已知实机信息

用户回报小电脑存在：
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -> ../../ttyUSB0`。
`fuser -v`未显示占用进程，仅代表该次命令观察范围；工具启动时会再检查。
波特率460800由用户给出。现有真实雷达/LIO入口和已通过记录保留。

上一轮操作者运行的圆心预览：
`/home/rm/AeroMind_mid360_gimbal_checks/turn_return_20261018_121552/mount_preview_v1.json`。
贴出统计：转后保持位移30.8897mm→5.8186mm；回原位差8.3547mm→8.8622mm；
最大采样位移46.0982mm→34.0724mm，均为三维距离。粗测杠杆补偿预览已执行，
不等于定位精度验收；本机未独立取得该physical JSONL。本轮不调整25/550mm
来压低统计，也不把暂定圆心反馈给导航。

## 最小接收范围

采用用户提供xuc_host的39字节反馈布局，只解释导航字段：

| 偏移 | 字段 | 判据 |
|---:|---|---|
| 22 | nav_allowed | 有效位bit2，值0或1 |
| 23 | nav_valid_flags | uint16小端；原值保留 |
| 25 | chassis_vx | float32 m/s、有效位bit0、有限值 |
| 29 | chassis_vy | float32 m/s、有效位bit1、有限值 |
| 33 | gimbal_relative_angle | float32 rad、有效位bit3、有限值 |
| 37 | CRC | 小端；覆盖前37字节 |

不解释其余应用字段、不启动ROS节点、不发任何控制包。先检查声明的MODBUS
0xA001/initFFFF；另列0x8408/initFFFF诊断结果，仅作为可能的校验不一致线索，
不会据此切换控制协议。只查39字节导航布局；未见该格式不能推断雷达故障。
连续至少3个CRC及字段结构有效帧仅用于辨认候选，不是位置、方向或链路验收。
有效位为0不补值；NaN、损坏、歧义或没有格式匹配均不成为有效导航角度。

## 小电脑操作：15秒静止采集

保持底盘、云台静止。若其它程序已经打开同一串口，本工具拒绝启动；不自动
停止进程。直接在普通终端运行，不需要source ROS或重新编译：

```bash
cd ~/AeroMind_20261016
git pull --ff-only
python3 -B tools/hardware/xuc_feedback_check/capture.py \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baud 460800 --seconds 15
```

新目录：`~/AeroMind_xuc_feedback_checks/raw_YYYYMMDD_HHMMSS/`。
同名目录拒绝覆盖，无自动重试。
保存raw.bin、逐次读入元数据reads.jsonl、capture_meta.json、独立解码派生文件
和summary.json。异步有界写盘保留每个实际读入字节；队列溢出/写盘失败/断开/
中断/源文件变化均记录为不完整。原始字节不排序、不补零、不flush清空。
每个读块保存调用开始/返回的单调时间及主机wall时间、偏移、长度；一帧完成
时刻只能关联至所在主机读块，绝不伪造成设备生成时刻或使用发射tick代替yaw采样时间。

接收器用O_RDONLY打开设备，设置460800/8N1/无流控并读回核对，使用flock和
TIOCEXCL防止后续竞争，结束后恢复原TTY参数。未向串口写字节，也未发DTR/RTS/
reset ioctl；实际开口时驱动控制线变化没有独立观测，不声称检测过所有复位。
不会改系统权限、udev、驱动、网络或固件；权限等失败保留报告而不自动修系统。

`capture_complete`只描述本次原始记录完整；即使字节为0，也不能靠该字段
判定链路通过，须看format_status与有效角度帧数。退出0要求完整采集、观察到
声明的39B/MODBUS候选并至少3帧相对yaw有效；navigation_ready永远false。
空闲阶段nav_allowed=0可作为有效反馈，并不要求导航接管。

独立重读（输出名不得已存在）：

```bash
python3 -B tools/hardware/xuc_feedback_check/inspect_wire.py /实际采集目录 \
  --output-name independent_wire_review_v1.json
```

## 开发机证据与下一步

12项测试通过，包含CRC已知向量、39B解码、缺有效位、NaN、坏CRC/半包、
诊断CRC不启用声明协议、歧义、接收元数据及缺原始哈希拒绝、真实生产接收器
在虚拟串口收100帧/3900字节、对端未收到发送字节、已占用设备拒绝、篡改拒绝。
另一次持久虚拟串口记录及独立CLI复核通过，重复输出拒绝。不是连接真电控的试验。
证据：`docs/evidence/xuc_navigation_receive_20261018/`。

实机收到有效导航角度后，再做固定底盘、手转云台的相对角正负/回绕/零位核对；
仍不下发速度。上电零位与机械前向零位尚未关联，不能先填0。没有角度测量设备时
也不能把约45度操作目标当真值。SOURCE/INSTALL导航与传感器基线不改。
