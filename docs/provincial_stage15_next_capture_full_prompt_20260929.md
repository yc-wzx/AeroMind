继续维护 /home/rmnav/AeroMind。

本轮目标：使用异步写盘和耗时诊断修复后的工具，最多启动一次独立 Gazebo，只发送一个正向 Full 目标，核验接收连续性、完整任务和原生 GT 最小间距。

状态保持 TRAINING-ONLY / PROVISIONAL、COMPETITION ARENA NOT VERIFIED，不进入 Stage 2。

1. 核对输入与范围
阅读 docs/provincial_stage15_capture_fix_20260929.md，以及 tools/results/provincial_stage15_capture_fix_20260929/ 下的 fix_verification.json、tests_final.log、historical_full_reaudit_v2.json 和 changes.patch。
55 项工具测试通过不等于真实 Full 恢复。旧试验的 0.343483 s 接收停顿唯一原因仍未证实，旧 FAIL 保留。不得把异步写盘修复写成已证实解决了全部实跑停顿。
检查 git status、source/install、残留 Gazebo/ROS 进程及当前工具哈希。保护历史数据和其他 dirty 修改。
保持 imperfect_sensors=false、GridRoute clearance=0.40 m、EGO inflation=0.22 m、当前车体/速度/场地/PGM/collision/等待停车与恢复逻辑。
不修改导航、控制、planner、最终速度输出、路线、物理步长、传感器频率或任何验收阈值。不运行 Return、A→B→C 或障碍测试。

2. 一次运行前的证据留档
使用指定新目录 tools/results/provincial_stage15_full_capture_20260929，trial_id=trial_full_capture_01。目录若存在，停止并报告，不覆盖、不自动另起目录。
发送目标前保存实际递归本地依赖的内容、路径、链接目标、SHA-256、Git HEAD/dirty diff、版本、启动命令。闭包按当前实际导入解析，不硬编码为旧 42 项。包含本轮记录器、运行器、评估器及其本地依赖，也包含实际导航基类/scan matcher、executor/adapter/YAML、planner 二进制、地图/场地和 launch。
启动后保存实际 ROS 参数，确认固定参数。依据真实 GT 完成起点/目标空闲、同 component、GridRoute、参考段 clearance、yaw=90° 矩形静态间距检查。任一预检不通过，不发目标；本轮不再启动第二个 Gazebo。

3. 唯一 Full
在已 source ROS/install 的 shell 中执行一次：
python3 -B tools/run_provincial_full_observed.py --output-dir /home/rmnav/AeroMind/tools/results/provincial_stage15_full_capture_20260929 --trial-id trial_full_capture_01
起点约 (4.7,0.5)，yaw=90°；唯一外部目标 (8.7,4.25)，yaw=90°。
保留导航订阅端身份、GID、QoS 匹配和稳定时间门槛。只发布一次目标，5 秒内必须收到导航节点的新鲜、route/最终坐标匹配的接受事件。
拒绝或超时则保存证据并结束，不重发、不重启。接受后由导航自行切换内部 waypoint；不外发中间目标、不 set_pose/reset、不清任务状态。

4. 记录和接收连续性
保留 CSV 和全部逐消息 native JSONL；不得抽样、补零、插值或重复旧数据。无自身时间戳的 Twist 只记录接收时间。
必须保存 trial_full_capture_01.native.health.json 和 trial_full_capture_01.native.timing.jsonl。
区分回调入口接收时间、后台队列等待、序列化/写盘耗时。健康文件必须证明正常关闭，逐话题收到/写出计数与原始文件一致；队列溢出、写盘失败、关闭超时或健康/耗时证据缺失不能 PASS。
保留每次订阅回调和 ros_graph_query 的起止单调时钟。若出现共同接收停顿，对齐回调耗时、节点图查询、后台写盘与队列延迟；只有时间证据支持时才归因。Python 线程/GIL 和系统调度未独立观测时写明限制。
不放宽现有接收连续性：GT/最终指令 0.2 s，odom 0.5 s；其他现有消息时间、新鲜度、终点和安全阈值照旧。
仿真时间戳连续不能豁免单调接收时间间断。接收序号连续不能证明发布端无丢包。不能把后台写盘延迟当成消息生成或接收时间。

5. 运行后独立审计
等待记录器关闭并退出后，计算原始数据和新健康/耗时文件哈希，重查输入内容与符号链接。执行：
python3 -B tools/summarize_provincial_full_observed.py --result-dir /home/rmnav/AeroMind/tools/results/provincial_stage15_full_capture_20260929 --output-name mission_verification_capture_v1.json
无论运行成功或失败都审核可用证据；文件缺失判证据不完整，不补成 PASS。
重新核验唯一外部目标与匹配接受、所有内部 waypoint 次序和坐标、GT/odom 误差和 yaw、完整 2+5 秒窗口、全程与终点的有限性/新鲜度/连续性、重试/卡住/false success、输入/参数/原始数据哈希及 writer health。
区分进程退出、导航完成、原始证据通过。不要手工覆盖任何 PASS 字段。

6. 原生 GT 间距
从本次全部原生 GT 重算全程矩形车体—墙采样间距，报告频率、最大仿真/单调接收间隔、最小值/时间/坐标/yaw/最近墙，以及是否小于 0.08 m。
实际经过 boundary_7 后，与旧 0.0844864394 m 和上次 0.0843167037 m 对照计划、参考线、setpoint、GT、正确坐标系速度及守卫状态，生成必要图表。未经过则 NOT_EXERCISED。
不得把离散采样升级为连续安全保证，不做统计重复性结论。无接触数据继续 CONTACT DATA UNAVAILABLE。

7. 收尾
聊天框明确本轮指令是否完成、Full/独立审计是否通过，列出接受延迟、内部事件、误差、2+5 秒、接收连续性、writer health/耗时、最小间距、失败分类、输入留档缺口及证据路径。
全部门槛通过才归档为暂定训练场可复核正向基线，并暂停扩展矩阵。失败保存现场，仅提出最小下一步；不得自动重跑、调导航或放宽阈值。
