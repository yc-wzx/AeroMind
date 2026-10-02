继续维护 /home/rmnav/AeroMind。

本轮目标：使用修复后的目标交接运行器，最多启动一次独立 Gazebo，只发送一个正向 Full 目标，验证目标接受、完整任务执行及原生 GT 的最小间距。

当前状态仍是 TRAINING-ONLY / PROVISIONAL、COMPETITION ARENA NOT VERIFIED。上一轮修复了运行器把自身订阅误当成导航就绪的问题；33 项工具/隔离测试已通过，包括真实生产导航节点的隔离目标接收。但没有证明之前 240 秒超时的唯一根因，也没有新的真实 Full 成功。保留历史结论和全部原始数据。

一、先读与核对

阅读 docs/provincial_stage15_goal_delivery_fix_20260929.md，核对 tools/results/provincial_stage15_goal_delivery_fix_20260929/fix_verification.json 及最终测试日志。
检查 git status、相关差异、source/install、残留 ROS/Gazebo 进程。不要回滚或覆盖任何历史记录。

保持 imperfect_sensors=false、GridRoute clearance=0.40 m、EGO inflation=0.22 m、车体、速度限制、geometry、PGM、collision，以及现有等待停车恢复逻辑。禁止改导航/控制/planner、最终速度、路线、安全阈值、物理步长、传感器频率；不运行 Return、A→B→C 或障碍测试，不进入 Stage 2。

二、运行前证据与门槛

使用新目录与 trial ID。运行前必须保存当前实际依赖内容、原始路径、符号链接目标、SHA-256、Git HEAD/dirty diff、ROS/Gazebo 版本、启动命令。
核对递归依赖闭包，至少包含导航接口、planar_navigation_simulator、scan_matcher、executor/adapter 及 YAML、GridRoute/几何、planner 二进制与配置、launch/world/PGM/地图 YAML/场地 JSON、运行器/记录器/评估器及直接导入模块。目前闭包为 42 项；以实际解析结果为准，不硬编码“42 即完整”。系统库未做内容快照时如实说明。
启动后先保存实际 ROS 参数并核对固定参数，再根据真实 GT 做起点/目标空闲、连通、GridRoute、参考段 clearance 与 yaw=90° 矩形间距检查。任一门槛不满足，不发送目标。

三、目标交接

仅使用修复后的运行器。不能用手工 ros2 topic pub 绕过就绪和接受确认。
记录唯一导航订阅端身份、GID、实际 QoS、匹配数量及稳定发现时间。记录器自身或其他观察者不算导航接收端。
只发送一次 (8.7,4.25,yaw=90°)，出生点约 (4.7,0.5,yaw=90°)。保留 /goal_pose 原始订阅证据。
发送后 5 秒内必须得到生产导航节点发出的、新鲜且 route/最终坐标匹配的接受事件。错误来源、旧 route、过期事件或其他目标不能确认本任务。拒绝或超时即保存数据、停止本次 Gazebo，不重发、不自动重启。本轮即使启动预检失败，也不要另开第二个 Gazebo。

建议在已 source ROS 与 install 的 shell 中执行一次：
python3 -B tools/run_provincial_full_observed.py --output-dir /home/rmnav/AeroMind/tools/results/provincial_stage15_full_delivery_20260929 --trial-id trial_full_delivery_01

目录已存在时停止并报告，不覆盖、不自动换目录重试。

四、记录与运行

保留终点评估 CSV 和每条实际接收消息的 native JSONL。至少包括 clock、GT 完整位姿速度、odom、原始/最终 cmd、外部/内部目标、route/waypoint 事件、诊断、setpoint、轨迹及可用 ID/起始时间。
保留消息时间戳、单调接收时间、接收时仿真时间、接收序号和原始字段。Twist 只有接收时间；不抽样、补零、插值或重复旧样本。报告实际频率、最大间隔、重复/乱序/非有限值及缺失话题；接收序号不是发布端丢包证明。
收到接受事件后，由原导航系统自行切换内部 waypoint。不得 set_pose、reset、清任务或外发中间目标。说明未独立监测的服务调用范围。
异常即保留现场并结束，不为了通过而改参数后再试。

五、独立审计

runner/启动器退出与导航完成、证据通过分别报告。无论运行成功与否，都尝试保存可用证据的独立审计；文件缺失时明确报告证据不完整，不能填补成 PASS。
运行结束后重新核对输入内容与符号链接目标；关闭原始文件后计算哈希。
使用新报告文件，不覆盖旧报告：
python3 -B tools/summarize_provincial_full_observed.py --result-dir /home/rmnav/AeroMind/tools/results/provincial_stage15_full_delivery_20260929 --output-name mission_verification_delivery_v1.json

从原始目标/接受消息重建交接，从 route 计划重建全部内部 waypoint 顺序及完成事件；复核 GT/odom 到点误差、实际/目标 yaw、完整 2+5 秒窗口、GT/最终指令的新鲜度连续性有限性、重试/卡住/false success、输入和数据哈希。
从本次所有原生 GT 重算车体—墙全程采样最小间距，不能只看 CSV 或旧最窄点。报告频率、最大间隔、最小值、时刻、GT、yaw、最近墙体以及是否低于 0.08 m。
若实际经过 boundary_7，再对照旧 Full 的 0.0844864394 m、计划/参考线偏差、setpoint、GT、机体和世界系速度、守卫及轨迹切换；生成必要的局部图和时间曲线。未到达该区域写 NOT_EXERCISED，不能用起点间距代替。

六、收尾

聊天框明确说明本轮指令是否全部完成；给出唯一试验、接受确认延迟、内部事件、到点误差、2+5 秒、原生频率/最小间距、输入留档缺口、失败分类和全部证据路径。
通过才归档为当前暂定训练场可复核正向基线，并暂停扩大矩阵；失败只给出最小下一步，不自动修导航或重跑。
保持 CONTACT DATA UNAVAILABLE（无接触数据时）。不把离散几何间距当作接触检测或连续时间安全保证，不宣称统计重复性，不视作正式省赛验收，不进入 Stage 2。
