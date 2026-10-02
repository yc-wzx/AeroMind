继续维护 /home/rmnav/AeroMind。

本轮目标：仅一次受控短距离Gazebo扫描断流—停车—恢复验证，确认新Stage2输入门控可以在真实运行中保护原任务。不运行Full/Return/ABC，不自动重跑，不接入MID-360/IMU/LIO。

阅读 docs/stage2_scan_input_guard_and_clearance_20261001.md，核对 tools/results/stage2_scan_guard_20261001/verification_v1.json、final_evidence_seal.json、tests_final_v3.log、counterexample_before/after.json、三个contract_audit_v5.json及narrow_clearance_review_v2.json。

当前仅离线/隔离完成，live_new_guard_verified=false。不得把历史Full PASS当成本轮新门控已实跑。
保持TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED。

固定 imperfect_sensors=false、GridRoute clearance=.40m、EGO inflation=.22m、间距阈值.08m、原误差注入模型/车体/速度/几何/物理步长/传感器发布频率。保留Stage1.5 92项封存输入与全部历史证据。不改planner/executor，不降阈值，不用GT作定位校正，不增路线特殊逻辑。

先检查git/source/install/残留进程及本轮代码哈希，完成隔离测试和离线合法性检查，再启动。新结果目录/trial ID必须不存在，最多一次独立Gazebo；启动或预检失败也不启动第二次。

允许新增仅用于试验的激光中继与独立launch：订阅真实/scan，将原始消息内容和原消息时间戳原样转发到独立/guarded_scan，只有新增Stage2定位接口使用该输入。禁止修改world或真实传感器频率；中继暂停时丢弃消息，不缓存回放，不伪造扫描。中继服务/状态日志记录暂停与恢复原因和时间。
保存原始/scan、实际/guarded_scan、scan门控/校正诊断和本轮实际节点图与端点，证明定位入口不会绕过中继读/scan。

使用zero误差profile，独立初始(4.7,.5), yaw90°，唯一外部目标(4.7,1.15), yaw90°。
以真实GT做只读起点/终点、连通性、clearance和矩形检查，保留Goal Guard和真实导航接收端/GID/QoS/新鲜接受事件。只发一次目标，不外发中间目标，不set_pose/reset或清除状态。

在任务被接受、最终输出与GT实际处于前进状态、且距离终点仍足够时暂停中继。先离线估算0.5s超时加响应期间的运动空间，避免在终点附近才触发；优先在短路段前半程触发。
记录从最后接受扫描到最终零速度输出及GT停车的时间、最大位移、静态几何间距。现有门槛不变；预期超时停车约0.5s加实际控制周期/时序裕度，不能凭假设代替实测。
稳定停车后至少观察2个仿真秒，再恢复转发新鲜扫描。旧消息不得回放，旧/错误扫描不能恢复运动。
观察原任务、原route/waypoint能否安全恢复，恢复时仍须有效新鲜轨迹和原安全守卫成立。终点仍满足完整2+5秒判据。

若目标先完成、未真实断流或没有触发停车，标NOT_EXERCISED；不重跑寻找一次通过。异常或不安全输出先保存证据、停止本次试验，不自动调参。

目标发送前封存实际依赖内容/符号链接/SHA、Git HEAD/dirty diff、启动命令、版本、实际参数与雷达SDF约定。结束复核输入、关闭文件后计算原始哈希。
保留原生逐消息GT/注入odom/校正odom/ground odom、clock、原始/最终速度、goal/内部目标/route事件、计划/setpoint、扫描原始及中继、暂停恢复状态、录制器health/timing。
独立审计原始消息与完整waypoint/终点，不凭runner返回码判断。连续性阈值保持GT/最终指令.2s、odom.5s；中继话题的故意断流单列，不能豁免其他关键记录中断。

最终聊天框明确本轮是否完成、是否实际触发停车和恢复、延迟/位移/最小间距、终点误差和2+5秒、输入哈希、失败/未覆盖项及数据路径。无接触数据写CONTACT DATA UNAVAILABLE。
不宣称毫米余量已稳健、统计重复性、连续安全或正式省赛验收；硬格式/错误frame的live覆盖若未测试，仍明确未覆盖。
