继续维护 /home/rmnav/AeroMind。

本轮目标：使用定位失效任务计时最小修复后的Stage2接口，最多启动一次独立Gazebo，仅发送一个短距离目标，复核扫描断流—停车—恢复、零内部重试及完整2+5秒终点证据。不自动重跑。

状态保持TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；没有接触数据时CONTACT DATA UNAVAILABLE。

一、核对基线
阅读 docs/stage2_localization_hold_retry_fix_20261001.md 与 docs/stage2_scan_loss_resume_live_20261001.md。
核对 tools/results/stage2_localization_hold_retry_fix_20261001/ 下的 fix_verification.json、final_evidence_seal.json、counterexample_before.json、counterexample_after_final.json、tests_combined_final.log、test_input_sha256_before.json、historical_reaudit_v3.json、unsafe_plan_review_v1.json、changes.patch。
77项隔离/工具测试通过不是新代码live通过。旧单次实跑retry_count=1的严格FAIL保留，不改历史数据/报告/快照/哈希。
检查git/source/install和残留Gazebo/ROS进程，保留其他dirty工作。Stage1.5的92项封存输入必须原样匹配。运行前使用新输出路径再次执行只读修复verification，不能覆盖fix_verification.json。

二、固定约束
保持imperfect_sensors=false、GridRoute=.40m、EGO inflation=.22m、采样间距判据=.08m、zero误差profile、现有车体/速度/geometry/PGM/collision/物理步长/传感器频率/导航与扫描校正参数。
不修改planner、executor、最终速度公式或阈值，不修改路线/场地来通过，不接入MID-360/IMU/LIO，不用GT反馈定位或控制。保留已有输入保护、等待停车、有效轨迹和矩形安全检查。
最多一次Gazebo、一次外部目标；启动、预检、接受确认或运行失败都不得再次启动/重发目标。允许必要只读记录/独立审计适配，不能在失败后改导航再跑。

三、指定一次试验
新目录：/home/rmnav/AeroMind/tools/results/stage2_scan_resume_retry_fixed_20261001
trial ID：stage2_scan_resume_retry_fixed_01
目录已存在即停止报告，不覆盖、不自动改名。
目标发送前封存实际本地运行依赖内容/路径/符号链接/SHA、Git HEAD/dirty diff、ROS/Gazebo版本、完整命令、实际参数和雷达SDF约定。包含新Stage2接口、relay、运行器、v2审计及本地导入闭包；按当前内容计算，不沿用旧哈希。系统依赖未完整封存则明确说明。

在已source的shell中只执行一次：
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/run_scan_resume_experiment.py \
  --output-dir /home/rmnav/AeroMind/tools/results/stage2_scan_resume_retry_fixed_20261001 \
  --trial-id stage2_scan_resume_retry_fixed_01

初始约(4.7,.5)，yaw90°；唯一外部目标(4.7,1.15)，yaw90°。启动后据真实GT只读检查起点/目标空闲、静态连通、GridRoute、参考段clearance及90°矩形间距。任一失败，不发送目标，本轮不再启动第二个Gazebo。
检查真实导航目标接收端/GID/QoS/匹配及稳定时间，发布一次后5秒内必须确认源自导航节点、新鲜且本route/最终坐标匹配的接受事件。观察者不能冒充接收端。旧接受事件不能确认此次任务。
扫描图必须证明导航只读/guarded_scan、不绕过中继读/scan；/guarded_scan唯一发布端为relay，原始/scan是真实bridge。原始scan内容和时间戳原样转发，暂停时丢弃，不缓存回放、改写或伪造。

四、实际占用本次门控
保持已验证的前半程触发：任务接受、有active waypoint、最终指令及GT确实运动、GT y约.60–.70且离目标至少.35m时暂停relay。离线确认原.5s超时及响应停车空间足够。
记录末次接受扫描、最终精确零指令、GT停车时间，分别报告仿真与单调接收时间；报告最大停车位移和最小静态几何间距。
稳定停车至少2个仿真秒后恢复，只转发新扫描。保持原route/waypoint，不能set_pose/reset、清任务或外发中间目标。
必须记录新增localization_task_clock诊断：paused/resumed、任务身份、计时转换与hold时长。等待期间不得内部重试、误完成、跳阶段或重发原目标；恢复前错误/旧数据不能解除停车。恢复后仍须有效定位、新鲜安全轨迹和原守卫才允许运动。
若未真实触发断流/停车，或目标先完成，写NOT_EXERCISED，不能算PASS，也不能重跑。

五、逐消息原生记录
保留GT、注入odom、校正odom、ground odom、clock、原始/最终速度、唯一外部goal、内部目标、route/waypoint事件、plan/Bspline及trajectory ID/start_time、setpoint、actuation、原始/guarded scan、relay状态/服务请求与确认、localization_task_clock及校正诊断。
保留每条真实消息的header stamp（若有）、单调接收时间、接收时仿真时间、序号与字段，以及native.health.json/native.timing.jsonl。Twist没有生成时间戳，不伪造。
GT/最终指令最大接收间隔.2s、odom .5s不变。故意guarded scan断流单列，不能豁免其他关键断流。接收/写入/文件行数一致、无队列溢出、关闭失败；不抽样、补零、插值或重复旧样本。

六、独立审计
关闭原始文件、结束Gazebo后复核输入内容/符号链接，计算原始数据哈希，保存健康和耗时文件哈希。
使用tools/stage2/audit_scan_resume_v2.py对新目录重新计算，指定不存在的新报告路径。启动器自动报告之外再只读复核一次：
python3 -B tools/stage2/audit_scan_resume_v2.py \
  --result-dir /home/rmnav/AeroMind/tools/results/stage2_scan_resume_retry_fixed_20261001 \
  --output /home/rmnav/AeroMind/tools/results/stage2_scan_resume_retry_fixed_20261001/independent_audit_review_v2.json

必须核验唯一外部目标及源匹配接受、全部waypoint顺序与坐标、原任务恢复、GT/odom误差/yaw、零retry/stuck/false success、完整前2秒+后5秒原始停车窗口、关键数据有限/新鲜/连续、原生GT全程矩形间距≥.08、输入/参数/数据哈希。
不能移除no_retry判据或修改记录器统计来通过。区分进程结束、导航到点、守卫触发、严格证据PASS。
新增task_clock事件是状态诊断，不是接受扫描；不能把它计为扫描校正或用它刷新定位。必要时只修正事件分类统计，保留所有原始记录、阈值和失败。
如再次遇到unsafe_plan，保留零输出拒绝；离线关联真实Bspline形状/ID与Marker，区分上下文yaw重放及原导航内部状态。不得为通过调整planner或放行风险轨迹。
如出现不安全输出/几何门槛违反，立即终止本次试验并保存证据；不改变代码后重跑。

七、收尾
聊天框明确本轮指令是否完成、是否真实门控停车恢复、等待/停车延迟与位移、任务计时转换、内部重试次数、终点误差/完整2+5s、原生频率/最大间隔/最小间距、输入哈希/剩余缺口、失败与未覆盖项、结果路径。
仅全部门槛通过才写STAGE2 LOCALIZATION-HOLD RETRY LIVE VALIDATION COMPLETE；失败保留FAIL，仅提出最小下一步，不自动扩展矩阵。
错误frame硬格式的live覆盖未执行则继续写未覆盖；不声称连续时间安全、统计重复性、实际接触检测通过或正式省赛验收。不要自动执行Full/Return/ABC。
