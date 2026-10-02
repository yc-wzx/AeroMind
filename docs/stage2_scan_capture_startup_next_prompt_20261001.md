继续维护 /home/rmnav/AeroMind。

本轮目标：只离线修复短距离断流试验的采集启动同步和审计入口，保留已证明的停车/恢复与零重试行为。不要启动Gazebo，不发送导航目标。本轮不进行Full、Return、A→B→C或硬件接入。

状态：TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。

一、先核对
阅读 docs/stage2_localization_hold_retry_live_20261001.md，以及上轮fix文档。
核对 tools/results/stage2_scan_resume_retry_fixed_20261001/ 的 independent_audit_review_v3_final.json、repeat.json、input_manifest/raw哈希、offline_analysis/analysis.json，以及 tools/results/stage2_scan_resume_retry_fixed_tools_20261001/ 的final_validation、final_evidence_seal、tests_audit_v3_final.log、missing_associations、报错栈。
检查git/source/install/残留进程，不覆盖、回滚或删历史工作。核验Stage1.5封存92项、上轮修复及本轮源码/证据封存。

二、明确结论边界
真实试验仅一次Gazebo、一次外部目标、一次内部目标：断流停车、保持原任务、恢复、零retry/stuck/false success、完整2+5秒均有证据通过。
总结果仍FAIL：143条accepted诊断中，stamp14.0–14.8的9条启动诊断找不到原始/guarded scan；两扫描话题从14.9起记录。9条全在发目标前，行驶期129次匹配可完整重算。
旧试验retry=1 FAIL和本次startup evidence FAIL全部保留，不能改报告、删除缺失诊断或把历史改成PASS。不能断言DDS发现或QoS是唯一原因。

三、固定范围
保持imperfect_sensors=false，GridRoute=.40m，EGO inflation=.22m，gap=.08m，zero profile，以及全部车体/速度/几何/地图/collision/物理步长/传感器频率/扫描校正及导航参数。
不改planner、控制、任务计时修复、最终速度或真实定位接口。只允许测试采集器、运行编排、只读评估器及针对性测试的必要修改。

四、先离线复现采集启动漏洞
用原始数据临时副本或内存副本复现：
1. 首clock前无header Twist/String receive_sim_s=None造成旧loss_audit TypeError；
2. accepted诊断先被记录、双扫描流尚未就绪时，旧运行器仍可进入目标交接；
3. 一条已accepted诊断缺原始/转发扫描应维持FAIL，不能被发目标前例外抹去。
保留反例与实际生产工具调用证据，不只测复制的判断表达式。

五、修采集启动顺序，不修导航
检查 ScanResumeRunner、NativeTrace、订阅创建/发现顺序以及运行器启动时点。设计有证据的采集就绪门槛：所有关键订阅已建立，clock/GT/odom/最终输出以及原始+guarded扫描都有新鲜有限原始数据，双扫描时间戳/内容对应，真实发布端/GID/QoS与计数连续性明确。不能只增加sleep或调ROS队列/发布频率来掩盖。
只给发目标加ready门槛仍不能补回已经保存的无原始扫描accepted诊断。若要完整证明扫描校正输入，必须保证采集订阅在定位节点开始接受扫描前已就绪；可在专用试验编排中拆分采集和节点启动门控，保持默认导航launch/算法不变。不得删除初始诊断、过滤缺失数据或重写时间戳。不要为证明设计而在本轮启动仿真。
工具启动/关闭职责明确：独立原始记录器先启动、运行器复用记录文件，不能双写同一文件；就绪失败只保存数据和停止，不发目标、不重启。对初始状态变换的证明范围如实说明，不伪称封存整个节点未观测生命期。

六、整合只读v3审计
保留v1/v2原文件及旧输出。必要参数化以使用新v3入口，但不得改重试、扫描关联、有限性/新鲜度/连续性、2+5秒和间距阈值。
v3首clock前时间关联处理必须局限于真实尚未收到clock的无header连续前缀：不豁免字段NaN，不豁免首clock后任何缺值，不给GT/odom填零，不制造Twist生成时间。
明确区分clock/state诊断与accepted/rejected扫描；核验任务身份与原任务不重发。新汇总缺文件/缺扫描/顺序错误均不能PASS。
运行器自动审计与独立命令实际使用的版本/哈希要一致；报告区分运行时工具和事后评估器，不能沿用v2对旧试验写死的溯源文字。

七、针对性隔离测试
至少测试：
- 采集先就绪、双扫描+odom+诊断对应后，才允许导航节点开始并进入目标就绪；
- scan/guarded/clock/GT/odom/最终输出之一未就绪，不发目标；
- 观察者冒充真实发布端或GID/QoS异常不ready；
- accepted诊断缺任一对应扫描，不允许归档为完整证据通过；
- 首clock前无header记录可明确分类，原始字段/行数不改；
- clock后缺时间、行驶期NaN、关键断流、停车窗速度超限均失败；
- task_clock不能计作scan，错route/waypoint/顺序/时长失败；
- 队列溢出/关闭失败、缺raw文件、输入或哈希不一致失败；
- 唯一目标/不重发/进程启动失败不重启的约束仍有效。
直接覆盖实际生产工具入口；所有ROS/Gazebo启动和发布在隔离测试中mock，不向运行链路发布任何数据。
对现有数据再次运行新版本只读汇总，应保留同样两个严格FAIL门槛、行为通过与证据缺口的区别。

八、归档与下一轮
使用不存在的新工具结果目录，例如 tools/results/stage2_scan_capture_startup_fix_20261001/；封存修改前后内容、工具依赖、测试日志、反例和独立复核，不覆盖本次数据。
输出 docs/stage2_scan_capture_startup_fix_20261001.md 和下一轮“最多一次独立Gazebo、一个相同短目标”的详细prompt，只生成prompt，不在本轮执行。
下一轮仍沿用(4.7,.5)→(4.7,1.15)、yaw90、断流后稳定停车2秒、恢复原任务零重试及完整2+5秒；新目录/trial唯一，任何启动/预检/接受/运行失败不重跑。必须能封存全部实际本地输入并证明采集启动同步，不可把新留档补作历史证明。

聊天框明确本轮指令是否完成、修改文件/是否影响导航、反例和测试结果、历史FAIL保留、是否仍有无法离线证明的启动顺序、报告/证据路径和下一轮prompt。
只有工具与隔离检查全通过才能称采集启动工具修复完成；没有新live数据，不能写定位断流严格live验证完成。不要把此轮扩展为完整Stage2完成。
