继续维护 /home/rmnav/AeroMind。

本轮目标：只做 Stage2 定位失效等待期间内部重试计时的隔离复现与最小修复；不启动 Gazebo，不发导航目标，不扩展功能。

先阅读 docs/stage2_scan_loss_resume_live_20261001.md，核对 tools/results/stage2_scan_resume_20261001/independent_audit_v2.json、independent_audit_v2_repeat.json、hold_stimulus.json、原始 native/events/actuation/plans、input_snapshot 和 offline_analysis_v2/analysis.json。

接受并保留结论：唯一一次短距离试验实际触发扫描缺失、最终零输出、稳定停车2s、新扫描恢复原任务，以及完整2+5s终点停车；但内部 retry_count=1，严格整体 FAIL。不要把守卫验证通过升级为整个试验 PASS。v1 的短目标/Full 默认坐标问题已由 v2 参数化纠正，旧报告和原始数据保留。

当前 TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。
保持 imperfect_sensors=false、clearance=.40、EGO inflation=.22、间距门槛=.08、现有车体/速度/geometry/PGM/collision/物理步长/传感器频率与误差模型。禁止planner、executor或最终速度输出调参，禁止MID-360/IMU/LIO，禁止GT反馈校正。

一、核对封存
检查 git status、source/install、上一轮证据seal、残留进程；保留全部dirty工作。Stage1.5的92项封存输入必须原样匹配。先保存本轮相关代码副本、哈希和差异，禁止覆盖旧数据、报告或哈希。

二、先复现生产逻辑
实际失败：sim≈18.56s，中继仍hold，导航处于stale_stop，R0001:W00内部重发；18.66s才resume。最终速度始终零，非用户第二次外发目标。
检查继承的update()中progress/retry分支和observe_guard_resume()。隔离执行实际生产方法，不只测试复制的布尔表达式。构造已有active waypoint、任务接受超过4s、无进展超过2s、扫描失效且存在新鲜非零上游指令；保存修复前内部发布、重试计数、计时字段、最终输出和route/waypoint身份。
不向正在运行的ROS链路发布目标或速度。

三、最小Stage2修复
只允许在新增Stage2接口层解决定位失效期间的任务进度/内部重试计时问题及必要只读诊断。不得改Stage1.5封存的gazebo_navigation_interface.py、planner、executor、地图或参数；不能通过删除日志、改retry_count统计或豁免零重试判据制造成功。
定位不新鲜/硬输入故障期间，原任务原route/waypoint保持，最终仍由现有守卫保证零；该保护等待不应消耗“可执行运动”的无进展/重试计时。
有效新扫描恢复后，必须重新满足有效定位、轨迹新鲜和现有安全检查才允许运动；不能放行旧轨迹、缓存扫描或错误frame。恢复后真正的持续无进展仍应按既有门槛触发重试，不能永久关闭失败恢复。
处理初始idle、已完成、多个内部waypoint、新目标替换、重复stale/resume；不得恢复旧任务计时到新route，不能改完成事件坐标、速度/距离阈值或任务身份。
如果实现无法同时保持92项Stage1.5输入且满足上述条件，报告具体冲突，不修改封存基线。

四、隔离测试
至少：失效等待跨越4s仍不内部重发；新鲜非零旧上游指令最终为零；恢复前/错误scan不能解除停车；有效定位但没有安全新轨迹仍停；合法新轨迹后正常执行；恢复后的真实无进展可按原条件重试；同route连续失效恢复不重复累加计时；任务完成、idle、目标替换和waypoint切换不被错误污染。
直接覆盖实际生产update()/相关回调，记录实际内部发布而非仅mock判断值。用可控隔离时钟，不更改生产时钟或阈值。保持已有扫描metadata/frame、时间关联和完整2+5s审计反例测试。

五、现有证据与unsafe_plan
对已封存的一次试验做只读复核，不把新代码补算哈希写为当时运行输入，不改历史retry_count=1或FAIL。
恢复后19.58s计划延伸至y≈1.947，守卫planned gap=0并停车，而实际GT最小gap≈.188906；保留该拒绝。只补足计划坐标/轨迹ID/时间与guard对应的诊断，区分上下文yaw采样与生产未来yaw模型。不因计划超出短目标自动改planner或绕过守卫；证据不足时只记录未知。

六、收尾
必要时只build新增Stage2安装目标，前后核对source/install与Stage1.5的92项。保存修复前后反例、测试日志、代码差异、新评估器哈希和新增报告。
聊天框明确本轮是否完成、改动是否影响Stage2任务计时/速度输出、反例修复前后结果、旧FAIL为何仍保留、尚未实跑的项、原始证据路径。
本轮通过只能写STAGE2 LOCALIZATION-HOLD RETRY ISOLATED FIX VERIFIED，不能写新的live PASS。
最后生成下一轮“最多一次短距离同路线scan断流—停车—恢复实跑”的完整prompt，但本轮不要执行它，也不要自动启动Full/Return/ABC。
