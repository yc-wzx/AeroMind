# Stage 1.5 目标交接工具修复（2026-09-29）

本轮工具修复及隔离验证已完成；**真实 Gazebo Full 恢复尚待下一轮验证**。没有启动新 Gazebo、运行新 Full、向常规 ROS 域发送目标或改变任何导航/控制/场地参数。状态保持 TRAINING-ONLY / PROVISIONAL、COMPETITION ARENA NOT VERIFIED、CONTACT DATA UNAVAILABLE。

## 已确认的问题

对修改前的实际 `Runner.wait_ready()` 做隔离调用，GT/odom/最终零指令新鲜、planner 名称存在、但 `/goal_pose` 只有记录器自身订阅时，方法错误返回 True。修复前代码、哈希与反例记录保存在 `tools/results/provincial_stage15_goal_delivery_fix_20260929/before/`、`before_sha256.json` 和 `before_self_subscription_reproduction.json`。

该漏洞可以导致没有确认导航接收端就绪就发目标。上轮 Full 没有录下发送瞬间的端点匹配信息，故不能证明它是历史 240 秒超时的唯一原因，也不能把上轮失败改写为通过。

## 本轮修改

1. `tools/run_provincial_low_speed_validation.py`：目标发送前要求唯一的 `/gazebo_navigation_interface` 订阅端，检查消息类型、实际 DDS 端点 QoS、匹配数及至少 0.5 秒的稳定发现状态；记录器或其他观察者不能代替导航端。实际 DDS QoS 用发布/订阅两侧发现结果比较，避免把未解析的 system-default 配置当作已解析 QoS。
2. 同一运行器：只发布一次目标；保存发送时间、端点 GID、就绪证据与确认记录。5 秒内必须收到导航 logger 发出的、新鲜且最终坐标匹配的完整 route 接受事件；排除已知旧 route、错误来源、旧时间戳、错误坐标和过期确认。收到拒绝或确认超时即保存失败，不自动重发。上层单次 Full 启动器随后结束本次 Gazebo。
3. `tools/run_provincial_full_observed.py`：新增必填 `--output-dir`、`--trial-id`，仍拒绝覆盖已有目录。递归收集项目内 Python 导入，当前计划留档 42 项；相较上轮补上离线墙体解析、导航继承类、scan matcher 的源文件及对应安装文件共五项。它们作为依赖存档，不启用新的扫描匹配或定位链路。运行后重新检查文件内容和符号链接解析目标。runner 失败会使启动器返回非零。
4. `tools/summarize_provincial_full_observed.py`：支持指定结果目录、从 manifest 读取 trial ID 和只读 `--verify-only`；检查新增端点/确认信息、原始 `/rosout` 中的接受消息与发送时间，不能只信 summary；同时核对项目内依赖闭包、运行参数哈希、运行前后解析路径。旧失败仍保持 FAIL，缺少新字段明确判为证据不完整，不回填。
5. 新增 `tools/test_provincial_goal_delivery.py`；调整 `tools/test_provincial_forward_evidence.py` 的只读复核测试，允许当前工具与历史输入不同，但要求准确报告该差异，历史原始任务证据仍通过，且两次结论一致、旧报告未改。没有放宽实际审计阈值。

本轮未修改 `provincial_native_trace.py` 的逐消息采集方式，也未修改任何导航最终输出代码。

## 验证与边界

33 项测试通过：11 项目标交接测试、10 项历史 waypoint/只读复核测试、12 项终点评估测试。具体包含记录器自订阅、无关观察者、QoS 不兼容、重复导航端点、延迟发现、错误/陈旧/过期确认、静默接收端超时且只发送一次，以及失败返回码。

两个真实 DDS 隔离测试都限定 `ROS_DOMAIN_ID=187`、`ROS_LOCALHOST_ONLY=1`，没有 Gazebo：一个接收桩延迟出现后收下唯一目标并发出匹配确认；另一个直接启动实际 install 下的 `gazebo_navigation_interface.py`，由测试夹具提供静止的合成 GT 与 clock，生产导航节点成功接收目标并产生 `R0001:W00 → W01 → W02` 的 route 计划。独立审计能从原始日志验证这次接受；删除临时记录中的接受事件后不能通过。planner 仅有名称占位节点，未进行真实规划或运动；测试输出中的静止 odometry 是合成夹具，不能当作 Gazebo truth 或 Full 成功。这些测试只证明目标交接和失败处理。

保全检查 `fix_verification.json`：27 个已记录导航/配置/场地/安装输入与上轮留档一致，42 个历史原始数据文件仍匹配原哈希。旧 Full 为 FAIL，旧 A/B/C/Full 成功试验的原始事件和终点结论保留。系统 ROS/Python/Gazebo 库不是完整系统镜像，仍按已记录版本与明确限制描述。

## 下一轮

只授权一次独立 Gazebo 单目标 Full，先离线确认工具和 42 项依赖留档，再按实际位姿预检；收到匹配接受确认后才进入正常导航评估。没有确认、被拒绝、输入缺口或安全异常时保存证据并结束，不重复发送、不再启动。只有完整 waypoint、2+5 秒终点、原生 GT 最小间距及输入审计全部通过，才归档为暂定训练场可复核正向基线。上轮 boundary_7 的 0.0844864394 m 仍须实际通过该位置后才能对照。

证据目录：`tools/results/provincial_stage15_goal_delivery_fix_20260929/`。最终测试日志为 `test_goal_delivery_final.log`、`test_forward_evidence_final.log`、`test_terminal_evidence.log`；中间失败日志也保留。没有启动器、Gazebo 或生产导航残留进程。
