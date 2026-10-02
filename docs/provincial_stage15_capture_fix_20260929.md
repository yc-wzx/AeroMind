# Stage 1.5 采集阻塞风险修复与接收连续性审计（2026-09-29）

本轮工具修复与离线验证完成，55 项测试通过；没有启动 Gazebo，也没有向生产导航发送目标。真实 Full 是否恢复仍未验证，旧 Full 的 FAIL 保留。

状态：TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。

## 已证实问题和限制

旧 Full 在 sim_t=9.00→9.02 s 的两条 GT 之间出现 0.343483230 s 的单调接收时间间隔。同一时期最终指令约 0.340920 s、odom 约 0.360768 s、clock 约 0.335733 s。仿真时间戳连续，不能因此豁免真实接收间断。GT 和最终指令分别超过既有 0.2 s 阈值；odom 仍在其既有 0.5 s 接收阈值内。

旧 NativeTrace.record() 在 ROS 回调中执行 JSON 序列化和同步写盘。隔离测试直接执行封存的旧生产类，注入阻塞文件流，确认回调随写盘一起阻塞。新类在同一阻塞条件下仍能提交后续原始消息，接收时间保持回调入口的真实单调时钟值。

这证明同步写盘是可消除的阻塞来源，但不能证明它就是旧试验停顿的唯一原因。旧记录未包含回调、节点图查询及磁盘写入耗时。运行循环每次 spin 后查询节点列表也可能耗时；本轮没有凭猜测修改节点图监测策略，新增了逐次查询耗时。JSON 转换仍在接收回调，Python 线程仍共享 GIL；后台写盘不是系统级实时性保证。

独立 native_audit 原来主要检查消息仿真时间间隔，没有独立强制原生接收时间连续性。旧顶层报告虽然通过其他 gate 拒绝了整次试验，其 native_record_pass 仍为 true。本轮补齐这个独立检查，新 native_record_pass=false，整次结论仍为 FAIL。

## 修改

- tools/provincial_native_trace.py：有界后台队列（8192 项），JSON 序列化和写盘移出接收回调。仍逐条保存消息、不抽样、不补零、不替换接收时间。队列溢出、写盘/flush 失败、关闭超时、接收/写入计数不符都标记失败；排队写盘延迟与实际接收延迟分开记录。
- tools/run_provincial_low_speed_validation.py：对每个订阅回调和节点图查询记录单调起止时间；保存记录器健康结果，健康失败不能得到 terminal_pass。
- tools/run_provincial_full_observed.py：原始数据关闭后的哈希范围包含新增 .native.health.json；.native.timing.jsonl 随其他 JSONL 一并封存。
- tools/summarize_provincial_full_observed.py：原生接收间隔独立检查，GT/最终指令 0.2 s、odom 0.5 s；新版本输入快照必须对应完整的健康与耗时文件，核对逐话题实际行数、收到/写出计数、关闭状态与错误。历史版本明确标记 LEGACY_NO_WRITER_HEALTH，不伪造历史健康证据。
- tools/test_provincial_native_trace.py：17 项记录器和原生连续性测试；与原有交接、终点及 waypoint 测试合计 55 项通过。

没有修改导航接口、executor、planner、场地或任何安全/终点判据。运行依赖中的 31 项 src/install 文件与上轮快照一致。原始试验目录现有 70 个文件哈希均未改变。没有回滚或清理原有 dirty 工作。

## 验证

最终日志：tools/results/provincial_stage15_capture_fix_20260929/tests_final.log。

覆盖旧生产写盘阻塞、新生产类在慢盘下接收不阻塞、2000 条原始消息和序号完整保留、无时间戳 Twist、不填补 NaN、队列溢出、写盘失败、关闭超时、丢失健康/耗时证据、实际行数不符，以及仿真时间戳连续但接收间隔超限的反例。原有目标端点/QoS/接受确认、完整 2+5 秒、缺值与 waypoint 次序测试继续通过。

前两次测试命令分别因 shell 环境中的 PYTHONPATH 和隔离 ROS 域设置不正确而报错，日志保留为 tests_v1.log/tests_v2.log。按测试要求 source ROS/install、在 tools 目录使用 ROS_DOMAIN_ID=187、ROS_LOCALHOST_ONLY=1 后通过。最终 55 项日志为 tests_final.log。没有在生产域发布测试目标。

新版本历史复核：tools/results/provincial_stage15_capture_fix_20260929/historical_full_reaudit_v2.json。all_pass=false，GT/最终指令原生连续性=false；最小几何间距仍为 0.0843167037 m，位置 boundary_7。完整终点窗口自身通过的结论保留，不抵消全程接收间断。

fix_verification.json 保存工具哈希和保护文件核验；changes.patch 是相对于本轮开始的工具差异；before/ 保存修改前工具；gap_localization.json 保存缺口前后原始行。旧失败不改写，历史数据不补造。

## 下一步

最多一次新独立 Gazebo Full，用本轮记录器及审计器验证。启动前重新封存全部实际输入，闭包以实际解析为准，不能沿用旧 42 项哈希作为本轮留档。运行后必须独立审核健康文件及所有原始行数，并利用 callback/ros_graph_query 耗时、队列等待与写盘耗时分析任何新间断。

若再次间断，保存 FAIL 与现场，不重跑、不放宽 0.2 s 连续性阈值。若耗时记录覆盖了停顿，可据其限定具体工具修复；若没有覆盖，则明确系统调度、发布端或接收调度仍未定位。下一次 Full 才能检验实跑恢复；本轮不能宣称其已恢复。
