继续维护 /home/rmnav/AeroMind。

本轮目标：补齐原生 50 Hz 数据的完整 2+5 秒终点评估，完成当前暂定训练场正向 Full 基线的离线封存。本轮不启动 Gazebo、不发送目标、不重跑 Full、不扩大测试矩阵。

一、范围和状态
保持 TRAINING-ONLY / PROVISIONAL、COMPETITION ARENA NOT VERIFIED、CONTACT DATA UNAVAILABLE。
保持 imperfect_sensors=false、GridRoute clearance=0.40 m、EGO inflation=0.22 m，以及当前导航、控制、速度、车体、geometry、PGM、collision和等待停车/恢复逻辑。
仅允许评估器、必要针对性测试和新增审计/基线文档的修改。不修改记录器、导航输出、planner或参数；不进入 Stage 2。
保留所有原始记录、报告、输入快照及哈希，反例只在副本制作。

二、阅读与核对
阅读：
docs/provincial_stage15_full_capture_20260929.md
docs/provincial_stage15_full_capture_review_20260930.md

核对：
tools/results/provincial_stage15_full_capture_20260929/
tools/results/provincial_stage15_full_capture_review_20260930/
尤其 native_terminal_readonly_v1.json、counterexample_mutation.json、counterexample_audit_v1.json 和 counterexample_native_speed/。

原始 Full 实际到点、所有 waypoint、完整原生停车窗和采样间距仍通过。
纠正统计口径：原自动汇总顶层检查15项；完整原生窗口351条GT，最大速度0.0190720623 m/s，最大目标误差0.0178260992 m，最大停车漂移0.0000759606 m。后5秒GT速度为零，最终指令全窗为零。
旧报告的0.011394 m/s和零漂移来自CSV，不能当成原生全窗最大值。

三、先复现已知漏判
在反例副本中，仅 sim_t=76.28 的一条原生GT速度被改为0.1 m/s，副本native数据哈希已更新；CSV未改。当前 summarize_provincial_full_observed.py --verify-only 仍 all_pass=true。
先保存修复前结果，不改原文件和旧报告。

四、最小修补独立评估器
在 tools/summarize_provincial_full_observed.py 的实际总汇总路径中增加原生终点审计。
CSV审计保留，但总PASS必须同时通过原生审计，不能只计算诊断字段。
从本次原始route/最终waypoint完成事件、GT、最终指令和clock证据核对停车窗边界及归属。summary只提供候选窗口，不作为通过证明。
分别重建前2秒、后5秒及完整7秒，检查：
- 原始覆盖范围、单调性、连续性、有限性和新鲜度；
- 每条GT平移/角速度，每条直接订阅最终平移/角速度；
- 目标误差、相对完整停车窗锚点的漂移；
- 终点快照与最后原始GT、最终指令的一致性；
- 唯一外部目标、接受事件、全部waypoint及最终完成事件归属。

保持现有阈值：GT/最终指令平移停车速度0.02 m/s、角速度0.03 rad/s、目标误差0.10 m、停车漂移0.03 m；接收连续性GT/最终指令0.2 s、odom0.5 s，其他现有判据保持。
无时间戳Twist使用接收时间及可核验clock关联，不能伪造生成时间。缺失、NaN、陈旧、不一致、断流不能补零或PASS；不得把CSV覆盖视作native覆盖。
不得为了反例失败而重新选择一个更晚的窗口隐藏违规，必须验证本次声明的窗口。

五、针对性测试
直接覆盖实际生产汇总入口。至少验证：
1. 原始trial_full_capture_01通过；
2. 已保存的76.28原生GT超速反例失败；
3. 后5秒仅native GT超速失败；
4. 仅native角速度超限失败；
5. 仅native最终指令非零超限失败；
6. native关键数据NaN、缺失一段或断流失败；
7. 超限目标误差、漂移或快照不一致失败；
8. 完成事件缺失/错route/错waypoint不能PASS；
9. writer健康/数据哈希及全部waypoint原有审计继续生效。
副本的内容哈希可以更新以证明拒绝来自判据本身；不得只靠哈希不匹配让反例失败。
保留 --verify-only 能力、历史CSV审计和旧v2/v3报告，不做无关重构。

六、新版本审计与封存
修复后对原始Full重新审计，输出新的文件，例如：
mission_verification_capture_v2.json
不得覆盖v1。保存新评估器及直接本地依赖哈希、实际复核命令、反例测试日志。
区分运行当时输入快照与本轮事后评估器版本：新评估器哈希不是当时运行输入，旧runtime快照不能被改写。
报告修复后GT/command逐窗口样本数、最大速度/角速度、误差、漂移和连续性，不硬编码顶层检查数量。

原始通过且反例全部拒绝后，新增暂定正向基线manifest，记录原始trial和证据路径、唯一目标、内部waypoint、现有阈值、真实运行输入manifest/原始文件哈希、新审计版本与命令、原生终点指标、0.0841436386 m采样最小间距、正式场地未核验状态及限制。
只封存已验证的正向Full，不把Return或整个省赛任务写成通过，不替换现有候选路线配置。
暂定0.8 m通道与训练虚拟墙仍待正式场地确认。保留缺口：起点、射击区、实际边界/墙体、转角连接宽度和固定障碍。

七、最终汇报
聊天框明确本轮指令是否全部完成：原始记录新审计是否通过、已知反例修复前后结果、全部原生停车指标、修改文件及是否影响导航输出、新报告/测试/manifest路径。
只有原始通过且反例全部正确拒绝，才写：
STAGE 1.5 PROVISIONAL FORWARD BASELINE AUDIT CLOSED
然后暂停扩大测试矩阵。
这不代表正式赛场验收、连续时间安全、统计重复性或Stage 2授权。
