# Full 原始证据再核验与下一步（2026-09-30）

本轮只读核验及下一轮 prompt 已完成，没有启动 Gazebo或修改导航/评估器生产代码。原始 `trial_full_capture_01` 的实际通过结论保留；现有自动汇总仍有原生终点速度漏判，需先补齐再封存审计闭环。

## 原始证据

对 `tools/results/provincial_stage15_full_capture_20260929/` 执行现有 `summarize_provincial_full_observed.py --verify-only`，结果仍为 all_pass=true。当前原始数据、42 项输入快照及工作区对应文件哈希匹配；记录器计数一致。原目录全部 74 个文件的哈希在本轮结束时保持不变。

修正上轮两处表述：顶层检查是 **15 项**，并非 14；报告中 0.011394 m/s 最大速度、0.017750 m 最大窗口误差和零漂移来自较稀疏 CSV。逐条原生 GT 的正确数值如下，均满足原有门槛：

- 前 2 秒：sim_t 76.28→78.28，101 条 GT/101 条最终指令；GT 最大速度 0.0190720623 m/s，最大目标误差 0.0178260992 m。
- 后 5 秒：78.28→83.28，251 条 GT/250 条最终指令；GT 最大速度为零。
- 完整 7 秒：351 条 GT/350 条最终指令；GT/最终指令最大角速度为零，最终平移指令最大为零。相对于首条停车 GT 的最大漂移 0.0000759606 m（约 0.076 mm）。分窗共同包含 78.28 边界 GT，不能将分窗计数简单相加。
- GT 最大接收间隔约 0.021966 s，最终指令约 0.029590 s，完整窗口覆盖有效。

全程最小采样间距 0.0841436386 m、boundary_7、sim_t=28.24 s 的结论保留。到点误差 0.017750 m、唯一外部目标及三个内部 waypoint 完成的结论保留。

## 评估器漏判反例

现有 `audit_run()` 用 CSV 核验终点速度/漂移；`native_audit()` 检查原生消息的有限性、连续性、几何间距等，却没有把逐条原生 GT 的终点速度判据接入总 PASS。

在本轮新目录的完整副本 `counterexample_native_speed/` 中，仅将 sim_t=76.28 的一条 `/gazebo/odometry` 的 twist 平移速度改成 `(0.1,0)` m/s，并更新副本的 native 文件哈希。CSV、summary、健康计数、原始输入快照均未改。现有汇总 `--verify-only` 仍 all_pass=true，证明它不能独立拒绝此原生速度超限。此副本是合成反例，不是新导航试验。

这不推翻原始实跑：原始原生速度最大 0.019072 m/s，手工编写的只读脚本已逐条核验其符合 0.02 m/s。但不能宣称现有自动评估器已完整覆盖原生 2+5 秒。

## 证据

目录 `tools/results/provincial_stage15_full_capture_review_20260930/` 保存 `native_terminal_readonly_check.py`、`native_terminal_readonly_v1.json`、`original_tree_sha256.json`、`counterexample_mutation.json`、`counterexample_audit_v1.json` 和反例副本。原报告保留，由本说明修正统计口径，不覆盖历史。

下一轮只修独立评估器/针对性测试，强制从 native 原始数据重建前 2 秒和后 5 秒全部终点判据；原始记录仍通过、反例必须失败。完成后生成新审计文件和暂定正向基线 manifest，暂停扩大测试矩阵。维持 TRAINING-ONLY / PROVISIONAL、COMPETITION ARENA NOT VERIFIED、CONTACT DATA UNAVAILABLE；不进入 Stage 2。
