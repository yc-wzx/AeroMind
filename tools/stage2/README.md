# Stage2 仿真步骤1–3（已完成）

最新报告：docs/stage2_steps123_completion_20261001.md
封存入口/输入：tools/config/stage2_steps123_completed_20261001.yaml

管理独立入口、GT/导航odom分离、比例/航向/随机游走累计误差，以及追加的已知2D场地扫描校正验证。不是MID-360、IMU、LIO或实机标定。

- summarize_steps123.py：原四组范围完成，未校正yaw_bias/random_walk的FAIL保留。
- run_scan_resume_startup_experiment.py：单记录器sensor-first编排，每次一个world/一个短目标，不自动重启或重发。
- audit_scan_resume_startup_v5.py：最新只读独立审计与实际溯源。
- tools/results/stage2_scan_resume_sensor_first_20261001_v3/：本轮严格PASS数据。
- tools/results/stage2_scan_capture_startup_fix_20261001/final_completion.json：完成汇总。

历史版本/失败/快照保留，目录已存在即拒绝覆盖。
TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。
