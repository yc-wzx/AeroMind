# 导航项目文件索引

## 当前入口

- Stage 1.5 已冻结：`provincial_stage15_training_completion_20261001.md`。
- 封存配置：`tools/config/provincial_stage15_training_frozen_20261001.yaml`。
- 原理想定位启动入口：`src/uav_bringup/launch/provincial_2025_provisional.launch.py`。
- Stage 2 开发报告：`stage2_simulation_steps123_20261001.md`。
- Stage 2 独立入口：`src/uav_bringup/launch/provincial_stage2_odometry.launch.py`。
- Stage 2 工具：`tools/stage2/`；本轮证据：`tools/results/stage2_steps123_20261001/`。
- 赛前建图准备：`provincial_stage15_race_day_30min_20261001.md`。

## 文件保留原则

历史报告、失败现场、CSV/JSONL、快照及哈希清单用于复核，不因版本旧而删除。
`src/third_party` 包含编译依赖与许可；`build` 可能是安装符号链接实际目标，不能直接当作垃圾。
清理仅删除确认可再生成的 Python 缓存，逐项记录于本轮 `cleanup_manifest.json`。
历史“下一轮 prompt”是对应阶段的交接记录；当前工作范围以用户最新指令为准。
