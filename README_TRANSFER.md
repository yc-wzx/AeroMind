# 当前暂定省赛仿真：下载与运行

小电脑迁移、依赖安装、构建、显示 Gazebo/RViz 的完整步骤见
[GitHub 迁移说明](docs/GITHUB_TRANSFER.md)。

推荐入口：`provincial_stage2_lio_fine_map.launch.py`。
最新模拟 LIO 验证：[体素优化报告](docs/stage2_step4_lio_fine_map_optimization_20261002.md)。

```bash
BUILD_JOBS=2 bash tools/deployment/build_training_sim.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_bringup provincial_stage2_lio_fine_map.launch.py \
  navigation:=true gui:=true rviz:=true auto_goal:=false
```

`TRAINING-ONLY / PROVISIONAL` / `COMPETITION ARENA NOT VERIFIED`。
历史原始采集仍保存在原电脑，不随 Git 下载。真实 MID-360/IMU 接入及实车验证尚未完成。
