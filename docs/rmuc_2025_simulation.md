# RMUC 2025 已建模场地仿真

双击 `D:\RMNav\Start-RMUC-Simulation.cmd` 启动 Gazebo、RViz、AeroMind 导航和机器人；双击 `D:\RMNav\Stop-Simulation.cmd` 停止。原来的 `Start-Simulation.cmd` 仍启动福建赛道。一次只运行一套仿真。

RMUC 场地视觉模型来自 `rmu_gazebo_simulator` 的 `rmuc_2025.stl`。Gazebo 静态碰撞使用保留真实 3D 高度的障碍网格，并排除可行驶的水平支撑面。正式 GridRoute 地图 `maps/rmuc_2025.pgm` 根据底盘碰撞高度带从同一场地网格生成；雷达高度截面和原始上游 PGM 分别保存在独立文件中。地图分层、高度参数和离线连通结果见 [RMUC 2025 地图语义与高度建模](rmuc_2025_map_layers.md)。

地图坐标按场地左下角 `(0,0)` 解释，场地约为 29 m × 16 m。GridRoute clearance 仍为 0.55 m，EGO inflation 仍为 0.22 m。本轮离线检查发现原示例起点 `(3,3)` 与目标 `(8.8,1.5)` 在地面图中不连通，因此该目标不能用于当前导航回归。场地内有一个离线可达的长距离候选 `(9.58,10.18)`，尚未在 Gazebo 中运行。当前交接明确禁止发送导航目标，所以仿真应保持无目标状态。

本阶段使用 Gazebo 360° 平面雷达；`imperfect_sensors=false`，不加入人为里程计漂移。这里验证的是 RMUC 仿真场地上的静态导航，不代表福建省赛场尺寸、真实 MID-360 或实际底盘定位表现。

源项目和许可：

- `SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator`：`src/uav_bringup/models/rmuc_2025/`，Apache-2.0，许可见 `docs/third_party_licenses/rmu_gazebo_simulator-LICENSE`。
- `SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav`：原始上游地图见 `src/uav_bringup/maps/rmuc_2025_source.pgm`，Apache-2.0，许可见 `docs/third_party_licenses/pb2025_sentry_nav-LICENSE`。

日志：`/home/rmnav/.cache/aeromind-simulation/latest.log`。场地源档导入脚本为 `tools/import_rmuc_assets.py`，地面地图生成脚本为 `tools/generate_rmuc_navigation_map.py`，3D 障碍碰撞网格生成脚本为 `tools/generate_rmuc_collision.py`，世界生成脚本为 `tools/generate_rmuc_world.py`。

