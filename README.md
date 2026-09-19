# AeroMind Planar

AeroMind Planar 是面向 ROS 2 Humble 全向轮底盘的激光自主导航系统。它由学长的
AeroMind UAV 工程改造而来，使用 Livox Mid-360 与 IMU 作为三维感知输入，但导航
状态、重定位、地图、规划和控制都限制在 SE(2)：

```text
state   = [x, y, yaw]
control = [vx, vy, yaw_rate]
```

`vx` 与 `vy` 独立，机器人朝向不必等于运动方向。

## 数据流

```text
Mid-360 3D cloud + IMU
  -> SPARK FAST-LIO (3D deskew/gravity, persistent state projected to SE(2))
  -> /odometry
  -> /ground/odometry

/cloud_registered
  -> height ROI + XY projection
  -> /cloud_registered_2d
  -> EGO planar occupancy/search/B-spline
  -> /planning/bspline (message z fields remain 0 for ABI compatibility)
  -> holonomic tracker
  -> /cmd_vel (body vx, vy, wz)

planar prior PCD + /cloud_registered_2d
  -> robust SE(2) matcher
  -> map -> odom [x, y, yaw]
```

TF 为 `map -> odom -> base_link -> lidar_link`。前两段只包含 x、y、yaw；
`base_link -> lidar_link` 保留真实三维安装外参。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/rmnav/AeroMind
colcon build --symlink-install --packages-up-to \
  livox_ros_driver2 spark_fast_lio ego_planner uav_planning uav_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

仓库补充了 `livox_ros_driver2/package.xml`。Livox-SDK2 的头文件和动态库需位于
`/usr/local/include` 与 `/usr/local/lib`。

## 启动

默认启动不会向底盘发送速度：

```bash
ros2 launch uav_bringup autonomous_demo.launch.py
```

底盘协议、急停和限速完成实车检查后，再显式使能：

```bash
ros2 launch uav_bringup autonomous_demo.launch.py enable_cmd_vel:=true
```

点击 RViz `2D Goal Pose` 后，目标的 x、y、yaw 进入规划，输入 z 被忽略。

## 主要接口

| 名称 | 类型 | 语义 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Mid-360 原始点云 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | IMU 输入 |
| `/odometry` | `nav_msgs/msg/Odometry` | FAST-LIO SE(2) 输出 |
| `/ground/odometry` | `nav_msgs/msg/Odometry` | 规划与控制统一里程计 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | LIO 的 3D registered cloud |
| `/cloud_registered_2d` | `sensor_msgs/msg/PointCloud2` | 高度 ROI 后的 XY 障碍点云 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | x/y/yaw 目标 |
| `/planning/bspline` | `traj_utils/msg/Bspline` | 平面轨迹，兼容 z 字段恒为 0 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 体坐标 vx/vy/yaw_rate |
| `/relocalize` | `std_srvs/srv/Trigger` | 更新 planar map->odom |
| `/save_map` | `std_srvs/srv/Trigger` | 保存 planar PCD |

关键参数在 `src/uav_bringup/config/ego_acl_offline.yaml`、
`src/uav_bringup/config/spark_mid360_live.yaml` 和
`src/uav_planning/config/`。其中 `obstacle_min_z`、`obstacle_max_z` 必须根据底盘
高度和雷达安装高度实测调整。

## 无硬件测试

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/test_planar_nodes.py
python3 tools/test_se2_relocalizer.py
```

详细设计、修改文件和验收边界见 [docs/PLANAR_PORT.md](docs/PLANAR_PORT.md)。
