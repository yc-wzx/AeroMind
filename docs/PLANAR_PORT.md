# 3D UAV 到 2D 全向底盘的移植记录

## 1. Architecture Changes

原系统的三维飞行高度、飞控位置设定值与 6DoF 重定位已从主启动链移除。新的有效
导航自由度为 `x/y/yaw`，控制量为相互独立的 `vx/vy/yaw_rate`。三维激光和 IMU
仍用于点云去畸变、重力补偿和 LIO 匹配。

## 2. Modified Files

- `spark_fast_lio.h/.cpp`：在每次 LiDAR 更新后投影持久滤波状态，约束 z、vz、
  roll、pitch；高频 IMU 外推输出也应用同一投影。
- `grid_map.h/.cpp`：planar 模式使用一个有效 z=0 平面和两层索引边界；点云插入、
  膨胀、查询与显示固定在平面。
- `dyn_a_star.cpp`：启发式、终止条件、邻居扩展和路径输出只使用 XY。
- `planner_manager.cpp`、`ego_replan_fsm.cpp`：起点、目标、速度、控制点和发布的
  B-spline 均限制在 XY，消息 z 字段恒为 0。
- `bspline_optimizer.cpp`：控制点 z 固定为 0，z 梯度清零，速度与加速度可行性只
  计算 x/y。
- `planar_cloud_projector.cpp`：按高度 ROI 过滤 3D 点，投影 XY 并降采样。
- `thin_odom_adapter.cpp`：输出 SE(2) 位姿、vx/vy/yaw_rate，拒绝无效姿态。
- `ego_goal_adapter.py`：只接受 x/y/yaw，忽略目标 z，输出 yaw-only 四元数。
- `ego_trajectory_executor.py`：二维样条采样、位置反馈和全向体坐标速度控制；yaw
  独立跟踪目标朝向，不由轨迹切线强制决定。
- `kiss_relocalizer.cpp`：以高度过滤、XY 投影、体素化和最近邻对应为前端，解算器
  每轮只优化 tx/ty/yaw。输出矩阵严格为 SE(2)。
- `registered_cloud_map_saver.cpp`：只保存 z=0 的 planar PCD。
- `autonomous_demo.launch.py`：主链不再加载飞控，连接 ground odometry、planar cloud、
  planar EGO、holonomic executor 和 SE(2) relocalizer。
- `ground_final.rviz` 和 YAML：更新二维话题、参数、限速与地图尺寸。
- `livox_ros_driver2/package.xml`、`CMakeLists.txt`、`pub_handler.cpp`：修复 Humble 构建
  清单和当前 Mid-360 SDK 兼容性。

上游 `third_party/KISS-Matcher/ros` 保留作来源参考，但加了 `COLCON_IGNORE`；它的
6DoF 后端不进入运行链。项目自有 relocalizer 保留原节点、服务、话题和 TF 接口。

## 3. Motion Model

```text
state:   q = [x, y, yaw]
control: u = [vx, vy, yaw_rate]

p_dot = v,  p=[x,y], v=[vx,vy]
v_dot = a,  a=[ax,ay]
yaw_dot = yaw_rate

|vx| <= max_vel_x
|vy| <= max_vel_y
|yaw_rate| <= max_yaw_rate
```

控制器把世界坐标的二维样条速度和位置误差旋转到 `base_link`，分别输出 body vx
和 body vy。因此 `yaw=0`、`vx=0`、`vy>0` 是有效命令，不会先转向 90 度。

## 4. EGO 2D

障碍物先按 `obstacle_min_z <= z <= obstacle_max_z` 过滤，再投影到 XY。栅格结构为
兼容 EGO 仍保留三维索引，但仅中间 z=0 层是有效地图；上下两层只作为边界哨兵。
A* 的状态、启发式、扩展和距离均为 XY。B-spline 的存储/ROS 消息继续使用三行，
算法将 z 控制点和梯度固定为 0；速度、加速度和碰撞代价只作用于平面。

## 5. LIO 2D

FAST-LIO 保留三轴 IMU propagation、bias、gravity、点云 deskew 和三维 scan matching。
LiDAR update 完成后，滤波器的持久状态通过 `kf_.change_x()` 投影为固定高度、零 vz
和 yaw-only 姿态，所以约束不只存在于 ROS 输出。这个投影会改变滤波协方差一致性，
必须用实际 Mid-360 bag 和实车平面运动验证长期稳定性。

## 6. Registration 2D

source/target 经过相同的高度过滤、XY 投影和体素化。最近邻对应使用二维点集；加权
Procrustes 每次迭代只求一个角度与二维平移，状态向量没有 tz/roll/pitch。最终
4x4 矩阵固定为：

```text
[ cos(yaw) -sin(yaw) 0 tx ]
[ sin(yaw)  cos(yaw) 0 ty ]
[    0         0     1  0 ]
[    0         0     0  1 ]
```

## 7. ROS Interfaces

- frames：`map -> odom -> base_link -> lidar_link`。
- navigation topics：`/ground/odometry`、`/cloud_registered_2d`、`/goal_pose`、
  `/planning/bspline`、`/cmd_vel`。
- services：`/relocalize`、`/save_map`。
- ABI compatibility：`Pose.position.z`、`Odometry.linear.z`、B-spline point.z 和 TF z
  保留字段但恒为 0；LiDAR 静态外参的 z 保留真实值。

## 8. Build Result

闭环软件仿真入口：

```bash
ros2 launch uav_bringup planar_simulation.launch.py
```

`planar_navigation_simulator.py` 根据体坐标 `/cmd_vel` 积分全向底盘状态，发布
`/ground/odometry`、固定二维障碍物、TF 与实际运动路径。该入口不启动雷达驱动。

已在 Ubuntu 22.04 / ROS 2 Humble x86_64 实际编译以下包：

```text
livox_ros_driver2
spark_fast_lio
plan_env path_searching bspline_opt traj_utils ego_planner
uav_planning uav_bringup
```

验证命令：

```bash
colcon build --symlink-install --packages-up-to \
  livox_ros_driver2 spark_fast_lio ego_planner uav_planning uav_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 tools/test_planar_nodes.py
python3 tools/test_se2_relocalizer.py
```

## 9. Remaining Risks

- `obstacle_min_z/max_z` 需按传感器安装高度、底盘高度和赛场坡面标定。
- FAST-LIO 状态投影要用真实 bag 检查创新、协方差和长期漂移。
- `/cmd_vel` 与电控的单位、坐标方向、心跳、回包、急停和饱和仍需联调。
- 最大速度、加速度、位置/yaw 增益需要在真实全向底盘低速整定。
- SE(2) 重定位的初值、对应距离和 RMSE 阈值需要真实 PCD/bag 验证。
- 当前没有硬件，无法验证 MID-360 网络、时间同步、轮胎打滑和赛道通过性。

## 10. Verification

`test_planar_nodes.py` 覆盖高度 ROI、z=0 投影、SE(2) odometry、二维目标，以及
`yaw=0` 时直接产生正 vy 的全向控制。`test_se2_relocalizer.py` 用已知变换生成 source
和 target，验证只恢复 x/y/yaw。编译器还覆盖 EGO 的二维搜索和优化改动。
