# 比赛场地 Gazebo 仿真

双击 D:\RMNav\Start-Simulation.cmd（或 Start-Gazebo.cmd）。
会同时启动 Gazebo 三维场地、全向机器人、激光、EGO 导航和 RViz。
数据就绪后约 10 秒开始自动驶向蓝色射击区。

Gazebo 看实际机器人、赛道和碰撞，RViz 看激光、规划和实跑轨迹。
在 RViz 顶部点 2D Goal Pose，在灰色自动赛道中心按住左键并拖动方向，松开发送目标。
回到红色起点也能规划，系统会按赛道的两个转角依次行驶。
Gazebo 左下角可暂停/继续；控制器使用仿真时间，不会在暂停时跳过轨迹。
双击 Stop-Simulation.cmd 停止整套仿真。关闭 Gazebo 窗口也会退出整套启动。
重复点击启动脚本不会创建第二套仿真。
再次启动从起点重新开始。

## 当前模型范围

- 场地外框 9.4 × 10 米，起点、射击区和目标区按旧赛规建立。
- 赛道宽度暂按 1.3 米训练近似；此尺寸未经赛规确认。
- 白色护栏是用于限制导航范围和碰撞测试的虚拟边界，不能据此制作正式赛场。
- 机器人采用 Gazebo 中的理想全向速度驱动，有碰撞体；未模拟滚子、打滑和起伏路面。
- 激光是 Gazebo 360 束、10 Hz 的二维扫描，不是 MID-360 的扫描模型。
- 定位暂用 Gazebo 真值；尚未验证 FAST-LIO 或真实雷达定位。
- 已知场地边界作为先验地图，与实时激光一起提供给规划器。

## 文件

源码在 WSL Ubuntu-22.04 的 /home/rmnav/AeroMind。
世界：src/uav_bringup/worlds/competition_2025.sdf
几何配置：src/uav_planning/config/competition_field_2025.json
启动：src/uav_bringup/launch/competition_gazebo.launch.py
修改几何后：python3 tools/generate_competition_world.py
日志：/home/rmnav/.cache/aeromind-simulation/latest.log

图形兼容性：GUI 使用 Ogre，激光服务器使用软件 Ogre2，解决本机 Intel WSL 图形接口崩溃和无效激光回波。
若以后出现 [WARN:COPY MODE] 并看不到窗口，停止仿真后在 PowerShell 执行 wsl --shutdown，再启动。
## 已执行验证

- Gazebo 实机位姿闭环往返测试通过：返程 19.3 秒，正程 19.0 秒。
- 1967 个位姿采样，车身与护栏无重叠，最小间隙 0.132 米。
- 返回起点误差 0.080 米，射击区误差 0.007 米，两端速度归零。
- 暂停 2.5 秒期间位姿和仿真时间均保持不变，恢复后继续行驶。
- 原有二维接口测试通过：高度过滤、平面里程计、目标和横向速度控制。
- 在两个 GUI 同时开启时，Gazebo 统计实时倍率约 0.998。
