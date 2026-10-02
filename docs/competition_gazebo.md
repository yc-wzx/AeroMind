# 比赛场地 Gazebo 仿真

> Stage 1.5 当前范围：用户确认实际省赛没有起伏路段。本场地按 2025 暂定平面图作训练近似，尚非本届正式几何。competition_gazebo.launch.py 已明确设置 imperfect_sensors=false；下列早期非理想传感器结果仅是历史记录，不是当前验收结果。

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
- 机器人有理想化单碰撞体；历史非理想模式会加入 0.12 秒响应滞后及 3% 速度比例误差，当前 imperfect_sensors=false 已关闭这些人为误差。仍未模拟滚子或打滑。
- 激光是 Gazebo 360 束、10 Hz、1 厘米标准差的二维扫描，不是 MID-360 的扫描模型。
- 历史非理想模式使用带比例偏差和随机游走的运动增量估计，并用激光回波匹配已知护栏修正；当前该模式关闭。Gazebo 真值可用于独立验收，但不能代替真实定位验证。
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

- 早期真值定位基线：往返通过，1967 个位姿采样无护栏重叠；详见 docs/gazebo_validation.json。
- 当前非理想传感器与驱动配置：五段连续目标全部通过，真值到点误差最大 0.052 米；4064 个位姿采样无护栏重叠，最小间隙 0.143 米。
- 激光护栏匹配后，定位误差中位数 0.011 米、95 分位 0.037 米、最大 0.051 米；详见 docs/gazebo_stress_validation.json。
- 运行中加入 0.20 米方形障碍后，机器人绕障返回起点，真值误差 0.023 米、车身最小障碍间隙 0.149 米；详见 docs/gazebo_obstacle_validation.json。
- 发现并修复 EGO 在紧急停车期间收到新目标时重复 spin 节点、导致规划进程退出的问题。
- 暂停 2.5 秒期间位姿和仿真时间均保持不变，恢复后继续行驶。
- 原有二维接口测试通过：高度过滤、平面里程计、目标和横向速度控制。
- 在两个 GUI 同时开启时，Gazebo 统计实时倍率约 0.998。

gazebo_navigation_interface 节点参数自身默认开启 imperfect_sensors，但当前 competition_gazebo.launch.py 显式传入 false，运行时以 launch 参数为准。仿真运行时，双击 D:\RMNav\Run-Stress-Test.cmd 可重跑五段目标，双击 D:\RMNav\Run-Obstacle-Test.cmd 可重跑障碍测试；报告在 D:\RMNav\reports。测得的误差来自这套训练场地和人为设定的噪声，实际雷达、底盘与赛场仍需实测标定。
