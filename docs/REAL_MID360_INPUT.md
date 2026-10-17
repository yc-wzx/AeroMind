# 真实 MID-360：先采集，再只读 LIO

2026-10-17。阶段：REAL SENSOR INPUT PREPARATION；NOT REAL NAVIGATION ACCEPTANCE。
原 `rule900` 仿真入口、导航控制、参数和历史数据不变。
本入口不启动 Gazebo、planner、电控串口或速度发布者。

## 小电脑实际操作

先在原仿真启动终端按 Ctrl+C 结束本次仿真，并关闭 Livox Viewer。
保持雷达和云台静止，第一次采集期间不要转云台或推动车辆。
用户确认主机有线地址192.168.1.5，MID-360为192.168.1.173；脚本只读核对，
不修改IP、路由、防火墙、代理、系统时间或设备固件。

新开普通终端（不要再进入交互式 clean bash）：

```bash
cd ~/AeroMind_20261016
git status --short
git pull --ff-only
BUILD_JOBS=2 bash tools/hardware/mid360/build.sh
bash tools/hardware/mid360/run.sh
```

若 git status 显示自己的源码修改，先保留并处理，不能 reset/clean 覆盖。
脚本使用独立 build_mid360/install_mid360/log_mid360，不重写仿真 install。
编译依赖已有 /usr/local 下 Livox-SDK2；若缺少会明确停止，绝不自动重装系统。
colcon若未找到，需要系统包python3-colcon-common-extensions；其它ROS依赖
沿用已经搭建好的Humble环境。新构建同时验证驱动和原版SPARK LIO。

run.sh 默认只启动真实点云、原始/单位换算IMU及RViz，不启动LIO。
屏幕显示原生雷达坐标；倒置安装使该坐标的Z朝向与水平控制系不同，
不能为使画面好看只翻点云。原生轴和时戳全部保留。

看到连续更新点云后，保持启动终端运行，新开第二个终端：

```bash
cd ~/AeroMind_20261016
bash tools/hardware/mid360/observe.sh --seconds 15
```

等待15秒，输出SUMMARY路径和完整统计。将summary.json内容提供给维护者。
默认数据目录 `~/AeroMind_mid360_captures/年月日_时分秒/`，若同名已存在就停止，
不覆盖或自动重试。raw_bag保留所有实际收到消息的序列化内容，receipts.jsonl
保留各条设备时间、单调接收时间和主机接收时间。默认保存约数十MB，依场景而定。
接收序号及bag行数只能证明记录到的消息，不能证明发布端绝无丢包。

需要单独查看话题时，每条命令均使用自己的干净环境和硬件域43：

```bash
bash tools/hardware/mid360/env.sh ros2 topic list
bash tools/hardware/mid360/env.sh ros2 topic hz /hardware/mid360/points
bash tools/hardware/mid360/env.sh ros2 topic hz /hardware/mid360/imu_raw
```

## 数据和单位

* /hardware/mid360/points：驱动格式0的原生PointCloud2；包含timestamp、line、tag。
* /hardware/mid360/imu_raw：驱动原样加速度g、角速度rad/s；不作为标准SI IMU使用。
* /hardware/mid360/imu_si：仅加速度乘9.80665，协方差乘该系数平方；保持时间和轴，
  角速度不变，无姿态时显式orientation_covariance[0]=-1。未扣重力或偏置。
* 硬件域43与仿真域42隔离，不会悄悄替换 /ground/odometry 或导航速度。

捕获会检查消息结构、全部关键数值有限性、逐点时间范围、设备时戳单调性、
真实接收间断、数据流重叠、原始/换算IMU逐时戳匹配以及bag读回计数。
原始包不抽样、不补零、不替换时戳。传感器时钟可能不是主机Unix时钟；
报告会保留差值，但绝不据此偷偷改时钟或把接收时刻当成测量时刻。
断流、无消息或异常不会被填成PASS。筛选阈值和来源见工具目录SOURCES.md。

## 只读 LIO 候选（先等本机原始采集审核）

原始采集通过且时间基准、静止IMU方向已审核后，保持当前驱动继续运行，选择
显式提供该次summary的只读LIO入口：

```bash
bash tools/hardware/mid360_lio/run.sh /实际采集目录/summary.json
```

缺少通过的采集报告、输入构建不一致或数据哈希改变会拒绝启用LIO。
2026-10-17 修正：旧候选YAML将visualization_frame模式填成了坐标名；新入口使用独立
修正版配置（mode=imu，imu_frame=mid360_imu），不改旧输入及采集哈希，不需重采。
旧的 lio:=true 入口不再推荐使用。
本机目前没有接真实雷达，不能将本地工具测试或人工合成消息宣称硬件通过。
目前应执行前面的 sensors-only 和15秒采集；本轮不授权真实车体运动。

候选LIO直接使用重建的SPARK PointCloud2 MID360分支（enum5），保持真实逐点
时差与常规IMU初始化，未加载仿真“无偏IMU/瞬时点云/精确起点”开关。
内置IMU相对LiDAR的内部外参采用上游作者MID360标称值；不是整机相对云台的安装外参。
整机倒置不会把内部外参变成180°。标称值不等同于本台设备标定结果。

LIO输出 /hardware/mid360/lio/odometry，map_frame=mid360_local_odom。
它保留启动时传感器局部坐标，不把未知底盘朝向、安装高度、云台转轴和杠杆臂
写成仿真数值。暂不宣称重力对齐地图或底盘中心位姿已成立。
雷达随云台转动时，还需后续安装变换、转轴偏置和地图对齐验证，才能接导航。
真实IMU噪声/偏置、硬件时钟与导航时钟的对应关系也需要这次实测。

## 本轮验证范围

开发机可做源码构建、原生PointCloud2实际C++预处理、IMU单位/协方差/时戳、
错误数据拒绝和原始bag写入读回测试。结果保存在
`docs/evidence/mid360_input_20261017/`。
这些测试不代替小电脑的真实点云/IMU、LIO静止漂移或转云台实测。
正式场地、实车导航、电控失联停车、碰撞接触均未由本次证明。

COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。
