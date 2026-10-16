# 2026 省赛赛道四：当前训练仿真迁移

当前推荐入口是 `tools/deployment/run_rule900_training.sh`，它运行直径
0.60 m 保守圆形包络、倒置雷达随云台 yaw 转动、物理底盘不自转的训练仿真。
原来的 2D / fine-map 入口和历史记录保留。不要用旧的 2D 启动命令代替本入口。

状态：`TRAINING-ONLY / PROVISIONAL`；`COMPETITION ARENA NOT VERIFIED`；
`CONTACT DATA UNAVAILABLE`。本次是源码部署准备，不是实车导航验收。

## 小电脑下载、构建和启动

适用环境：Ubuntu 22.04 x86_64、ROS 2 Humble、Gazebo Fortress。
建议新目录，保留小电脑已有的 `~/AeroMind` 和自瞄/MVS 环境：

```bash
cd ~
git clone --depth 1 --single-branch --branch codex/provincial-stage2-transfer \
  https://github.com/yc-wzx/AeroMind.git AeroMind_20261016
cd ~/AeroMind_20261016
BUILD_JOBS=2 bash tools/deployment/build_rule900_training.sh
bash tools/deployment/run_rule900_training.sh --check-only
bash tools/deployment/run_rule900_training.sh
```

最后一条显示 Gazebo 和 RViz，需要已有可用的桌面/NoMachine 图形会话。
启动后不自动下发目标；等定位和传感器稳定后，在 RViz 用 `2D Goal Pose`，
目标放在空闲通道内。Ctrl+C 停止。无界面检查用 `--headless`。
本次启动验证使用 headless，尚未在小电脑重新验证 GUI 和实时性能。

脚本按当前目录解析所有路径；新路径需重新构建，不可搬运旧 install。
清洁环境仅对子进程生效，用来避免海康 MVS 的 Qt/libusb 库和旧 ROS overlay
混入仿真，不修改 shell 启动文件、MVS 安装或系统网络设置。
默认本机 ROS_DOMAIN_ID=42，ROS_LOCALHOST_ONLY=1；观察命令应使用同一域。

## 依赖与磁盘

已配置 ROS 软件源的电脑可按原依赖说明安装；注意正确名称是
`libignition-plugin-dev`，不是 `libignition-plugin1-dev`：

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-pil python3-yaml python3-matplotlib \
  libeigen3-dev libpcl-dev libignition-gazebo6-dev libignition-plugin-dev \
  libignition-sensors6-dev ros-humble-desktop ros-humble-ros-gz \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
  ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs
```

仿真不需要编译真实 Livox 驱动。BUILD_JOBS=1 可降低编译内存占用。
本次独立源码构建约 8 分 17 秒，build 约 1.3 GB、install 约 540 MB；
这些是本机测量，工具链、依赖下载和小电脑耗时/空间可能不同。
不要求删除小电脑其他工作；先检查 `df -h /`。

## 版本与实际验证

本版带齐独立 Python 依赖和插件源码，在新目录编译 11 个 ROS 包，
另外重建当前 LIO 源码；不上传 install/build/.so，也不复用旧电脑二进制。
保留已通过版本的规划、控制、等待停车逻辑及参数：imperfect_sensors=false，
GridRoute=0.40 m，EGO inflation=0.22 m，速度 0.25 m/s、加速度 0.35 m/s²。
两处 EGO 源码差异是原可运行版本已经采用的修复，本次直接保留。
部署改动包括依赖归包、相对安装路径、本机构建清单及去除旧绝对路径 gdb 包装。

本次验证：干净构建通过；3 个生产 C++ 测试通过；39 个私有模块导入通过；
动态库解析通过；正反向离线路线通过；一次 20 秒仿真启动检查通过，
有仿真点云/IMU/LIO，真实进程加载新目录文件，未发布导航目标，空闲输出为零。
停机日志含 SIGINT 导致的 Python KeyboardInterrupt，不代表运行中崩溃。

原电脑历史全流程 260.74 秒 PASS 的摘要另行保留，但本次没有在新构建上重新
运行全流程，也不能据此宣称小电脑或实车已经通过。
证据位于 `docs/evidence/provincial_transfer_20261016/`。

每台电脑构建后生成自己的
`install/provincial_rule900_scratch_sim/share/provincial_rule900_scratch_sim/deployment_manifest.json`。
源文件或安装文件改变会拒绝启动，需重新构建。清单覆盖本地源码、主要运行产物，
不是完整操作系统镜像；系统 ROS/Gazebo/驱动依赖未封存为可移植镜像。
构建头部 Git hash 记录构建时 HEAD；提交后的同内容源码无需伪造重写旧记录。

## 真实硬件仍需下一步

本入口只运行仿真。真实 MID-360/LIO 的外参、IMU 先验、时间和话题，
以及 xuc 串口和电控失联停车，均不在本次接入范围。
当前 LIO 带显式 simulation-only 门禁；仿真无偏 IMU 参数不能直接用于真实雷达。
不要接电机后把仿真入口当作实车入口。
