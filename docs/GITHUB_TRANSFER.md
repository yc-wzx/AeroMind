# 从 GitHub 迁移到小电脑

本次上传保存截至 2026-10-02 的省赛暂定训练场代码，推荐入口为
`provincial_stage2_lio_fine_map.launch.py`。仿真传感器、三维 LIO、全向导航及
等待停车逻辑均保留；LIO 的 `filter_size_map=0.05`，原有入口继续保留。

状态仍为 `TRAINING-ONLY / PROVISIONAL`、`COMPETITION ARENA NOT VERIFIED`。
没有接触传感器证据：`CONTACT DATA UNAVAILABLE`。这不是实车验收。

当前登录账号为 `yc-wzx`，对学长 `Gypsophilazd/AeroMind` 仓库没有写权限，
因此本版上传到个人 fork `yc-wzx/AeroMind` 的专用分支，保留学长仓库远端。

## 拉取

新电脑第一次下载：

```bash
git clone --branch codex/provincial-stage2-transfer \
  https://github.com/yc-wzx/AeroMind.git
cd AeroMind
```

已有仓库且工作区干净时：

```bash
cd /你的路径/AeroMind
git fetch origin
git switch codex/provincial-stage2-transfer
git pull --ff-only
```

如果小电脑已有自己的修改，保留它们并在另一个目录 clone，避免强制覆盖。
工作目录可以不同于 `/home/rmnav/AeroMind`；本轮迁移脚本根据自身位置解析路径。

如果已有仓库的 `origin` 仍指向学长仓库，上面的 fetch 不会取得本分支。
在干净工作区新增个人远端并首次切换：

```bash
git remote add training https://github.com/yc-wzx/AeroMind.git
git fetch training
git switch --track training/codex/provincial-stage2-transfer
```

后续在此分支运行 `git pull --ff-only`。`training` 已存在时先用 `git remote -v`
确认地址，不要重复添加。

## 环境和依赖

已验证的环境是 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress（`ign gazebo`）。
小电脑应先安装对应架构的 ROS 2 Humble 并配置 ROS 软件源。不要复制旧电脑的
`build/`、`install/` 或 `.so`；在小电脑重新编译。Windows 使用 WSL2 Ubuntu 22.04
及可用图形环境；Linux 桌面直接运行。当前没有验证小电脑的 GPU、内存和实时性能。

配置 ROS 软件源之后，安装构建和仿真依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-pil python3-yaml python3-matplotlib \
  libeigen3-dev libpcl-dev \
  libignition-gazebo6-dev libignition-plugin1-dev libignition-sensors6-dev \
  ros-humble-desktop ros-humble-ros-gz ros-humble-pcl-ros \
  ros-humble-pcl-conversions ros-humble-cv-bridge \
  ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs
```

该仿真构建不需要真实雷达，也不需要 Livox-SDK2；它明确选择仿真依赖包，
不会构建硬件 `livox_ros_driver2`。

## 构建

```bash
cd /你的路径/AeroMind
BUILD_JOBS=2 bash tools/deployment/build_training_sim.sh
source install/setup.bash
```

内存不足可用 `BUILD_JOBS=1`。脚本限制 colcon 和 C++ 编译并发，扫描范围仅为
`src/`，避免历史快照被误识别为重复 ROS 包。

脚本会编译普通 ROS 包，然后从版本化源码构建：

- `PositionImu` 物理仿真插件；
- 带瞬时点云处理修正的独立 LIO 插件；
- 安装安全导航及相邻 Python 导入模块。

独立 LIO 仍由 `lio_point_boundary_mapping.py` 显式加载，不替换原版 FAST-LIO。
源修正由 `tools/stage2/lio_optimization/prepare_instantaneous_cloud.py` 重建，
不依赖本机的 `tools/results/`。编译产物哈希依赖电脑、工具链和绝对路径，
不能要求与历史试验二进制哈希相同。

安装脚本拒绝覆盖字节不同的现有插件。已有旧安装发生冲突时，请使用新 clone
目录重新构建；不要删掉历史基线来强行通过。构建后的清单位于
`build/training_profile/installation_manifest.json`。

## 显示仿真

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_bringup provincial_stage2_lio_fine_map.launch.py \
  navigation:=true gui:=true rviz:=true auto_goal:=false
```

Gazebo 和 RViz 应显示训练场；等待传感器与定位就绪，再在 RViz 使用
`2D Goal Pose`。目标要落在空闲通道内，避免墙体。这个入口不会自动发目标。
若环境没有图形显示服务器，先解决桌面/WSLg 显示；`gui:=true` 无法替代显示服务。
停止时在启动终端按 Ctrl+C。

只检查安装，不启动仿真：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/deployment/check_training_install.py
```

## 上传内容与证据边界

上传导航源码、第三方依赖源码与许可证、配置、地图、场地模型、工具、历史报告，
并在 `docs/evidence/training_baseline_20261002/` 保存最新验证摘要、哈希清单和图。
其中 `copy_manifest.json` 记录这些副本与本机原件的 SHA-256。

约 21 GB 的 `tools/results/` 原始采集、输入快照及旧失败现场继续保留在原电脑，
不进入 Git。`build/`、`install/`、日志及可再生缓存也不上传。
摘要和哈希清单不能替代原始证据；历史审计/封存工具需要原始记录才能完整复核，
不要在新电脑仅凭摘要声称已复验历史结果。

最近一次原电脑 Full：独立审计 36 项通过，GT 到点误差约 3.06 cm，
最大 LIO XY 偏差约 2.54 cm，最小采样车体—墙间距约 12.83 cm，完整 2+5 秒
停车观察通过。这是单次暂定训练场仿真，不承诺实车精度、统计重复性或连续时间安全。

迁移验证另见 `docs/evidence/github_transfer_20261002/`。迁移构建/导入检查本身
不产生新的导航通过结论。
