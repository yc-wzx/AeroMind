# GitHub 迁移核验（2026-10-02）

本轮上传完成所需源码、配置、地图及构建脚本，未新增导航试验。
被核验的代码提交：`3ef563d3207b135da2ace5348dd2449fdb73ca6c`。
后续提交仅追加本目录核验记录。

- 干净副本的 1207 个版本化文件与当时提交内容一致；未带入原电脑 build/install/results。
- 10 个 ROS 包从源码编译通过。
- 专用 LIO 与 PositionImu 插件重建通过；补偿修正头文件与已验证版本逐字节相同。
- 新安装前缀下启动描述、Python 导入和动态链接检查通过。
- 重复安装及加载检查通过，没有覆盖原电脑基线。
- 原基线 330 个源码文件、2617 个证据文件哈希仍匹配。
- 保留的体素审计工具 8 项测试通过。

首次临时副本构建因 bringup 依赖一个被发现但未构建的 Livox 包而失败，
修复为只发现指定的十个仿真包后，在新的干净副本重建通过。
首次临时目录随 WSL 生命周期未保留；其完整日志不可用，工具输出中的失败
记录见 initial_build_failure.json。最终构建完整输出及重复安装检查已经留档。

本次是在原电脑已有的 Ubuntu 22.04 / ROS Humble / Fortress 系统依赖上做干净源码构建，
不是新操作系统、小电脑或实车验证，也未启动 GUI/Gazebo 或发导航目标。
原始历史采集仍留在原电脑；上传摘要不能替代全部原始证据。

TRAINING-ONLY / PROVISIONAL
COMPETITION ARENA NOT VERIFIED
CONTACT DATA UNAVAILABLE
