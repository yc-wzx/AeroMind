# Stage 1.5 正向证据闭环与 Full 间距离线诊断（2026-09-28）

**本轮指定工作已完成。** 当前结果仍限于 `TRAINING-ONLY / PROVISIONAL`；`COMPETITION ARENA NOT VERIFIED`。没有启动 Gazebo、发送导航目标或改变导航输出、planner、场地、地图、车体及任何验收阈值。旧原始记录、`mission_verification.json` 和输入 manifest 均保留。

## 新版独立审计

原汇总只强制核对最终 waypoint。修复前反例保存在 `tools/results/provincial_stage15_forward_integration_20260928/pre_fix_waypoint_audit_reproduction.json`：临时副本删除 Full 的 `R0001:W00` 完成事件，并更新副本的数据哈希，旧汇总仍给出 `all_pass=true`。修复后相同副本的总判定为 FAIL；原始 Full 的 `R0001:W00 → W01 → W02` 依次完成，仍为 PASS。

新版 `tools/summarize_provincial_forward_integration.py` 从原始 route 计划事件解析 waypoint ID、顺序和坐标，与离线预检及最终目标核对；逐条检查完成事件归属、顺序、时间、位姿及报告距离。完全相同的重复传输只计一次，互相矛盾的重复传输不通过。只有最后一个 waypoint 对应最终目标及完整 2+5 秒停车证据。临时副本测试覆盖删除首/中/末事件、乱序、错误 route、坐标冲突和重复事件；全部按预期判定。

新增只读 `--verify-only` 和拒绝覆盖旧文件的 `--output-name`。两次对同一证据运行的判定一致。新的版本报告为：

| 原始试验 | 新报告 | 全部内部完成事件 | 完整停车窗口 | 总判定 |
| --- | --- | --- | --- | --- |
| 同一 Gazebo 的 A→B→C | `continuous_abc/mission_verification_v4.json` | A `W00`；B `W00→W01`；C `W00→W01` | 三段 PASS | PASS |
| 独立 Gazebo 的单目标 Full | `single_full/mission_verification_v4.json` | `R0001:W00→W01→W02` | PASS | PASS |

从原始 CSV 复核，同一 Gazebo 的 A/B/C 第一条 GT 样本时间依次为 `0.58 / 14.12 / 41.16 s`，前两段完整终点快照分别为 `13.52 / 39.70 s`；下一段首条样本均在完整停车窗口结束后。两段衔接的首末 GT 坐标相同，route ID 依次为 `R0001/R0002/R0003`。报告分别记录进程退出、导航完成和原始证据审计结果；总 PASS 由重新计算的证据产生。原始输入哈希、快照和运行后原始数据哈希仍匹配。37 项相关测试通过。

在已加载 ROS 环境的 shell 中，可重复运行只读核验：

```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/summarize_provincial_forward_integration.py --mode chain --verify-only
python3 -B tools/summarize_provincial_forward_integration.py --mode full --verify-only
```

上述命令不会覆盖报告。`v4` 中还记录了完整的单行运行命令、当前评估器 SHA-256 与运行时保存的旧评估器 SHA-256。两者不同是本轮评估器修补造成的，原始导航及已列出的其他输入没有因此改变。已有运行器代码、一个 route 接受事件、Gazebo 进程记录和连续 GT 支持“同一 Gazebo 接续、每段一个目标”的结论；当时没有独立录制所有 `/goal_pose` 发布及 `set_pose/reset` 服务调用，因此不能把“没有额外外部目标或重置”表述成完全独立的总线级审计。

输入追溯仍有边界：原运行 manifest 保存了 18 个文件的哈希，其中非 install 文件有快照，包括导航接口、GridRoute、几何、场地、world、PGM、EGO 主配置、planner 二进制及当时评估器。launch 实际依赖的 `ego_goal_adapter.py`、goal adapter YAML、trajectory executor YAML、运行辅助模块及相应 install 文件没有全部进入当时 manifest。本轮 `v4` 只补记这些文件的**当前**哈希，明确标为 `capture_time_hash_available=false`；ROS/Gazebo 包版本也无完整运行时记录。它们不能被事后补算成运行时哈希。当前 source/install 对应的导航、轨迹执行器、adapter、配置、world 和地图内容已核对一致。

## Full 最窄处

离线脚本 `tools/analyze_provincial_forward_clearance.py` 使用运行时快照中的 SDF 和原始 CSV/JSONL。Full 的采样最小车体—墙间距为 **0.0844864394 m**，出现在 `sim_t=24.92 s`、GT `(5.453734, 1.855514)`、yaw 90°。最近的是 SDF `boundary_7`，水平范围 `x=4.3–8.3 m`，朝向通道的实体边界 `y=2.20 m`。机器人此时长边沿世界 y，车体最上缘 `y≈2.115514 m`，故间距为 `2.20−2.115514≈0.084486 m`。

参考线为 `y=1.800 m`，若车体沿参考线且保持 90°，对这堵墙的几何间距为 `0.140 m`。Full 的 GT 向上墙偏 `0.055514 m`，几乎正好解释减少的约 `0.0555 m`。当时有效局部计划在相同 x 的点约为 `(5.454650,1.855326)`，setpoint 为 `(5.445471,1.855317)`；GT 与 setpoint 的世界 y 差约 `0.000196 m`，完整平面距离约 `0.00827 m`（主要来自 x 时序差）。附近 4 秒诊断状态为 `active/allowed/ego_executor`，计划间距最低约 `0.08455 m`、守卫预测最低约 `0.08445 m`。证据支持**局部计划本身在此处贴近上墙，控制基本跟随该计划**；尚不能仅凭一次 Full 和一次 B 判断该计划形状差异的算法原因。

在共同空间带 `x=5.3–5.6 m`，Full 30 个 GT 样本中 y 最高 `1.855514 m`、最小间距 `0.084486 m`；分段 B 的 50 个样本中 y 最高 `1.800264 m`、最小间距 `0.138880 m`，其同位置局部计划 y 约 `1.7987 m`。两次起点、目标距离及 EGO 历史状态不同，这只是对照观察，不能直接归因于某个参数或控制故障。上个 waypoint `W00` 已在最窄点前完成；最窄点附近 yaw 保持 90°。原始/最终指令的机体系 `vy≈−0.17 m/s` 经 yaw 变换后主要是世界 `+x≈0.17 m/s`，不能将负的机体系 vy 误读为朝下墙移动。

图像与数值证据保存在 `tools/results/provincial_stage15_forward_integration_20260928/offline_clearance_review_v2/`：`full_min_clearance_plan_view.png` 显示墙、参考线、Full/B GT、各自局部计划和最窄处车体矩形；`full_min_clearance_timeline.png` 显示前后各 2 秒的 GT/odom/setpoint、三种间距、机体指令转换后的世界 x 速度、参考线及跟踪误差；`full_min_clearance_offline_audit.json` 给出所用原始文件哈希和数值。

Full GT 的最大相邻采样间隔为 `0.06 s`，最窄处前后 2 秒内也是 `0.06 s`。所有**已采样**间距均未低于 `0.08 m`，最近样本比阈值高约 `4.49 mm`。现有离散 GT 与诊断没有给出可独立验证的连续时间间距下界，因此不宣称采样之间必然始终高于阈值。也没有接触传感器数据：**CONTACT DATA UNAVAILABLE**。当前证据不支持为通过路线立即改动 planner 或安全参数；保留该窄余量限制，待正式省赛几何或新验证条件明确后再决定是否需要最小修复。
