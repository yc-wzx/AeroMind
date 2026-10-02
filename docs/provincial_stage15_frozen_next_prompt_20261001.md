继续维护 /home/rmnav/AeroMind。

当前已完成限定的暂定训练场 Stage 1.5 收尾，状态为：
STAGE 1.5 PROVISIONAL TRAINING VALIDATION COMPLETE
TRAINING-ONLY / PROVISIONAL
COMPETITION ARENA NOT VERIFIED
CONTACT DATA UNAVAILABLE

本轮只做冻结基线的只读复核和赛前 30 分钟操作准备，不启动 Gazebo，不发目标，不继续扩大测试，不进入 Stage 2。

先读：
docs/provincial_stage15_training_completion_20261001.md
docs/provincial_stage15_race_day_30min_20261001.md
tools/config/provincial_stage15_training_frozen_20261001.yaml

核对：
tools/results/provincial_stage15_completion_20261001/provisional_stage15_completion_audit_v3.json
hold_clock_recheck/independent_hold_audit_v3.json
round_trip_08、09、10 的输入/参数/原始哈希和原生数据。
旧 01–07 的失败记录及被 v2 取代的 clock_handoff v1 推断必须保留。

执行可重复的只读复核：
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/summarize_provincial_stage15_completion.py --verify-only

保持当前导航、时间基准、等待停车/恢复和守卫恢复计时。
保持 imperfect_sensors=false，GridRoute=.40 m，EGO inflation=.22 m，速度/车体/geometry/PGM/collision 和全部现有阈值。
不要为提高报表通过率改算法、阈值、路线或原始数据。
当前最小原生采样间距 0.0800902951 m，安全余量很小；不能声称连续安全、统计可靠性或正式赛场验收。

正式地图仅赛前约 30 分钟提供，格式尚不清楚，不再等待提前取得正式图作为开发结束条件。
基于现有 prepare_provincial_arena.py，整理两名队员分工的纸面操作演练清单：
1. 图片/PDF、CAD、现场资料各自如何确认单位/原点/起点/射击区/边界/障碍尺寸；
2. 哪些必须人工输入与交叉核对；
3. 哪些资料不足时要问裁判，不能用旧训练尺寸补齐；
4. 当前工具仅支持的平面几何范围，如何识别不支持的情况；
5. 新候选目录、预览、离线 connectivity/矩形间距/SDF-PGM 一致性与人工复核的操作顺序；
6. 30 分钟是预算，机器约 0.603 s 不代表总时间；安排人工盲测时由人实际计时，不能由模型编造；
7. 正式当天部署前仍需真实机器人验证，当前没有实机定位/电控接入。

不要生成虚构的正式比赛地图，不擅自启动 Stage 2 或安装传感器驱动。
如需规划 Stage 2，只列硬件信息缺口和候选工作，不执行接口或定位代码。
最后在聊天框说明本轮是否全部完成、基线是否仍一致、哪些是已有证据、哪些是操作计划或未验证项。
