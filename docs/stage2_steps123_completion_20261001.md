# Stage2 第1–3项完成报告（2026-10-01）

**本轮指令完成，停止在你要求的第3项。采集启动同步、最新严格断流停车与恢复证据也已补齐。不生成下一轮prompt。**

REQUESTED STAGE2 STEPS 1–3 COMPLETE
STAGE2 LOCALIZATION-HOLD RETRY LIVE VALIDATION COMPLETE

TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。

## 第1–3项

1. 独立Stage2入口完成，保留Stage1.5基线，默认zero，imperfect_sensors=false。
2. GT与导航odom分离完成：导航读模拟里程计，GT仅用于记录与试验终止，不把GT位姿反馈校正或控制；坐标约定、watchdog与最终速度守卫保留。
3. 比例、航向偏置、随机游走累计误差模型与四组有界试验完成；不逐帧回真值、不在发目标时重置误差。19项原工具测试、7类终点反例及逐步递推证据保留。

本轮重新运行summarize_steps123.py，requested_steps123_complete=true，10项检查全部通过。四组模型和原始记录/输入核验通过，Stage1.5的92项封存输入及登记历史证据均未变。

| 原未经扫描校正的试验 | 结果 | GT终点误差/最小采样间距 |
|---|---|---|
| zero | PASS | 0.017660m / 0.084080m |
| scale | PASS | 0.041273m / 0.090521m |
| yaw_bias | FAIL保留：真值间距低于0.08m | 未到终点 / 0.079963m |
| random_walk | FAIL保留：真值间距低于0.08m | 未到终点 / 0.079986m |

“第3项完成”指误差注入与有界验证完成，不是任意误差下都能安全完成任务。此前追加的2D已知场地扫描校正及三组Full成功记录保留，它们属于历史运行版本；不能冒充本轮新代码重新跑过全部Full。本轮最新代码另外完成下述严格短距离验证。

当前是360束/10Hz瞬时2D仿真雷达，不是MID-360点云、IMU/LIO或实机标定；这些后续阶段未接入。

## 本轮工具修复

单一记录器先启动，然后启动一个Gazebo、bridge、relay、模拟odom及原规划控制节点。双扫描内容/时间戳匹配、真实发布端与本记录器接收端GID/实际QoS稳定、核心记录新鲜有效后，才启动定位节点；导航记录全部就绪才放行唯一目标。

三个专用launch文件直接复用原Node动作及参数。未修改planner、控制、定位校正、任务计时修复、最终速度、世界/地图/collision、物理步长或传感器频率。

工具修正：本机QoS API按tuple解包，使用DDS实际解析后的发布/订阅两端QoS；就绪图查询约10Hz执行，其间持续接收并保存全部原始消息；补齐progress.profile_path；v4核验启动原始数据与先后顺序，v5以实际SHA纠正继承的旧溯源文字。sensor-first试验要求全部最终输出/actuation都有真实clock关联，不需要启动缺时间豁免；不补零/插值/伪造时间戳。

28项隔离测试通过，覆盖实际Runner/采集入口、实际Humble QoS API、缺话题/扫描关联、错误发布端/GID/QoS、NaN、断流/非零停车指令、writer故障、既有目录和不自动重启/重发。

本轮两次工具启动失败均保留：第一次QoS API错误，定位未启动、目标0；第二次就绪预检超时与缺profile_path，目标0。第二次过密检查的影响由代码与隔离测试支持，未独立证明唯一调度根因。工具修正后第三个独立目录通过。合计3次world启动，实际仅1个外部导航目标，不是反复发送目标筛选成功。

## 最新实跑与独立审计

目录：tools/results/stage2_scan_resume_sensor_first_20261001_v3/
trial：stage2_scan_resume_sensor_first_03
起点(4.7,0.5)，唯一目标(4.7,1.15)，yaw90°，R0001:W00。没有set_pose/reset/清任务/重启导航或外发中间目标。无重置依据限于运行器、日志、连续GT与任务记录，没有监控所有服务调用。

| 检查 | 原始证据结果 |
|---|---|
| 接受确认延迟 | 0.001787416单调秒 |
| relay暂停/恢复 | sim=16.60 / 19.06；丢弃25条scan，原始scan继续发布 |
| 末次有效扫描/最终零输出 | 16.50 / 17.02；0.52仿真秒，0.503087536单调接收秒 |
| GT停车 | sim=17.06；末次scan→停车0.56s，hold→停车0.46s |
| 最大停车位移/稳定观察 | 0.061900976m / 2.00仿真秒，稳定采样漂移0 |
| 新扫描/恢复运动 | 19.10 / 19.14仿真秒，保持原任务 |
| task_clock | paused=17.02，resumed=19.136，同一R0001:W00；hold=2.119376107单调秒 |
| retry/stuck/false success | 0 / 0 / false；内部目标记录1条，无重发 |
| 最终GT / odom误差 | 0.017022393 / 0.016889030m |
| 最终GT / yaw | (4.700078305,1.132977787) / 1.569501468rad |
| 完整2+5秒终点 | 21.74–23.74、23.74–28.74，CSV/native均通过 |
| 含终点观察的任务仿真耗时 | 13.86s |
| 最小采样车体—墙间距 | 0.189000074m，boundary_5，sim20.60，GT=(4.700437817,1.025349088)，yaw1.568632477rad |

246条accepted诊断全部找到原始/转发scan与odom；0条rejected，3条非扫描诊断单列，没有删除启动诊断。全部任务期扫描匹配与校正变换重新核算。没有unsafe_plan，没有采样间距低于0.08m或几何重叠。

原生GT1437条，50Hz，最大消息时间间隔0.020s；全程最大单调接收间隔0.024764016s，行驶期0.021305169s。最终指令全程最大接收间隔0.054270415s，odom0.057782189s，满足原0.2/0.5s判据。接收/写入/实际行数一致，writer队列峰值29/8192，无溢出或关闭失败。

自动v4与独立v4均23项PASS；溯源v5增加运行审计依赖内容确认后24项PASS。没有降低阈值。运行本地依赖120项内容、路径、符号链接和SHA已封存且前后匹配；原始文件关闭后哈希包含health/timing。系统依赖与动态dlopen插件没有完整镜像，不是全系统封存。v5为事后只读评估器，运行时v4保存在快照，未伪称当时用了v5。

## 归档入口

- 最新报告：docs/stage2_steps123_completion_20261001.md
- 完成清单/封存配置：tools/config/stage2_steps123_completed_20261001.yaml
- 本轮汇总：tools/results/stage2_scan_capture_startup_fix_20261001/final_completion.json
- 第1–3项独立复核：同目录steps123_final_reaudit.json
- 最新严格审计：实跑目录independent_audit_provenance_v5.json
- 原始逐消息、CSV、事件/计划/指令、health/timing、输入快照：最新实跑目录
- 图/GT时间线：实跑目录final_review/scan_loss_resume_pass.png、native_task_timeline.csv
- 最终源码/证据封存：工具结果目录final_evidence_seal.json

可复核命令（输出必须不存在）：
```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/audit_scan_resume_startup_v5.py \
 --result-dir tools/results/stage2_scan_resume_sensor_first_20261001_v3 \
 --output /tmp/new_stage2_scan_resume_review.json
```

修改文件仅专用启动编排、采集就绪/审计/隔离测试工具、README与报告/清单。旧16项源码和168项封存证据匹配，旧retry=1、9条缺扫描及本轮启动失败数据不覆盖、不回滚。

## 停止边界

imperfect_sensors=false、GridRoute=.40m、EGO inflation=.22m、gap=.08m保持。没有接触传感器，因此CONTACT DATA UNAVAILABLE；采样几何不重叠不是独立接触检测通过。50Hz不是连续时间安全证明；只有一次新导航试验，不构成统计重复性。错误frame/硬格式live覆盖未新增。正式省赛场地仍未核验。

本轮第1–3项及追加断流恢复闭环完成，仿真进程退出，停止扩展，不生成prompt。
