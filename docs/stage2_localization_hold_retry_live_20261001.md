# Stage2 定位断流与重试修复：单次实跑核验（2026-10-01）

本轮执行及归档完成：仅启动一次独立 Gazebo，仅发送一个短距离外部目标，未重发、未重跑。严格验收尚未完成，结论为 **FAIL / STARTUP SCAN EVIDENCE INCOMPLETE**。不能写 STAGE2 LOCALIZATION-HOLD RETRY LIVE VALIDATION COMPLETE。

TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。

## 已核验的运行行为

起点 GT=(4.7,0.5)，yaw=90°；唯一目标=(4.7,1.15)，yaw=90°，route=R0001、waypoint=R0001:W00。外部目标原始订阅记录1条、内部目标1条；接受确认延迟0.023174533秒，确认真实导航接收端的身份/GID/QoS及稳定时间。导航只读 guarded_scan，relay原样转发原始scan。未记录所有 reset/set_pose 服务调用，关于无重置的依据限于运行器代码、日志、连续GT与任务记录。

| 项目 | 原始证据核验结果 |
|---|---|
| relay暂停/恢复 | 17.26 / 19.74 仿真秒；实际丢弃25条扫描 |
| 末次接受扫描 | 17.20 仿真秒 |
| 最终精确零指令 | 17.70 仿真秒；扫描→零输出0.50仿真秒，诊断接收→指令接收0.480169075单调秒 |
| GT停车 | 17.74 仿真秒；扫描→GT停车0.54秒，hold→GT停车0.48秒 |
| 最大停车位移/稳定停车 | 0.0693146119 m / 2.00仿真秒；稳定阶段采样漂移0 |
| 定位任务计时 | paused=17.70，resumed=19.84；同一R0001:W00；诊断hold时长2.140658988单调秒 |
| 新扫描/恢复运动 | 19.80 / 19.84 仿真秒；原任务保持，无内部重发 |
| retry / stuck / false success | 0 / 0 / false |
| 完成事件 | GT=23.72仿真秒时匹配R0001:W00；完成时GT误差0.0387981410 m |
| 完整2+5秒 | 23.74–25.74 / 25.74–30.74，CSV与原生独立核验均通过 |
| 最终GT / odom误差 | 0.0392465650 / 0.0375845179 m |
| 最终GT yaw / 目标yaw | 1.5715659706 / 1.5707963268 rad |
| 7秒最大GT平移速度 / 最终指令 | 0.0187671751 m/s / 0，保持原阈值 |
| 7秒停车最大漂移 | 0.0000733041 m |
| 原生GT | 792条，50 Hz，最大仿真间隔0.020 s，最大单调接收间隔0.021689279 s |
| 最终指令 / ground odom最大接收间隔 | 0.044906959 / 0.047370010 s |
| 全程最小采样车体—墙间距 | 0.1883560284 m，boundary_5，sim=20.78，GT=(4.700948948,0.906285165)，yaw=1.568120263 rad |

没有采样间距低于0.08m、没有采样几何重叠。没有接触传感器证据，因此不能称独立碰撞检测通过。上述间距由所有原生GT与封存SDF车体/墙体几何重新计算，未插值；50Hz不构成连续时间安全证明。只有本次短距离样本，无统计重复性结论。

本次未出现 unsafe_plan；不需要关联风险轨迹或修改守卫。内部Bspline4条不等于外部目标重发或内部waypoint重试。任务计时转换有原始诊断证据；恢复后的私有计时状态不是独立订阅量，其语义另由封存运行源码及上轮实际生产逻辑隔离测试支持。

## 为什么总结果仍为FAIL

启动阶段保存了143条accepted定位诊断，但原始/guarded扫描记录从stamp=14.9s才开始。stamp=14.0、14.1、…、14.8的9条accepted诊断都有注入odom，找不到对应的原始及转发扫描。9条全在唯一目标发送前；行驶期129次扫描匹配能够从原始记录重新计算，残差差异0。

仍保留两项失败：localization_raw_evidence、loss_resume_raw_evidence。后者具体失败项 every_accepted_scan_was_forwarded；其他12项断流行为/停车/恢复检查通过。前者具体失败项 accepted_matches_have_raw_scan_and_raw_odom。未丢弃这些诊断、未制造缺失scan、未收窄原有匹配门槛，所以不能把实际动作正确升级为完整证据PASS。

这表明记录流的发现/开始范围不一致，尚不足以证明DDS发现、QoS队列或具体调度是唯一根因。接收序号、写入行数一致也不能证明发布端绝无丢包。

## 本轮只读工具修正

仅新增：
- tools/stage2/audit_scan_resume_v3.py
- tools/stage2/test_scan_resume_audit_v3.py
- tools/stage2/review_scan_resume_retry_fixed.py

不影响导航、定位输入、最终速度输出、planner或场地。上轮已修复的Stage2接口未在本轮修改；Stage1.5的92项封存输入、上轮11项源码和75项证据仍匹配。

原v1自动报告和规定命令生成的v2报告报TypeError并保留。原因是断流子审计遍历完整记录时将首条clock前的47条Twist、8条无header的actuation String的receive_sim_s=None用于数值比较。它们保留真实单调接收时间和原始字段，不是速度数据缺失。

v3仅在断流窗口子检查中明确排除这些首clock前的连续前缀，保持完整native审计原始输入；首clock后缺仿真时间、字段非有限值、窗口内非零输出仍不能通过，既不补零也不制造时间戳。首clock后第一条有时间关联的最终指令约19.12ms内收到。新增12项反例/证据测试通过；第一版测试错把非零之后重建稳定停车窗当成固定窗，后来改为核验确实应失败的exact_zero_until_new_scan，原失败日志保留，未改生产逻辑或阈值。

v3还独立核验paused/resumed身份、顺序、持续时间、原任务未重发及完成时间；task_clock诊断不计入扫描校正，143条accepted扫描、0条rejected扫描、2条状态诊断分开统计。两次v3复核的判据与断流计算完全一致，仍FAIL。

## 留档与复核

实际运行本地输入110项均有内容快照、路径、符号链接解析与SHA-256，运行前后及独立审计匹配；实际ROS参数、雷达SDF约定、Git HEAD/dirty diff、版本和启动命令已留档。原始文件哈希25项匹配；health和timing均包含在内。19类话题接收、写入与实际行数一致，队列峰值29/8192、无溢出/写盘关闭失败。系统依赖内容与动态dlopen插件未全部封存，不是完整系统镜像；目录里包含livox构建库不代表接入MID-360。

原始证据目录：tools/results/stage2_scan_resume_retry_fixed_20261001/
- independent_audit_v1.json、independent_audit_review_v2.json：保留报错
- independent_audit_review_v3.json：保留中间报错
- independent_audit_review_v3_final.json、independent_audit_review_v3_repeat.json：最终严格FAIL
- offline_analysis/analysis.json、native_gt_timeline.csv、stop_resume_timeline.png
- stage2_scan_resume_retry_fixed_01.native.jsonl、CSV、summary、events、plans、published_commands、actuation、native.health.json、native.timing.jsonl
- input_manifest.json、input_snapshot/、input_sha256_after.json、raw_data_sha256.json、runtime_parameters/

本轮工具留档：tools/results/stage2_scan_resume_retry_fixed_tools_20261001/，包括preflight/after verification、反例测试日志、缺失关联列表、报错栈、工具内容快照及final_evidence_seal.json。

可重复只读复核命令（--output必须是尚不存在的新文件）：
```bash
cd /home/rmnav/AeroMind
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/stage2/audit_scan_resume_v3.py \
  --result-dir tools/results/stage2_scan_resume_retry_fixed_20261001 \
  --output /tmp/scan_resume_retry_fixed_new_review.json
```

退出非零代表严格证据FAIL，本轮没有第二次Gazebo或第二个目标。错误frame、硬格式失效的live覆盖仍未执行。下一步仅离线修记录器启动同步与对应审计，不改变导航，也不自动执行Full/Return/ABC。
