# Stage2 定位失效等待：内部重试隔离修复

本轮指令已完成。状态为 **STAGE2 LOCALIZATION-HOLD RETRY ISOLATED FIX VERIFIED**，仅表示隔离生产逻辑及工具核验通过。`live_new_fix_verified=false`；本轮 Gazebo 启动次数和实际外部目标发送次数均为0。

仍为 TRAINING-ONLY / PROVISIONAL；COMPETITION ARENA NOT VERIFIED；CONTACT DATA UNAVAILABLE。没有接入MID-360、IMU、LIO，没有运行Full/Return/ABC，没有调整planner、executor、clearance、inflation、车体、速度、地图、collision、物理步长或传感器频率。

## 修复前后

在隔离状态中执行真实 `LocalizedInterface.update()` 及其继承的完整生产方法，保留真实ROS消息、实际几何守卫和任务方法；仅将发布端替换为内存收集器、时钟替换为可控测试时钟。未执行rclpy.init，没有运行ROS节点，也没有向在线链路注入目标/速度。

条件：已有R0001:W00、一个pending waypoint、任务和无进展计时超过原门槛、低速、定位失效、原始odom及非零上游指令仍新鲜。

| 项目 | 修复前 | 修复后 |
|---|---|---|
| 最终速度 | vx=vy=wz=0 | vx=vy=wz=0 |
| 内部目标实际发布次数 | 1 | 0 |
| stage_retry_count | 1 | 0 |
| odom发布次数 | 1 | 1 |
| 原route/waypoint及队列 | R0001/W00、pending=1 | 保持 |

证据为 `counterexample_before.json` 和 `counterexample_after_final.json`。早期中间版本的after记录亦保留，未覆盖。

## 生产代码的修改

唯一修改的生产节点是 `src/uav_planning/scripts/stage2_localized_interface.py`。其安装入口为源文件符号链接，内容一致，无需重建。Stage1.5的92项封存输入和原有入口保持匹配；`gazebo_navigation_interface.py`、planner、executor和地图未修改。

新增逻辑：

- 每次update读取一次定位新鲜状态，失效期间继续复用现有传感器watchdog输出零速度。
- 用route、waypoint、目标对象及坐标识别当前阶段。失效等待时暂停阶段计时；恢复同一任务时重启原有阶段/无进展窗口，不重发目标、不重置retry_count、不清除轨迹。原本为None的耗尽状态不被重新激活。
- 在冻结基类已有的 `observe_guard_resume` 钩子处，使用仅由Stage2 wrapper捕获的私有暂停信号，结束本次任务尾部。这发生在最终速度和odom发布之后、完成/重试/恢复分支之前，避免失效时误报到达、内部重试或跳过阶段。没有隐藏active waypoint，也没有删除完成日志来制造成功；失效期间根本不执行完成分支。
- 定位无效时暂不发送pending目标，也不使用旧扫描重新排队active waypoint；最终仍为零。定位恢复后原动态障碍检查与原安全守卫正常执行。
- 任务身份变更时丢弃旧暂停记录，不能把旧计时恢复到新route/waypoint。增加只读 `localization_task_clock` 转换事件，记录paused/resumed/discard_previous_task与时间。

**速度影响范围**：没有修改最终速度公式、限制或安全阈值，失效时仍由原生产watchdog发布精确零。修改了Stage2任务计时、失效期间的完成/dispatch许可和恢复时序；由于stage_sent_at也参与现有低速收尾条件，恢复后的收尾启用时间可能随计时重启改变。不能声称新旧整个运动过程逐tick完全相同；实际运动效果仍需下一轮唯一一次短路线实跑。

## 测试与工具

最终联合测试77项通过：25项新生产任务测试、8项已有输入保护、9项定位测试、15项扫描中继/刺激/证据测试、4项原始目标反例、4项运行入口拒绝测试、12项完整终点评估反例。

覆盖长时间失效无内部重发、非零旧上游指令仍停车、hard fault和错误frame、恢复后缺失/过期/不安全计划仍停车、有效新计划允许输出、恢复后真正无进展仍超过原4秒/2秒条件而实际重试、重复失效/恢复只在转换时重新计时、初始idle/完成、临近终点不能失效误完成、新目标回调和waypoint切换、耗尽状态、动态障碍仍正常处理、非预期异常传播与sensor状态恢复。

已有 `tools/stage2/test_localization.py` 仅补齐三个简化夹具所缺的节点状态字段，没有修改原断言或验收门槛。早期夹具缺值导致的失败日志保留。最终77项联合测试前封存41项本地Python依赖内容及哈希，之后内容仍匹配。

运行工具 `tools/stage2/run_scan_resume_experiment.py` 增加显式 `--output-dir` / `--trial-id`，拒绝已有目录、非法trial ID和非zero profile；没有自动换目录或重试。选用已纠正短目标坐标的v2独立审计，避免继承Full默认坐标。本轮只隔离验证入口，没有执行其真实run。

新增工具：`localization_hold_fixture.py`、`reproduce_localization_hold_retry.py`、`test_localization_hold_retry.py`、`test_scan_resume_entry.py`、`review_hold_unsafe_plan.py`、`verify_localization_hold_retry_fix.py`，均位于 `tools/stage2/`。

## 历史FAIL与unsafe_plan保留

上一轮唯一实跑以只读方式重新审计，输出本轮新增 `historical_reaudit_v3.json`。实际停止/恢复与终点2+5s仍有证据，但原始retry_count=1仍使整体FAIL。新代码未在该历史运行中使用，不能追认历史通过。当前文件与历史运行输入的预期差异为源/安装Stage2接口及运行器三项；没有把本轮补算哈希冒充历史运行输入。

对原生6条Marker逐点重新生成B-spline采样，全部唯一匹配对应原始Bspline消息，最大采样坐标差为0。危险计划关联到 **traj_id=6**、start_time=19.58s、duration≈13.6063883s。Marker自己的ID始终为0，轨迹关联依据全部坐标重建，不能把Marker ID当作trajectory ID。

采用原生最新先到的ground odom上下文（stamp=19.56s，yaw≈1.5721042rad）及适配后目标，调用未修改的生产 `planned_trajectory_callback()`，保留实际未来yaw规则。得到planned min gap=0 / safe=false，与19.60/19.70s原始unsafe_plan诊断和零输出一致。最近SDF边界为boundary_7，第133点约(4.699086,1.940189)，未来yaw≈90°。实际GT采样最小间距仍为旧值 .188905667m；计划风险不等于实际接触。

限制：上下文来自记录器先到消息，不是完整录制的导航回调内部状态；目标header为 `/ground/planning/goal` 适配后的header，未单独录制 `/navigation/segment_goal`。该生产规则的上下文重放不证明精确原回调顺序，也不能仅凭一次关联断言planner生成危险形状的完整因果。本轮不修改planner，也不绕过unsafe_plan。

## 结果路径及复核

本轮全部结果：`tools/results/stage2_localization_hold_retry_fix_20261001/`。包括input_before、test_input_snapshot、41项前后哈希、before/after反例、联合77项日志、各相关测试日志、历史reaudit、unsafe_plan_review_v1.json、changes.patch、Git状态、fix_verification.json及final_evidence_seal.json。

独立入口（输出必须不存在）：

```bash
cd /home/rmnav/AeroMind
python3 -B tools/stage2/verify_localization_hold_retry_fix.py \
  --output /tmp/localization_hold_fix_NEW.json
```

本轮没有新增运行记录、没有接触数据或统计重复性证明，也不代表正式省赛验收。下一轮prompt另存为 `docs/stage2_localization_hold_retry_next_live_prompt_20261001.md`，本轮不执行它。
