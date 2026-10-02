# Stage 2 第4项：闭环预检失败与参数读取工具修复

**本轮闭环验收未完成，第4项仍未完全通过。**
TRAINING-ONLY / PROVISIONAL · COMPETITION ARENA NOT VERIFIED · CONTACT DATA UNAVAILABLE

## 本轮实际执行

使用上轮修复的模型验证运行器，启动且仅启动一个独立Gazebo。离线预检通过：参考长度7.75m、GridRoute clearance=0.40m、yaw90矩形采样间距0.14m。后者为离线参考检查，不是新Full轨迹间距。

运行期成功读取/lio_mapping参数，确认3D模式、acc_cov=1e-6、gyr_cov=1e-8、两项偏置随机游走为0。随后/lio_sensor_adapter/list_parameters请求8秒超时，运行器保留错误并结束。launch.log同时记录服务端“failed to send response (timeout)”。没有重发请求以跳过本轮预检，没有启动第二次Gazebo。

外部目标次数为0，没有route接受或完成事件。独立审计为INCOMPLETE_EVIDENCE。导航Full、boundary_7、内部航点及完整2+5秒停车窗口均NOT_EXERCISED，不记为planner失败，也不能宣称已通过。

运行时实际输入快照177项，结束后内容及符号链接目标无变化。原始哈希清单22文件一致，native/sensor健康通过，接收、写入计数与实际文件行数一致。当前原始哈希清单未递归纳入runtime_parameters；本轮最终封存另外包含参数响应与query文件，不能追认为原始运行器已经封存这些文件。

## 本轮记录的启动期事实

GT494条，原生时间间隔最大约0.020s，单调接收最大间隔0.022391s，墙钟接收约50.01Hz；最终Twist494条，最大单调接收间隔0.038529s，约50.00Hz。最终Twist没有自身时间戳，不伪造消息生成时间。

记录范围约sim=0.02至9.88s，最终指令各轴最大绝对值0.0，GT距出生点最大偏差0.0m。这只支持“未下发目标时保持停车”，不能代替终点或全程安全审计。IMU原始2470条、适配2448条等差异保留，不由序号声称绝无丢包。

## 排查和最小工具改进

1. 在DDS域76隔离启动实际生产SensorAdapter，不输入传感器、GT或导航数据；旧读取器连续6次成功，现场问题未被该条件复现。不能宣称发现唯一根因。
2. 新增独立tools/stage2/lio_model_validation_v2/，旧运行器、工具和输入保持原样。新参数读取器预创建只读服务客户端，等待服务发现并推进响应发现，核对唯一节点身份；请求8秒响应时限保持，单服务最多3次、整个节点读取总预算20秒，失败请求取消，不复用陈旧future。
3. 每次查询保存服务名、次数、单调时间及结果；持续超时仍拒绝，参数缺失、NaN或数量不一致不补零、不输出成功YAML。重试仅针对只读参数服务，不重发导航目标，不set参数。
4. 同目录v2运行器维持唯一外部目标和全部已有门槛，增加实际参数及查询文件的发送目标前哈希留档；v2审计新增参数证据哈希、参数门槛检查，原严格审计不放宽。

在DDS域77，刻意延迟首个响应8.3秒：旧读取器失败，新读取器第二次收到正确匹配响应。持续延迟、NaN、缺失值均不能成功；真实SensorAdapter读取成功。共11项检查通过。该延迟是工具鲁棒性反例，不是对现场DDS超时原因的证明。

另有4项实际v2审计入口的参数哈希变异测试通过：正常证据、参数文件篡改、缺少manifest、重算哈希后的失败gate。原始导航审计在这4项测试中由fixture替代，只验证新增证据守卫，不冒称完整任务审计测试。

## 修改与归档

- 仅新增工具：tools/stage2/lio_model_validation_v2/、tools/stage2/lio_parameter_diagnostics/。
- 没有修改导航、最终速度、planner、LIO算法、LIO配置、车体、速度、安全距离、地图、物理步长或传感器频率。
- 本轮证据：tools/results/stage2_step4_lio_live_noise_model_20261002/。
- Full启动失败证据：full_01/progress.json、runtime_parameters/lio_sensor_adapter.query.json、launch.log、independent_audit_v1.json、input_snapshot/及全部native/sensors原始记录。
- 旧读取器隔离结果：parameter_service_probe_01/。
- 11项查询检查：parameter_reader_v2_tests_01/verification.json。
- 4项哈希变异检查：parameter_hash_guard_tests.log。
- 结构化结论：round_review.json；封存：final_evidence_seal.json。
- 上轮192项源文件、455项证据哈希复核无差异；其中原Stage1.5、步骤1–3和初版步骤4基线继续保留。

## 下一步

第4项仍需一次新的真实Full。使用v2工具入口和新的目录，先成功封存所有运行参数、确认唯一导航接收端、真实GT离线预检与传感器新鲜度，再只发(8.7,4.25), yaw90一个目标。仍采用全部内部waypoint、GT/odom、完整2+5秒及原生最小间距独立审计。此工具在真实Gazebo负载下是否解决参数响应问题尚未验证。

不将上轮离线4.75mm定位偏差当作新导航到点误差；保留旧Full完成事件5cm判据FAIL。没有接触、连续时间安全或统计重复性证据，不进入真实硬件第5项。
