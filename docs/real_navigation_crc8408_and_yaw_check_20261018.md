# 真实导航 39B CRC8408 接收与云台方向检查（2026-10-18）

## 当前判断

用户实机报告显示 39B 导航反馈已被观察到：2999 帧 CRC、结构和相对角度有效，主机帧完成接收频率约 200.013 Hz，最大间隔约 5.283 ms。`nav_valid_flags=15`，速度为 0，`nav_allowed=0`；后者表示当前未允许导航运动，不是串口损坏。相对角度约 0.00997–0.01074 rad。静止时非零不能据此强制清零。

实机使用 CRC 反射多项式 0x8408、初值 FFFF、低字节在前；旧文档预期的 MODBUS 0xA001 无匹配。此前 `NAV39_DIAGNOSTIC_CRC_ONLY` 是旧预期与实际校验配置不符，不是“没有导航数据”。本轮仅把只读检查器的 CRC 预期设为显式参数，不修改电控或任何发送协议，也不自动回退。

实机目录：`/home/rm/AeroMind_xuc_feedback_checks/raw_20261018_130944/`。此处只有用户粘贴的报告；原始字节未传到开发机，未声称独立重算实机数据。原报告及其提供的哈希保留在 `docs/evidence/xuc_navigation_guided_20261018/physical_report_as_received.json`。

## 修改范围

仅修改 `tools/hardware/xuc_feedback_check/`：capture.py、inspect_wire.py，新增 guided_yaw.py 和 test_guided.py。串口仍通过 O_RDONLY 打开、460800/8N1、检查其他占用者、恢复原 termios、不清空原始字节；没有串口 TX 编码、ROS 发布或速度输出。没有修改导航、控制、planner、已封存原始雷达及 phased LIO 文件。

旧默认 0xA001 及旧报告保留；`--crc-profile 0x8408` 必须显式指定。历史数据可只读复核到新文件：

```bash
python3 -B tools/hardware/xuc_feedback_check/inspect_wire.py \
  /home/rm/AeroMind_xuc_feedback_checks/raw_20261018_130944 \
  --crc-profile 0x8408 --output-name independent_crc8408_review_v1.json
```

既有输出不覆盖。观察到校验匹配始终不等于已启用控制解码器，`navigation_ready=false` 保持。

## 下一次实机检查

底盘放着不动，先将云台对准你们规定的机械正前方。雷达驱动和 LIO 不是这次串口检查的必要输入。运行一次：

```bash
cd ~/AeroMind_20261016
git pull --ff-only
python3 -B tools/hardware/xuc_feedback_check/capture.py \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baud 460800 --crc-profile 0x8408 --guided-turn
```

自动提示共 25 秒，不用按回车：原位静止 5 秒 → 单向向左慢转约 45°用 5 秒 → 转后静止 5 秒 → 单向转回原位用 5 秒 → 原位静止 5 秒。不要来回调整。开始运动前会检查新鲜、有效的 39B 角度；若异常结束，停止手动转动并保留完整输出，不自动重试。串口被占用时拒绝启动，不停止其他程序。

每阶段时间来自实际主机单调时钟。独立审计从原始字节重新解析、检查标记顺序/覆盖、角度有效位和有限性、接收最大间隔 0.2 秒、左右变化方向。unwrap 假设相邻角度变化小于 π；5°仅为实际发生运动的门槛，不是精度阈值。约 45°是操作提示，没有独立角度真值。

输出包括 `guided_turn_check`、各次静止角度、左右变化与回原位差值。初始机械前向对应的编码角只记为候选，不写回零位配置；没有编码器精度、底盘朝向、时间同步或导航验收结论。

## 工具验证和证据

原 12 项测试和新增 7 项测试通过。新增覆盖显式 CRC、不自动回退、非零初值、跨 ±π、错误方向、缺值/间断及未就绪不提示运动。另用实际生产采集和提示逻辑完成一次 25 秒虚拟串口端到端检查，所有原始写入计数一致、无溢出/写入错误、对端未收到 TX payload，独立重算结果一致。

虚拟输入为脚本波形，不是实机编码器验收；只在测试中观察原始 start() 时间以对齐波形，不替换采集/判据。全部测试及虚拟报告存于 `docs/evidence/xuc_navigation_guided_20261018/`；开发机原始虚拟字节在 `tools/results/xuc_navigation_guided_20261018/virtual_guided_capture/`，不纳入分发仓库。

复现：

```bash
python3 -B tools/hardware/xuc_feedback_check/tests.py
python3 -B tools/hardware/xuc_feedback_check/test_guided.py
```

本轮完成的是工具适配。实机左右转和回原位检查等待用户执行；不发送导航目标或电机指令，不涉及自瞄。
