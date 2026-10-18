# 导航反馈只读采集

基础说明：docs/real_navigation_feedback_receive_20261018.md。
真实 CRC8408 和 25 秒方向检查：docs/real_navigation_crc8408_and_yaw_check_20261018.md。

仅收取原始串口字节并离线解释39B导航字段；不含发送编码器或导航输出。
实际设备检查须显式使用 --crc-profile 0x8408；旧默认保留，不自动回退。
--guided-turn 只提示操作者转云台，不发送运动命令。

python3 -B tools/hardware/xuc_feedback_check/tests.py
python3 -B tools/hardware/xuc_feedback_check/test_guided.py
