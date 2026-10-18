# 导航反馈只读采集

见 docs/real_navigation_feedback_receive_20261018.md。

仅收取原始串口字节并离线解释39B导航字段；不含发送编码器或导航输出。

python3 -B tools/hardware/xuc_feedback_check/tests.py
