# MID-360 实车安装测量（只读）

先读 docs/real_mid360_mount_preparation_20261018.md。

模板缺值必须保留 null，不要把仿真偏移写入实车表。
check_measurements.py 只做离线候选计算，不发 ROS 数据，不改任何生产配置。

python3 -B tools/hardware/mid360_mount_check/tests.py

操作者暂定数据：mount_measurements.operator_20261018_v1.json（25mm/550mm为粗测）。
离线圆心预览：preview_axis_centre.py /已有通过的转出返回目录。
本轮串口/自瞄格式审查：docs/real_mid360_operator_mount_and_serial_review_20261018.md。
相对角度反馈尚未实收，不能认为当前自瞄原版30B已支持目标39B协议。
