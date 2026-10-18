# MID-360 实车安装测量（只读）

先读 docs/real_mid360_mount_preparation_20261018.md。

模板缺值必须保留 null，不要把仿真偏移写入实车表。
check_measurements.py 只做离线候选计算，不发 ROS 数据，不改任何生产配置。

python3 -B tools/hardware/mid360_mount_check/tests.py
