# LIO 与云台编码器联合只读检查

见 docs/real_mid360_joint_encoder_preview_20261018.md。
保留旧版本；不发布导航、TF或串口运动命令。
先启动真实MID-360及phased LIO，统一域43，再在机械前向开始25秒检查。

MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_joint_check/run.sh

独立审计 review.py <结果目录> --output-name <新文件名>，需phased env。
结果只表示诊断完整性，compensation_accuracy_accepted和navigation_ready始终false。
