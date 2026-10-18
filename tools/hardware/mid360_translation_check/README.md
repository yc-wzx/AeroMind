# 实测距离参考的手推直线去回

见 docs/real_mid360_translation_tool_20261018.md。
保留旧云台转动工具；只读订阅与串口接收，不发布电机、导航或TF数据。
必填 --distance-m，使用卷尺实际测得的米数，不默认0.5。
按终端提示回车标记A就位、开始推、B到点、开始返回、A到点；三次静止各5秒。
诊断完整不等于精度验收，accuracy_accepted和navigation_ready保持false。

MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_translation_check/run.sh --distance-m 实测米数
