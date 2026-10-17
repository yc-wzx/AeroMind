# Input conventions and sources (2026-10-17)

* User-confirmed wired addresses: host 192.168.1.5, MID360 192.168.1.173.
* Pinned local Livox driver2 src/lddc.cpp copies gyro/acc unchanged; the
  [manufacturer protocol](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)
  specifies rad/s and g respectively. The separate IMU adapter uses 9.80665,
  scales acceleration covariance by its square and marks orientation unavailable.
* [Official driver](https://github.com/Livox-SDK/livox_ros_driver2) supports
  PointCloud2 format 0, including per-point timestamp, line and tag. No format2
  XYZI conversion that would discard timing is used.
* The sensor-only JSON keeps driver extrinsic_parameter all zero. Raw points and
  IMU are not rotated. Inverted installation is not an internal LiDAR/IMU rotation.
* [Upstream author MID360 config](https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml)
  supplies the fixed internal LiDAR-in-IMU translation [-.011,-.02329,.04412]m,
  identity rotation, 4 lines, .5m blind range and default noise values. These are
  upstream nominal values, not measurements of this particular device.
* Our SPARK preprocess.cpp accepts PointCloud2 MID360 with enum 5. This differs
  from upstream FAST-LIO's CustomMsg enum 1. This package does not change either
  algorithm. LIO works in its initial sensor-local frame, not a ground/chassis
  navigation frame. No fabricated installation height, pivot or lever arm.
* The initial raw precheck is engineering screening: 15s actual receipts,
  >=10s span/overlap, point rate8..12Hz, IMU150..250Hz; receive/header max gaps
  .3s/.1s. These do not replace Stage1.5 navigation/parking acceptance thresholds.
  Sequence counters count received messages; they cannot prove no publisher loss.

Actual hardware data, timestamp epoch, gyro/acc noise and inverted mounting must
still be reviewed on the mini-PC. No real hardware was attached to the development
machine. No navigation, base_link frame, serial bridge or motor commands here.
