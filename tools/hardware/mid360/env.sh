#!/usr/bin/env bash
set -eo pipefail
MID360_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
exec env -u LD_PRELOAD -u PYTHONPATH -u PYTHONHOME \
 -u QT_PLUGIN_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH \
 -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
 -u AEROMIND_DECLARED_UNBIASED_IMU -u AEROMIND_LIO_CONSISTENCY_FIX \
 -u AEROMIND_LIO_KEYFRAME_MAP -u AEROMIND_STATIC_SIM_STARTUP \
 -u AEROMIND_EXACT_LOCAL_GAUGE -u AEROMIND_INPUT_TRACE -u AEROMIND_LIO_TRACE \
 -u AEROMIND_PLANE_QUALITY_GUARD -u AEROMIND_INTERNAL_TIMING_DIR \
 ROS_DOMAIN_ID="${MID360_ROS_DOMAIN_ID:-43}" ROS_LOCALHOST_ONLY=1 \
 LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/usr/local/lib \
 PATH=/usr/bin:/bin:/usr/sbin:/sbin PYTHONNOUSERSITE=1 \
 /bin/bash --noprofile --norc -c '
set -eo pipefail
cd "$1"
source /opt/ros/humble/setup.bash
if [[ -f install_mid360/local_setup.bash ]]; then source install_mid360/local_setup.bash; fi
shift
exec "$@"
' mid360_env "$MID360_ROOT" "$@"
