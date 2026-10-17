#!/usr/bin/env bash
set -eo pipefail
MID360_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$MID360_TOOLS/env.sh" bash --noprofile --norc -c '
set -eo pipefail
TASK_JOBS="${BUILD_JOBS:-2}"
[[ "$TASK_JOBS" =~ ^[1-9][0-9]*$ ]] || { echo "BUILD_JOBS must be positive" >&2; exit 2; }
[[ -f /usr/local/include/livox_lidar_api.h && -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]] || { echo "Livox-SDK2 is required in /usr/local; existing simulation install will not be changed" >&2; exit 3; }
export CMAKE_BUILD_PARALLEL_LEVEL="$TASK_JOBS" MAKEFLAGS="-j$TASK_JOBS"
colcon --log-base log_mid360 build \
 --base-paths src/third_party/livox_ros_driver2 src/third_party/spark-fast-lio/spark_fast_lio \
 --packages-select livox_ros_driver2 spark_fast_lio --executor sequential \
 --build-base build_mid360 --install-base install_mid360 \
 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install_mid360/local_setup.bash
cmake -S tools/hardware/mid360/production_preprocess_probe -B build_mid360/production_preprocess_probe
cmake --build build_mid360/production_preprocess_probe -j"$TASK_JOBS"
build_mid360/production_preprocess_probe/mid360_preprocess_probe
python3 -B tools/hardware/mid360/preflight.py --record-build
python3 -B tools/hardware/mid360/tests.py
'
