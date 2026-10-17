#!/usr/bin/env bash
set -eo pipefail
PHASE_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$PHASE_TOOLS/../mid360/env.sh" bash --noprofile --norc -c '
set -eo pipefail
TASK_JOBS="${BUILD_JOBS:-2}"
[[ "$TASK_JOBS" =~ ^[1-9][0-9]*$ ]] || exit 2
export CMAKE_BUILD_PARALLEL_LEVEL="$TASK_JOBS" MAKEFLAGS="-j$TASK_JOBS"
python3 -B tools/hardware/mid360_lio_phased/variant.py prepare
colcon --log-base log_mid360_lio_phased build \
 --base-paths build_mid360_lio_phased/src/spark_fast_lio --packages-select spark_fast_lio \
 --executor sequential --build-base build_mid360_lio_phased/build --install-base install_mid360_lio_phased \
 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install_mid360_lio_phased/local_setup.bash
cmake -S tools/hardware/mid360_lio_phased/probe -B build_mid360_lio_phased/probe
cmake --build build_mid360_lio_phased/probe -j"$TASK_JOBS"
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 build_mid360_lio_phased/probe/phase_output_probe
python3 -B tools/hardware/mid360_lio_phased/test_audit.py
python3 -B tools/hardware/mid360_lio_phased/variant.py seal
'
