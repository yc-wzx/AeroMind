#!/usr/bin/env bash
# Build the accepted simulation without Livox hardware SDK or local recordings.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
JOBS="${BUILD_JOBS:-2}"
if ! [[ "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo 'BUILD_JOBS must be a positive integer.' >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble on Ubuntu 22.04 first; see docs/GITHUB_TRANSFER.md.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
cd "$ROOT"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
export MAKEFLAGS="-j$JOBS"
# Explicit src discovery avoids duplicate package names in historical snapshots.
# uav_bringup has a hardware Livox dependency; the simulation needs no SDK.
colcon build --base-paths "$ROOT/src" --symlink-install \
  --executor sequential --packages-select \
  quadrotor_msgs traj_utils plan_env path_searching bspline_opt ego_planner \
  spark_fast_lio uav_planning uav_bringup stage2_lio_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$ROOT/install/setup.bash"
python3 -B "$ROOT/tools/deployment/install_training_profile.py" --jobs "$JOBS"
python3 -B "$ROOT/tools/deployment/check_training_install.py"
echo 'Build and install checks passed. Start command: see docs/GITHUB_TRANSFER.md.'
