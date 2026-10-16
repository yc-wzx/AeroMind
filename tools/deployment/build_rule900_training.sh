#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$PROJECT_ROOT/tools/deployment/rule900_env.sh" bash --noprofile --norc -c '
set -eo pipefail
TASK_JOBS="${BUILD_JOBS:-2}"
[[ "$TASK_JOBS" =~ ^[1-9][0-9]*$ ]] || { echo "BUILD_JOBS must be positive" >&2; exit 2; }
export CMAKE_BUILD_PARALLEL_LEVEL="$TASK_JOBS" MAKEFLAGS="-j$TASK_JOBS"
mapfile -t PACKAGE_PATHS < <(colcon list --base-paths src --packages-select \
  quadrotor_msgs traj_utils plan_env path_searching bspline_opt ego_planner \
  spark_fast_lio uav_planning uav_bringup stage2_lio_sim provincial_rule900_scratch_sim --paths-only)
[[ "${#PACKAGE_PATHS[@]}" == 11 ]] || { echo "Expected eleven source packages" >&2; exit 3; }
colcon build --paths "${PACKAGE_PATHS[@]}" --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
# Retain the legacy entry and build its existing optional runtime too.
python3 -B tools/deployment/install_training_profile.py --jobs "$TASK_JOBS"
python3 -B tools/deployment/rule900_deployment.py build --jobs "$TASK_JOBS"
python3 -B tools/deployment/rule900_deployment.py check
echo "Build checks complete. Start: bash tools/deployment/run_rule900_training.sh"
'
