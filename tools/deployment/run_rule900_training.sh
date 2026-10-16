#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="${1:-run}"
[[ "$#" -le 1 && ( "$MODE" == run || "$MODE" == --check-only || "$MODE" == --headless ) ]] || { echo "Usage: bash $0 [--check-only|--headless]" >&2; exit 2; }
exec bash "$PROJECT_ROOT/tools/deployment/rule900_env.sh" bash --noprofile --norc -c '
set -eo pipefail
python3 -B tools/deployment/rule900_deployment.py check
if [[ "$1" == --check-only ]]; then exit 0; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${IGN_PARTITION:-aeromind_rule900_training}"
export QT_QPA_PLATFORM=xcb
if [[ "$1" == --headless ]]; then GUI=false; else GUI=true; fi
exec ros2 launch provincial_rule900_scratch_sim provincial_rule900_training.launch.py \
  navigation:=true gui:="$GUI" rviz:="$GUI" auto_goal:=false
' rule900_run "$MODE"
