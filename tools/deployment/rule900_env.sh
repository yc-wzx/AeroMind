#!/usr/bin/env bash
# Process-local clean environment; does not edit MVS, shell startup or networking.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec env -u LD_PRELOAD -u PYTHONPATH -u PYTHONHOME \
  -u QT_PLUGIN_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH \
  -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin PYTHONNOUSERSITE=1 \
  /bin/bash --noprofile --norc -c '
set -eo pipefail
cd "$1"
source /opt/ros/humble/setup.bash
if [[ -f install/local_setup.bash ]]; then source install/local_setup.bash; fi
shift
exec "$@"
' rule900_clean "$PROJECT_ROOT" "$@"
