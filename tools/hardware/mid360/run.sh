#!/usr/bin/env bash
set -eo pipefail
MID360_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$#" != 0 ]]; then echo "Usage: bash $0 (sensors only; LIO remains gated until raw capture review)" >&2; exit 2; fi
exec bash "$MID360_TOOLS/env.sh" bash --noprofile --norc -c '
set -eo pipefail
python3 -B tools/hardware/mid360/preflight.py --live
exec ros2 launch tools/hardware/mid360/real_mid360.launch.py lio:=false rviz:=true
'
