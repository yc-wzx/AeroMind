#!/usr/bin/env bash
set -eo pipefail
JOINT_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$JOINT_TOOLS/../mid360_lio_phased/env.sh" python3 -B "$JOINT_TOOLS/capture_joint.py" "$@"
