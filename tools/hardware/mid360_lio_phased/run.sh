#!/usr/bin/env bash
set -eo pipefail
PHASE_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$PHASE_TOOLS/env.sh" python3 -B "$PHASE_TOOLS/start.py" "$@"
