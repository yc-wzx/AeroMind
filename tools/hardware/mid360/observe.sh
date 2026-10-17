#!/usr/bin/env bash
set -eo pipefail
MID360_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$MID360_TOOLS/env.sh" python3 -B "$MID360_TOOLS/capture.py" "$@"
