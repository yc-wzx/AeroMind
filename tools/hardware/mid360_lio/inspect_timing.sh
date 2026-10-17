#!/usr/bin/env bash
set -eo pipefail
LIO_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$LIO_TOOLS/../mid360/env.sh" python3 -B "$LIO_TOOLS/inspect_timing.py" "$@"
