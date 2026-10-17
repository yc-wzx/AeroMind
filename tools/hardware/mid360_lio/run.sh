#!/usr/bin/env bash
set -eo pipefail
LIO_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$#" != 1 ]]; then echo "Usage: bash $0 /path/to/real_raw_capture/summary.json" >&2; exit 2; fi
exec bash "$LIO_TOOLS/../mid360/env.sh" python3 -B "$LIO_TOOLS/start.py" "$1"
