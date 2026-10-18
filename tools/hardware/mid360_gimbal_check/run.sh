#!/usr/bin/env bash
set -eo pipefail
CHECK_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$CHECK_TOOLS/../mid360_lio_phased/env.sh" python3 -B "$CHECK_TOOLS/record.py" "$@"
