#!/usr/bin/env bash
set -eo pipefail
GUARD_TOOLS="$(cd -- "$(dirname -- "$0")" && pwd)"
exec bash "$GUARD_TOOLS/../mid360_lio_phased/env.sh" python3 -B "$GUARD_TOOLS/session.py" "$@"
