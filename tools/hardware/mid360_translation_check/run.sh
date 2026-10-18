#!/usr/bin/env bash
set -eo pipefail
TRANSLATION_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$TRANSLATION_TOOLS/../mid360_lio_phased/env.sh" python3 -B "$TRANSLATION_TOOLS/translation_record.py" "$@"
