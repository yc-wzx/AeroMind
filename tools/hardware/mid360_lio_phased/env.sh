#!/usr/bin/env bash
set -eo pipefail
PHASE_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$PHASE_TOOLS/../mid360/env.sh" bash --noprofile --norc -c '
set -eo pipefail
source install_mid360_lio_phased/local_setup.bash
exec "$@"
' mid360_phase_env "$@"
