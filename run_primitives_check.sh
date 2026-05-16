#!/usr/bin/env bash
set -euo pipefail

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-${USER}}"
mkdir -p "${MPLCONFIGDIR}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/unitree_g1_primitives"
exec conda run -n "${G1_PRIMITIVES_ENV:-g1-primitives}" bash preflight_check.sh
