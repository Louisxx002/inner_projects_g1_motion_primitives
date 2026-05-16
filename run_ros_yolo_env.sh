#!/usr/bin/env bash
set -euo pipefail

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-${USER}}"
mkdir -p "${MPLCONFIGDIR}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [[ $# -gt 0 ]]; then
  exec conda run -n g1-primitives-ros312 "$@"
fi

source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate g1-primitives-ros312
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash
