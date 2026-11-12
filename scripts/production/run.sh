#!/usr/bin/env bash
# Use local_setup so a recorded legacy parent overlay cannot enter this process.
set -euo pipefail
production_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -f "$production_workspace/install-production/local_setup.bash" ]]; then
  echo 'Production install missing; run scripts/production/build.sh first.' >&2
  exit 2
fi
if [[ $# -eq 0 ]]; then
  echo 'Usage: scripts/production/run.sh COMMAND [ARGS...]' >&2
  exit 2
fi
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u VIRTUAL_ENV -u CONDA_PREFIX \
  PATH=/usr/bin:/bin:/opt/ros/humble/bin \
  bash --noprofile --norc -c '
    source /opt/ros/humble/setup.bash
    source "$1/install-production/local_setup.bash"
    cd -- "$1"
    shift
    exec "$@"
  ' production-run "$production_workspace" "$@"
