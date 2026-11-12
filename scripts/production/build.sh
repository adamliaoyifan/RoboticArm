#!/usr/bin/env bash
# Rebuild the production closure without inheriting another workspace overlay.
set -euo pipefail
production_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u VIRTUAL_ENV -u CONDA_PREFIX \
  PATH=/usr/bin:/bin:/opt/ros/humble/bin \
  bash --noprofile --norc -c '
    source /opt/ros/humble/setup.bash
    export CMAKE_BUILD_PARALLEL_LEVEL=2
    exec /usr/bin/python3 "$1/scripts/production/build.py" "${@:2}"
  ' production-build "$production_workspace" "$@"
