#!/usr/bin/env bash
# Exclusive non-scored ST09 rehearsal. Domain 7, no GUI/RViz, 20 minutes.
#
# The install prefix must be a colcon install root (the default
# --install-base), not a named overlay such as install/g4. Gazebo resolves
# resources through that prefix.
#
#   colcon build --install-base /path/install --symlink-install \
#     --packages-up-to luggage_gazebo
#   scripts/frame_migration_st09_rehearsal.sh --install /path/install --out DIR
set -euo pipefail

PRIMARY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL="${INSTALL:-$PRIMARY/install}"
OUT=""
FAULT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install) INSTALL="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --fault) FAULT="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT" ]] || { echo "--out required" >&2; exit 2; }
[[ -f "$INSTALL/setup.bash" ]] || {
  echo "default install prefix missing: $INSTALL/setup.bash" >&2
  echo "colcon build --install-base $INSTALL --symlink-install --packages-up-to luggage_gazebo" >&2
  exit 2
}

export ROS_DOMAIN_ID=7
export ELFIN_SIM_PIDFILE="${ELFIN_SIM_PIDFILE:-/tmp/elfin_humble_sim.pid}"
export YOLO_OFFLINE=1
export ULTRALYTICS_OFFLINE=1
export YOLO_AUTOINSTALL=false

unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# Isolated frame prefixes do not contain elfin_description. Overlay the
# complete g4 robot install first, then this prefix so the live luggage_*
# copies (probe timeout, v2 adapter) win.
if [[ ! -d "$INSTALL/elfin_description" && -f "$INSTALL/g4/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$INSTALL/g4/setup.bash"
fi
# shellcheck disable=SC1091
source "$INSTALL/setup.bash"
set -u

cmd=(python3 "$PRIMARY/scripts/frame_migration_st09_rehearsal.py"
  --install "$INSTALL" --out "$OUT")
if [[ -n "$FAULT" ]]; then
  cmd+=(--fault "$FAULT")
fi
exec "${cmd[@]}"
