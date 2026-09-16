#!/usr/bin/env bash
# Non-scored sim_world startup rehearsal loop.
#
# Diagnoses the gz_ros2_control robot_description race that stopped PF-R7
# generations 7, 8 and 9 before a single case was attempted. It scores nothing
# and writes no campaign verdict, so it is not bound by the one-scored-live
# budget; it exists precisely so startup wiring is fixed by repetition instead
# of by burning acceptance slots.
#
# Usage:
#   scripts/sim_startup_rehearsal.sh --iterations 10 --out <dir>
#   OVERLAY=/tmp/pfr10_g6 scripts/sim_startup_rehearsal.sh --iterations 3 ...
set -euo pipefail

PRIMARY="$(git rev-parse --show-toplevel)"
OVERLAY="${OVERLAY:-$PRIMARY}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ELFIN_SIM_PIDFILE="${ELFIN_SIM_PIDFILE:-/tmp/elfin_humble_sim.pid}"
# The rehearsal must never stall on an Ultralytics network install.
export YOLO_OFFLINE=1
export ULTRALYTICS_OFFLINE=1
export YOLO_AUTOINSTALL=false

slot_busy() {
  pgrep -f '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null && return 0
  pgrep -f 'scripts/place_only_run.sh' >/dev/null && return 0
  pgrep -f 'place_only_eval_driver.py' >/dev/null && return 0
  return 1
}

if slot_busy; then
  echo "sim slot busy; refusing to start a second world" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$OVERLAY/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$OVERLAY/install/setup.bash"
fi
set -u

"$PRIMARY/scripts/stop_sim.sh" >/dev/null 2>&1 || true

exec python3 "$PRIMARY/scripts/sim_startup_rehearsal.py" \
  --ros-domain-id "$ROS_DOMAIN_ID" \
  --pidfile "$ELFIN_SIM_PIDFILE" \
  "$@"
