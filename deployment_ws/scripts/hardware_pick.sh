#!/usr/bin/env bash
# Real-cell pick graph. Person on e-stop. Ctrl+C stops the launch
# (servo disable + TCP drop; does not BlackOut / cut 48 V).
#
#   ./scripts/hardware_pick.sh
#   ./scripts/hardware_pick.sh semantic_device:=cpu
#   ./scripts/hardware_pick.sh use_rviz:=true semantic_device:=cpu
#
# Other terminal, same ROS_DOMAIN_ID:
#   ros2 run luggage_planning hardware_pick_driver.py --detect-only
#   ros2 run luggage_planning hardware_pick_driver.py --plan-only
#   ros2 run luggage_planning hardware_pick_driver.py
#   ros2 run luggage_planning hardware_pick_driver.py --release
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
HUMBLE_WS="${REPO}/elfin_humble_ws"
SDK="${REPO}/third_party/huayan_python_sdk"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "need /opt/ros/jazzy" >&2
  exit 1
fi
if [[ ! -f "$DEPLOY/install/setup.bash" ]]; then
  echo "build deployment_ws: colcon build --packages-select elfin_trajectory_executor" >&2
  exit 1
fi
if [[ ! -f "$HUMBLE_WS/install/setup.bash" ]]; then
  echo "missing ${HUMBLE_WS}/install/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "$DEPLOY/livox_ws/env.sh" ]]; then
  # shellcheck disable=SC1091
  source "$DEPLOY/livox_ws/env.sh"
fi
# shellcheck disable=SC1091
source "$HUMBLE_WS/install/setup.bash"
# shellcheck disable=SC1091
source "$DEPLOY/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_LOCALHOST_ONLY
export PYTHONPATH="${SDK}${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

if ! python3 -c "import moveit_msgs" >/dev/null 2>&1; then
  echo "WARNING: MoveIt not installed. --detect-only works; plan/pick need:" >&2
  echo "  sudo apt install ros-jazzy-moveit-msgs ros-jazzy-moveit-configs-utils \\" >&2
  echo "    ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl \\" >&2
  echo "    ros-jazzy-moveit-kinematics ros-jazzy-moveit-simple-controller-manager" >&2
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "hardware pick launch. e-stop person required. Ctrl+C stops nodes (no BlackOut)."

# ros2 launch wants foo:=bar, not --foo:=bar. A lone "--" is also dropped.
launch_args=()
for arg in "$@"; do
  if [[ "$arg" == "--" ]]; then
    continue
  elif [[ "$arg" == --*:=* ]]; then
    launch_args+=("${arg#--}")
  else
    launch_args+=("$arg")
  fi
done

exec ros2 launch luggage_planning hardware_pick.launch.py "${launch_args[@]}"
