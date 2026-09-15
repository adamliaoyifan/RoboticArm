#!/usr/bin/env bash
# Real-cell pick graph. Person on e-stop. Ctrl+C stops the launch
# (servo disable + TCP drop; does not BlackOut / cut 48 V).
#
#   source ./scripts/site_env.sh
#   export ROS_DOMAIN_ID=7    # you choose
#   ./scripts/hardware_pick.sh
#   ./scripts/hardware_pick.sh semantic_device:=cpu
#   ./scripts/hardware_pick.sh use_rviz:=true semantic_device:=cpu
#
# Other terminal, same ROS_DOMAIN_ID after sourcing site_env.sh:
#   ros2 run luggage_planning hardware_pick_driver.py --detect-only
#   ros2 run luggage_planning hardware_pick_driver.py --plan-only
#   ros2 run luggage_planning hardware_pick_driver.py
#   ros2 run luggage_planning hardware_pick_driver.py --release
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$DEPLOY/scripts/site_env.sh"

if ! python3 -c "import moveit_msgs" >/dev/null 2>&1; then
  echo "WARNING: MoveIt not installed. --detect-only works; plan/pick need:" >&2
  echo "  sudo apt install ros-jazzy-moveit-msgs ros-jazzy-moveit-configs-utils \\" >&2
  echo "    ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl \\" >&2
  echo "    ros-jazzy-moveit-kinematics ros-jazzy-moveit-simple-controller-manager" >&2
fi

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
