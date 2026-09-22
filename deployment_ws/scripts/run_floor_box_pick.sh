#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
#
# Overlay + YOLO floor-box pick driver. Person on e-stop. ROS_DOMAIN_ID=7.
# Pick graph must already be up (./scripts/hardware_pick.sh).
#
# Observe pose (GoToRobotPose):
#   OBSERVE_POSE=current          # default: joints at driver start (no FJT)
#   OBSERVE_POSE=pickup_observe   # only after filling robot_poses.site.yaml
#
#   ./scripts/run_floor_box_pick.sh              # plan-only (no pick motion)
#   ./scripts/run_floor_box_pick.sh execute       # print waypoints, wait for yes
#   ./scripts/run_floor_box_pick.sh execute --yes # no stdin prompt (non-TTY)
#
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_SH="${DEPLOY}/env_jazzy_real.sh"
MODE="${1:-plan}"
shift || true
OBSERVE_POSE="${OBSERVE_POSE:-current}"

set +u
# shellcheck disable=SC1090
source "${ENV_SH}"
set -u

if ! ros2 pkg prefix luggage_planning >/dev/null 2>&1; then
  echo "luggage_planning not in overlay. Build in the repo root:" >&2
  echo "  colcon build --symlink-install --packages-select \\" >&2
  echo "    luggage_msgs elfin_description elfin_control elfin_moveit_config \\" >&2
  echo "    luggage_description luggage_perception luggage_planning" >&2
  echo "  then source ${ENV_SH}" >&2
  exit 1
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "luggage_planning=$(ros2 pkg prefix luggage_planning)"
echo "OBSERVE_POSE=${OBSERVE_POSE}"

case "${MODE}" in
  plan|plan-only|--plan-only)
    exec ros2 run luggage_planning hardware_pick_driver.py \
      --observe-pose "${OBSERVE_POSE}" --plan-only "$@"
    ;;
  execute|pick)
    exec ros2 run luggage_planning hardware_pick_driver.py \
      --observe-pose "${OBSERVE_POSE}" "$@"
    ;;
  *)
    echo "usage: $0 [plan|execute] [driver flags...]" >&2
    echo "  plan     detect + print waypoints, no PlanMotion" >&2
    echo "  execute  detect + human yes (or pass --yes)" >&2
    echo "  OBSERVE_POSE=current|pickup_observe (default current)" >&2
    exit 2
    ;;
esac
