#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
#
# Record a bag and run one floor-box pick. Observe pose = joints at
# driver start (--observe-pose current). Person on e-stop. No BlackOut.
#
# Pick graph must already be up:
#   ./scripts/hardware_pick.sh semantic_device:=cpu use_rviz:=false
#
#   ./scripts/record_floor_box_pick.sh              # plan-only + bag
#   ./scripts/record_floor_box_pick.sh execute       # PickSession execute
#
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "${DEPLOY}/.." && pwd)"
ENV_SH="${DEPLOY}/scripts/env_jazzy_real.sh"
MODE="${1:-plan}"
shift || true
OBSERVE_POSE="${OBSERVE_POSE:-current}"
STAMP="$(date +%Y%m%d_%H%M%S)"
EV="${EV:-${REPO}/docs/status/evidence/elfin_trajectory/${STAMP}_floor_box_pick}"
BAG_PID=""

mkdir -p "${EV}/bags"

cleanup() {
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

set +u
# shellcheck disable=SC1090
source "${ENV_SH}"
set -u

if ! ros2 pkg prefix luggage_planning >/dev/null 2>&1; then
  echo "luggage_planning not in overlay; source ${ENV_SH} after colcon build" >&2
  exit 1
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "evidence=${EV}"
echo "OBSERVE_POSE=${OBSERVE_POSE} (startup joints; no sim pickup_observe)"

"${DEPLOY}/scripts/dump_observe_pose.sh" > "${EV}/observe_startup.yaml" 2>"${EV}/observe_startup.err" || true
if [[ ! -s "${EV}/observe_startup.yaml" ]]; then
  echo "WARN: could not dump startup joints (graph up?)" >&2
  cat "${EV}/observe_startup.err" >&2 || true
fi

# record_all_topics.sh defaults domain 11; env already exported 7.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
stdbuf -oL -eL "${DEPLOY}/scripts/record_all_topics.sh" \
  -o "${EV}/bags" -n pick > "${EV}/bag_record.log" 2>&1 &
BAG_PID=$!
sleep 2
if ! kill -0 "${BAG_PID}" 2>/dev/null; then
  echo "bag record failed; see ${EV}/bag_record.log" >&2
  exit 1
fi
echo "bag pid=${BAG_PID}"

case "${MODE}" in
  plan|plan-only|--plan-only)
    ros2 run luggage_planning hardware_pick_driver.py \
      --observe-pose "${OBSERVE_POSE}" \
      --plan-only "$@" | tee "${EV}/plan_only.txt"
    ;;
  execute|pick)
    ros2 run luggage_planning hardware_pick_driver.py \
      --observe-pose "${OBSERVE_POSE}" \
      "$@" | tee "${EV}/execute.txt"
    ;;
  *)
    echo "usage: $0 [plan|execute] [driver flags...]" >&2
    exit 2
    ;;
esac
