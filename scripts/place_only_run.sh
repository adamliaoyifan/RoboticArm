#!/usr/bin/env bash
# POS-1 place-only streak runner (docs/plans/place_only_perfect_geometry_sim.md).
#
# Owns the simulator lifecycle around the eval driver:
#   1. queue behind any running Gazebo stack (one sim at a time);
#   2. clean-room gate before every launch (stop_sim + residual check);
#   3. launch the place-only profile headless on ROS_DOMAIN_ID=7;
#   4. run the driver (dry-run matrix or one P0-P4 streak);
#   5. stop_sim, prove zero residual sim/bridge/MoveIt processes.
#
# Usage:
#   scripts/place_only_run.sh --mode dry-run  --out DIR [--ws /tmp/pos1_g1]
#   scripts/place_only_run.sh --mode streak   --out DIR --streak-index N
set -euo pipefail

MODE="streak"
OUT=""
STREAK_INDEX=1
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CASES="P0,P1,P2,P3,P4"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --streak-index) STREAK_INDEX="$2"; shift 2 ;;
    --cases) CASES="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT" ]] || { echo "--out required" >&2; exit 2; }

PRIMARY_WS="${AGENT_COORD_ROOT:-/home/adamliao/work/elfin_humble_ws}"
# ROS setup scripts reference unset vars (AMENT_TRACE_SETUP_FILES); relax
# nounset around the sources.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=7
PIDFILE="${ELFIN_SIM_PIDFILE:-/tmp/elfin_humble_sim.pid}"

residual_count() {
  pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|clock_bridge|camera_bridge|moveit_ros_move_group/move_group|place_only_eval_driver' \
    | wc -l | tr -d ' '
}

wait_for_slot() {
  local waited=0
  while true; do
    local busy=0
    pgrep -af 'ros2 launch luggage_gazebo' >/dev/null 2>&1 && busy=1
    [[ -f "$PIDFILE" ]] && busy=1
    if [[ "$busy" -eq 0 ]]; then
      break
    fi
    if (( waited % 60 == 0 )); then
      echo "waiting for exclusive sim slot (${waited}s elapsed)" >&2
    fi
    sleep 10
    waited=$((waited + 10))
  done
}

clean_room() {
  if [[ -f "$PIDFILE" ]]; then
    bash "$PRIMARY_WS/scripts/stop_sim.sh" || true
  fi
  local residuals
  residuals=$(residual_count || true)
  echo "{\"stage\": \"clean_room\", \"residual_processes\": $residuals}" \
    >> "$OUT/lifecycle.jsonl"
  if [[ "$residuals" -ne 0 ]]; then
    echo "residual sim/bridge processes ($residuals); refusing to launch" >&2
    return 1
  fi
}

teardown() {
  bash "$PRIMARY_WS/scripts/stop_sim.sh" || true
  sleep 2
  local residuals
  residuals=$(residual_count || true)
  echo "{\"stage\": \"teardown\", \"residual_processes\": $residuals,
        \"ts\": \"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
  echo "teardown residual count: $residuals"
  if [[ "$residuals" -ne 0 ]]; then
    pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|moveit_ros_move_group/move_group' || true
    return 1
  fi
}

launch_stack() {
  (ros2 launch luggage_gazebo sim_world.launch.py \
      profile_config:="$WS/install/luggage_gazebo/share/luggage_gazebo/config/place_only_profile.yaml" \
      gui:=false use_rviz:=false \
      > "$OUT/launch_streak${STREAK_INDEX}.log" 2>&1 &)
  sleep 2
  local pid
  pid=$(pgrep -f "ros2 launch luggage_gazebo sim_world.launch.py" | head -1)
  [[ -n "$pid" ]] || { echo "launch failed" >&2; return 1; }
  echo "$pid" > "$PIDFILE"
  echo "{\"stage\": \"launch\", \"pid\": $pid, \"ts\": \"$(date -Is)\"}" \
    >> "$OUT/lifecycle.jsonl"
}

run_driver() {
  local driver_args=(
    --out "$OUT"
    --cases "$CASES"
    --streak-index "$STREAK_INDEX"
  )
  if [[ "$MODE" == "dry-run" ]]; then
    driver_args+=(--dry-run)
  fi
  ros2 run luggage_gazebo place_only_eval_driver.py "${driver_args[@]}"
}

mkdir -p "$OUT"
echo "{\"stage\": \"start\", \"mode\": \"$MODE\", \"streak\": $STREAK_INDEX,
      \"ws\": \"$WS\", \"ts\": \"$(date -Is)\"}" > "$OUT/lifecycle.jsonl"

wait_for_slot
clean_room
trap 'teardown' EXIT
launch_stack
sleep 20   # controller spawn + move_group + auto scene sync
run_driver
status=$?
teardown
trap - EXIT
exit $status
