#!/usr/bin/env bash
# Pack-to-full capacity run (docs/plans/pack_eval_test_guide.md B4).
#
# Owns the simulator lifecycle around pack_eval_driver.py:
#   1. queue behind any running Gazebo stack (one sim at a time);
#   2. clean-room gate before launch (stop_sim + residual check);
#   3. launch the full pick+place+packing stack headless on ROS_DOMAIN_ID=7;
#   4. run the driver until BIN_FULL / MAX_BOXES / SPAWN_EXHAUSTED / ABORT;
#   5. stop_sim, prove zero residual sim/bridge/MoveIt processes.
#
# Usage:
#   scripts/pack_to_full_run.sh --out DIR [--max-boxes N] [--sequence-ids IDS]
set -euo pipefail

OUT=""
MAX_BOXES=50
SEQUENCE_IDS="carryon"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --max-boxes) MAX_BOXES="$2"; shift 2 ;;
    --sequence-ids) SEQUENCE_IDS="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT" ]] || { echo "--out required" >&2; exit 2; }

PRIMARY_WS="${AGENT_COORD_ROOT:-/home/adamliao/work/elfin_humble_ws}"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=7
export DISPLAY="${DISPLAY:-:1}"
PIDFILE="${ELFIN_SIM_PIDFILE:-/tmp/elfin_humble_sim.pid}"

# Match real stack processes only. Monitoring shells and this script carry the
# pattern in their own command line, so exclude shell wrappers explicitly;
# a bare pgrep here deadlocks the slot wait against the operator's own terminal.
stack_pids() {
  pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|clock_bridge|camera_bridge|moveit_ros_move_group/move_group' \
    | grep -v -e 'bash -c' -e 'pack_to_full_run.sh' -e 'builtin eval' || true
}

residual_count() {
  stack_pids | wc -l | tr -d ' '
}

wait_for_slot() {
  local waited=0
  while [[ -n "$(stack_pids)" || -f "$PIDFILE" ]]; do
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
  residuals=$(residual_count)
  echo "{\"stage\": \"clean_room\", \"residual_processes\": $residuals, \"ts\": \"$(date -Is)\"}" \
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
  residuals=$(residual_count)
  echo "{\"stage\": \"teardown\", \"residual_processes\": $residuals, \"ts\": \"$(date -Is)\"}" \
    >> "$OUT/lifecycle.jsonl"
  echo "teardown residual count: $residuals"
  rm -f "$PIDFILE"
}

launch_stack() {
  (ros2 launch luggage_gazebo sim_world.launch.py \
      gui:=false use_rviz:=false \
      use_semantic:=true use_motion:=true use_vacuum:=true \
      use_cargo_map:=true use_packing:=true \
      visual_kind:=mesh size_mode:=catalog \
      sequence_ids:="$SEQUENCE_IDS" \
      observe_pose_name:=pickup_observe \
      semantic_require_backend:=bbox_fill \
      > "$OUT/launch.log" 2>&1 &)
  sleep 3
  local pid
  pid=$(pgrep -f "ros2 launch luggage_gazebo sim_world.launch.py" | head -1)
  [[ -n "$pid" ]] || { echo "launch failed" >&2; return 1; }
  echo "$pid" > "$PIDFILE"
  echo "{\"stage\": \"launch\", \"pid\": $pid, \"ts\": \"$(date -Is)\"}" \
    >> "$OUT/lifecycle.jsonl"
}

mkdir -p "$OUT"
echo "{\"stage\": \"start\", \"max_boxes\": $MAX_BOXES, \"sequence_ids\": \"$SEQUENCE_IDS\",
      \"ws\": \"$WS\", \"ts\": \"$(date -Is)\"}" > "$OUT/lifecycle.jsonl"

wait_for_slot
clean_room
trap 'teardown' EXIT
launch_stack

setsid ros2 run luggage_gazebo pack_eval_driver.py \
  --sequence-ids "$SEQUENCE_IDS" \
  --max-boxes "$MAX_BOXES" \
  --goto-timeout 60 \
  --out "$OUT"
