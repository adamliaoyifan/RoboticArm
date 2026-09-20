#!/usr/bin/env bash
# OCC-1 cargo-map integrate eval. Exclusive Gazebo on ROS_DOMAIN_ID=7.
#
# Usage:
#   scripts/cargo_map_integrate_run.sh --out DIR [--n 3]
set -euo pipefail

OUT=""
N=3
SEQUENCE_IDS="carryon,standard,large"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --n) N="$2"; shift 2 ;;
    --sequence-ids) SEQUENCE_IDS="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT" ]] || { echo "--out required" >&2; exit 2; }

PRIMARY_WS="${AGENT_COORD_ROOT:-/home/adamliao/work/elfin_humble_ws}"
# Isolate from satellite worktree overlays (dsim1) so ros2 run sees this WS.
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PYTHONPATH="$WS/src/luggage_perception:$WS/src/luggage_planning:$WS/src/luggage_packing:$WS/src/luggage_gazebo:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=7
export DISPLAY="${DISPLAY:-:1}"
PIDFILE="${ELFIN_SIM_PIDFILE:-/tmp/elfin_humble_sim.pid}"

stack_pids() {
  pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|clock_bridge|camera_bridge|moveit_ros_move_group/move_group' \
    | grep -v -e 'bash -c' -e 'cargo_map_integrate_run.sh' -e 'builtin eval' || true
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
echo "{\"stage\": \"start\", \"n\": $N, \"sequence_ids\": \"$SEQUENCE_IDS\",
      \"ws\": \"$WS\", \"ts\": \"$(date -Is)\"}" > "$OUT/lifecycle.jsonl"

wait_for_slot
clean_room
trap 'teardown' EXIT
launch_stack

setsid ros2 run luggage_gazebo cargo_map_integrate_eval_driver.py \
  --n "$N" \
  --payload vacuum \
  --goto-timeout 60 \
  --out "$OUT"
