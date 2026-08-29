#!/usr/bin/env bash
# forward_shift_sweep.sh -- one-shot runner for forward_shift_sweep.py.
# Starts atlas_stack (Gazebo + MoveIt + scene_manager), waits for services,
# runs the forward-shift x tolerance x clearance sweep, then tears down.
#
# Usage:
#   ./forward_shift_sweep.sh [scene_tf_config]
set -uo pipefail

PKG_PLANNING="$(rospack find luggage_planning)"
SCENE_TF="${1:-$(rospack find luggage_description)/config/scene_tf.yaml.example}"
OUTPUT_YAML="${PKG_PLANNING}/data/reachability_atlas/forward_sweep_results.yaml"

echo "==> scene_tf: ${SCENE_TF}"
echo "==> output:   ${OUTPUT_YAML}"

echo "==> Launching atlas_stack (Gazebo + MoveIt + scene_manager)..."
roslaunch luggage_bringup atlas_stack.launch scene_tf_config:="${SCENE_TF}" &
STACK_PID=$!

cleanup() {
  echo ""
  echo "==> Tearing down..."
  kill -INT "${STACK_PID}" 2>/dev/null
  wait "${STACK_PID}" 2>/dev/null
  pkill -9 -f gazebo 2>/dev/null
  pkill -9 -f move_group 2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null
  pkill -9 -f scene_manager 2>/dev/null
  pkill -9 -f rosmaster 2>/dev/null
  pkill -9 -f roscore 2>/dev/null
  echo "==> Done. Stack stopped."
}
trap cleanup EXIT

echo "==> Waiting for /compute_ik + /scene_manager/sync_static_scene (up to 180s)..."
for i in $(seq 1 180); do
  svc=$(rosservice list 2>/dev/null)
  if echo "$svc" | grep -q '^/compute_ik$' && echo "$svc" | grep -q '/scene_manager/sync_static_scene'; then
    echo "==> services available after ${i}s"
    break
  fi
  sleep 1
  if [ $i -eq 180 ]; then
    echo "!! services did not come up in 180s. Aborting." >&2
    exit 1
  fi
done

echo "==> Settling 8s for planning scene / controllers..."
sleep 8

echo "==> Running forward_shift_sweep.py..."
rosrun luggage_planning forward_shift_sweep.py \
  _scene_tf_config:="${SCENE_TF}" \
  _output_yaml:="${OUTPUT_YAML}"
RC=$?

echo "==> Sweep exit code: ${RC}"
exit ${RC}
