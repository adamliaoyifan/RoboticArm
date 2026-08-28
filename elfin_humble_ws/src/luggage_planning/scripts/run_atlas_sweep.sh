#!/usr/bin/env bash
# run_atlas_sweep.sh -- one-shot runner for build_payload_atlas_sweep.sh
# Starts Gazebo + MoveIt + scene_manager, waits for /compute_ik, runs the
# 3-box-size payload sweep, then tears everything down (gazebo/moveit/roscore).
#
# Usage:
#   ./run_atlas_sweep.sh [output_dir] [scene_tf_config]
set -uo pipefail

PKG_PLANNING="$(rospack find luggage_planning)"
SWEEP="${PKG_PLANNING}/scripts/build_payload_atlas_sweep.sh"
OUTPUT_DIR="${1:-${PKG_PLANNING}/data/reachability_atlas}"
SCENE_TF="${2:-$(rospack find luggage_description)/config/scene_tf.yaml.example}"

echo "==> output_dir: ${OUTPUT_DIR}"
echo "==> scene_tf:   ${SCENE_TF}"

# 1. Start the stack in background. roslaunch manages gazebo/move_group/scene_manager.
echo "==> Launching atlas_stack (Gazebo + MoveIt + scene_manager)..."
roslaunch luggage_bringup atlas_stack.launch \
  scene_tf_config:="${SCENE_TF}" &
STACK_PID=$!

cleanup() {
  echo ""
  echo "==> Tearing down..."
  # Kill roslaunch (sends SIGINT to its managed nodes)
  kill -INT "${STACK_PID}" 2>/dev/null
  wait "${STACK_PID}" 2>/dev/null
  # Force-kill any stragglers
  pkill -9 -f gazebo 2>/dev/null
  pkill -9 -f move_group 2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null
  pkill -9 -f scene_manager 2>/dev/null
  pkill -9 -f rosmaster 2>/dev/null
  pkill -9 -f roscore 2>/dev/null
  echo "==> Done. Stack stopped."
}
trap cleanup EXIT

# 2. Wait for /compute_ik (MoveIt) to come up.
echo "==> Waiting for /compute_ik service (up to 180s)..."
for i in $(seq 1 180); do
  if rosservice list 2>/dev/null | grep -q '^/compute_ik$'; then
    echo "==> /compute_ik available after ${i}s"
    break
  fi
  sleep 1
  if [ $i -eq 180 ]; then
    echo "!! /compute_ik did not come up in 180s. Aborting." >&2
    exit 1
  fi
done

# Extra settle for planning scene + controllers.
echo "==> Settling 8s for planning scene / controllers..."
sleep 8

# 3. Run the sweep (builds empty-load + 3 payload atlases).
echo "==> Running build_payload_atlas_sweep.sh..."
bash "${SWEEP}" "${OUTPUT_DIR}" "${SCENE_TF}"
SWEEP_RC=$?

echo "==> Sweep exit code: ${SWEEP_RC}"

# 4. Print summary.
echo ""
echo "==> Generated atlas files:"
ls -lh "${OUTPUT_DIR}"/*_container_collision_aware*.{npz,yaml} 2>/dev/null \
  | awk '{print "   ", $5, $9}' || echo "   (none found)"
echo ""
echo "==> reachability_rate by atlas:"
for f in "${OUTPUT_DIR}"/*_container_collision_aware*.yaml; do
  [ -e "$f" ] || continue
  rate=$(grep -m1 "reachability_rate:" "$f" | awk '{print $2}')
  printf "   %-60s rate=%s\n" "$(basename "$f")" "$rate"
done

exit ${SWEEP_RC}
