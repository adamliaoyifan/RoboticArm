#!/usr/bin/env bash
# Gate 1 handshake without the real arm: sim executor + 2° FJT round-trip.
# Isolated ROS_DOMAIN_ID. No Gazebo. Do not use this as a substitute for
# jazzy_real.launch.py on hardware.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN="${ROS_DOMAIN_ID_SIM:-87}"
LOG="$(mktemp /tmp/gate1_sim_XXXX.log)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
set +u
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

export ROS_DOMAIN_ID="$DOMAIN"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (sim handshake)"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -- -"$LAUNCH_PID" 2>/dev/null || kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

setsid ros2 launch elfin_trajectory_executor sim.launch.py >"$LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 40); do
  if ros2 action list 2>/dev/null | grep -q '/elfin_arm_controller/follow_joint_trajectory'; then
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "sim launch died:"
    cat "$LOG"
    exit 1
  fi
  sleep 0.25
done

if ! ros2 action list | grep -q '/elfin_arm_controller/follow_joint_trajectory'; then
  echo "FJT action never appeared"
  cat "$LOG"
  exit 1
fi

ros2 run elfin_trajectory_executor send_joint_trajectory --delta-deg 2 --and-back
echo "gate1 sim handshake: PASS (READY_FOR_NEXT). Real arm still needs jazzy_real + ping."
