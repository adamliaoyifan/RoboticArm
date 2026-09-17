#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
#
# Waypoint-only Elfin tests. ServoEsJ is disabled. Person on e-stop for
# launch / h1 / h2. One CPS owner. No Gazebo.
#
#   ./scripts/test_elfin_waypoint.sh help
#   ./scripts/test_elfin_waypoint.sh offline
#   ./scripts/test_elfin_waypoint.sh check
#   ./scripts/test_elfin_waypoint.sh launch          # terminal 1, keep running
#   ./scripts/test_elfin_waypoint.sh launch-h1       # terminal 1, accel 0.5
#   ./scripts/test_elfin_waypoint.sh h1              # terminal 2, zero motion
#   ./scripts/test_elfin_waypoint.sh h2              # terminal 2, 3x 2° and-back
#   ./scripts/test_elfin_waypoint.sh h3-detect       # detect only, no PlanMotion
#
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "${DEPLOY}/.." && pwd)"
ENV_SH="${DEPLOY}/scripts/env_jazzy_real.sh"
CMD="${1:-help}"

_load_env() {
  # shellcheck disable=SC1090
  source "${ENV_SH}"
}

_need_build() {
  if [[ ! -f "${DEPLOY}/install/local_setup.bash" ]]; then
    echo "build first:" >&2
    echo "  cd ${DEPLOY} && colcon build --packages-select elfin_trajectory_executor --symlink-install" >&2
    exit 1
  fi
}

offline() {
  cd "${REPO}"
  PYTHONPATH="${REPO}/deployment_ws/src/elfin_trajectory_executor" \
    python3 -m pytest -q \
    deployment_ws/src/elfin_trajectory_executor/test/test_waypoint_profile.py \
    deployment_ws/src/elfin_trajectory_executor/test/test_fake_cps_execute.py \
    deployment_ws/src/elfin_trajectory_executor/test/test_trajectory_score.py \
    deployment_ws/src/elfin_trajectory_executor/test/test_cps_parse.py
}

check() {
  _load_env
  python3 "${DEPLOY}/scripts/check_site.py"
  python3 "${DEPLOY}/scripts/check_gate1.py"
  echo "cps sockets:"
  ss -tn | awk '/192.168.0.10:10003/{print}' || echo "cps_free"
}

launch() {
  _need_build
  _load_env
  echo "[test_elfin_waypoint] waypoint 20/60. Ctrl+C disables servo, does not BlackOut."
  exec ros2 launch elfin_trajectory_executor jazzy_real.launch.py \
    max_velocity_deg:=20.0 \
    command_acceleration_deg:=60.0 \
    execution_backend:=waypoint
}

launch_h1() {
  _need_build
  _load_env
  echo "[test_elfin_waypoint] H1 launch accel=0.5. Expect INVALID_GOAL, no motion."
  exec ros2 launch elfin_trajectory_executor jazzy_real.launch.py \
    max_velocity_deg:=20.0 \
    command_acceleration_deg:=0.5 \
    execution_backend:=waypoint
}

h1() {
  _load_env
  ros2 run elfin_trajectory_executor send_joint_trajectory \
    --delta-deg 2 --duration 0.4
}

h2() {
  _load_env
  local i
  for i in 1 2 3; do
    echo "[test_elfin_waypoint] H2 cycle ${i}/3"
    ros2 run elfin_trajectory_executor send_joint_trajectory \
      --delta-deg 2 --and-back
  done
  echo "[test_elfin_waypoint] H2 3/3 done. Expect READY_FOR_NEXT each cycle."
}

h3_detect() {
  _load_env
  echo "[test_elfin_waypoint] detect only. Do not execute if top_z is far below the cup."
  echo "Need hardware_pick graph already up (D555 + detect + executor)."
  ros2 run luggage_planning hardware_pick_driver.py \
    --skip-observe --detect-only --no-vacuum
}

help() {
  cat <<'EOF'
Waypoint-only site tests (ServoEsJ off). Person on e-stop for powered steps.

Terminal 1 — executor (keep running):
  cd /home/adamliao/work/RoboticArm-master/deployment_ws
  ./scripts/test_elfin_waypoint.sh offline
  ./scripts/test_elfin_waypoint.sh check
  ./scripts/test_elfin_waypoint.sh launch          # 20 deg/s, 60 deg/s^2

Terminal 2 — same env, after log shows [executor] Ready.:
  source scripts/env_jazzy_real.sh
  ./scripts/test_elfin_waypoint.sh h2              # 3x 2° J1 and-back

H1 preflight (zero motion). Restart terminal 1 with:
  ./scripts/test_elfin_waypoint.sh launch-h1
  ./scripts/test_elfin_waypoint.sh h1              # expect INVALID_GOAL
Then Ctrl+C launch-h1 and go back to ./scripts/test_elfin_waypoint.sh launch

H3 dry pick is not this script. Empty-space dry used --virtual-at-tcp.

Floor suitcase (YOLO lid ~0.24 m world). Graph already up, person on e-stop:
  ros2 run luggage_planning hardware_pick_driver.py --skip-observe --plan-only
  ros2 run luggage_planning hardware_pick_driver.py --skip-observe
  # prints TCP / top_z / waypoints; type yes. Abort with anything else.

H4 ServoEsJ: dropped on this cell (Push 20006). Do not run.
H5 fixture plate: skipped (vacuum already proven).

Abort on e-stop, protective stop, unexpected direction, or duplicate CPS owner.
EOF
}

case "${CMD}" in
  help|-h|--help) help ;;
  offline) offline ;;
  check) check ;;
  launch) launch ;;
  launch-h1) launch_h1 ;;
  h1) h1 ;;
  h2) h2 ;;
  h3-detect) h3_detect ;;
  *)
    echo "unknown command: ${CMD}" >&2
    help
    exit 1
    ;;
esac
