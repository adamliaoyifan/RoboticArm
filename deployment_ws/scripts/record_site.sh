#!/usr/bin/env bash
# One-shot site recording. Foreground: Ctrl+C stops drivers and the bag.
#
#   ./scripts/record_site.sh
#   ./scripts/record_site.sh pendant
#   ./scripts/record_site.sh real -o ~/robotarm_bags/tracking
#   ./scripts/record_site.sh pendant -o ~/robotarm_bags/2026-09-09 -n jog_box1
#   ./scripts/record_site.sh real --out /data/bags/fail_blend
#
# -o/--out  is a directory (created if missing). The bag folder is created
#           inside it and must not already exist.
# -n/--name bag folder name; default record_site_<mode>_<YYYYMMDD_HHMMSS>
# Extra ros2 launch args after -- :
#   ./scripts/record_site.sh pendant -- use_rviz:=true
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: record_site.sh [pendant|real] [-o DIR] [-n NAME] [-- LAUNCH_ARGS...]

  pendant   teach pendant, CPS monitor only (default)
  real      jazzy_real FollowJointTrajectory (e-stop person required)

  -o, --out DIR    parent directory for the bag (default: ~/robotarm_bags)
  -n, --name NAME  bag folder name under DIR
  -h, --help

Ctrl+C stops recording. Do not start scene.launch.py or a second CPS client.
EOF
}

MODE="pendant"
OUT_DIR="${HOME}/robotarm_bags"
BAG_NAME=""
LAUNCH_EXTRA=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    pendant|real)
      MODE="$1"
      shift
      ;;
    -o|--out)
      OUT_DIR="$2"
      shift 2
      ;;
    -n|--name)
      BAG_NAME="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_EXTRA=("$@")
      break
      ;;
    -*)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      # Bare path: treat as -o for convenience
      OUT_DIR="$1"
      shift
      ;;
  esac
done

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
HUMBLE_WS="${REPO}/elfin_humble_ws"
SDK="${REPO}/third_party/huayan_python_sdk"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "need /opt/ros/jazzy" >&2
  exit 1
fi
if [[ ! -f "$DEPLOY/install/setup.bash" ]]; then
  echo "build deployment_ws first: colcon build --packages-select elfin_trajectory_executor" >&2
  exit 1
fi
if [[ ! -f "$HUMBLE_WS/install/setup.bash" ]]; then
  echo "missing ${HUMBLE_WS}/install/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$DEPLOY/livox_ws/env.sh"
# shellcheck disable=SC1091
source "$HUMBLE_WS/install/setup.bash"
# shellcheck disable=SC1091
source "$DEPLOY/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
# Keep ROS DDS on loopback so Image/PointCloud2 do not flood enp0s31f6
# (that NIC also carries CPS TCP). D555 realdds + Livox SDK still use the NIC.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export PYTHONPATH="${SDK}${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

pkill -f livox_ros_driver2_node >/dev/null 2>&1 || true
sleep 0.3

OUT_DIR="$(mkdir -p "$OUT_DIR" && cd "$OUT_DIR" && pwd)"
if [[ -z "$BAG_NAME" ]]; then
  BAG_NAME="record_site_${MODE}_$(date +%Y%m%d_%H%M%S)"
fi
BAG_PATH="${OUT_DIR}/${BAG_NAME}"
if [[ -e "$BAG_PATH" ]]; then
  echo "bag already exists: $BAG_PATH" >&2
  echo "pick another -n NAME or omit -n for a timestamp" >&2
  exit 1
fi

echo "record_mode=${MODE}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "bag -> ${BAG_PATH}"
echo "Ctrl+C to stop"
if [[ "$MODE" == "real" ]]; then
  echo "REAL mode: executor will electrify/enable. Person on e-stop."
fi

exec ros2 launch elfin_trajectory_executor record_site.launch.py \
  "record_mode:=${MODE}" \
  "bag_path:=${BAG_PATH}" \
  use_rviz:=false \
  "${LAUNCH_EXTRA[@]}"
