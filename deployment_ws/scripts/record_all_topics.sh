#!/usr/bin/env bash
# Record every topic on the current graph. Does not start drivers.
# Ctrl+C stops the bag only.
#
#   export ROS_DOMAIN_ID=11
#   ./scripts/record_all_topics.sh
#   ./scripts/record_all_topics.sh -o ~/robotarm_bags -n pp_ab_B
set -euo pipefail

OUT_DIR="${HOME}/robotarm_bags"
BAG_NAME=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--out) OUT_DIR="$2"; shift 2 ;;
    -n|--name) BAG_NAME="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: record_all_topics.sh [-o DIR] [-n NAME]"
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      exit 2
      ;;
  esac
done

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
QOS="${DEPLOY}/src/elfin_trajectory_executor/config/bag_qos_overrides.yaml"
if [[ ! -f "$QOS" ]]; then
  QOS="${DEPLOY}/install/elfin_trajectory_executor/share/elfin_trajectory_executor/config/bag_qos_overrides.yaml"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "${REPO}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${REPO}/install/setup.bash"
fi
if [[ -f "${DEPLOY}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${DEPLOY}/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}"
unset ROS_LOCALHOST_ONLY

OUT_DIR="$(mkdir -p "$OUT_DIR" && cd "$OUT_DIR" && pwd)"
if [[ -z "$BAG_NAME" ]]; then
  BAG_NAME="all_topics_$(date +%Y%m%d_%H%M%S)"
fi
BAG_PATH="${OUT_DIR}/${BAG_NAME}"
if [[ -e "$BAG_PATH" ]]; then
  echo "bag already exists: $BAG_PATH" >&2
  exit 1
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "bag -> ${BAG_PATH}"
echo "Ctrl+C to stop"

cmd=(
  ros2 bag record -a
  -o "$BAG_PATH"
  --include-hidden-topics
  --max-cache-size "$((64 * 1024 * 1024))"
)
if [[ -f "$QOS" ]]; then
  cmd+=(--qos-profile-overrides-path "$QOS")
fi

exec "${cmd[@]}"
