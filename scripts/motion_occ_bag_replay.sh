#!/usr/bin/env bash
# Replay a MOTION-OCC rosbag in RViz on an isolated domain.
#
# NEVER play onto the live sim (ROS_DOMAIN_ID=7) or a real arm.
# This script does not start Gazebo.
#
# Usage:
#   scripts/motion_occ_bag_replay.sh --bag DIR/rosbag/motion_occ
#   scripts/motion_occ_bag_replay.sh --bag DIR/rosbag/motion_occ --no-rviz
set -euo pipefail

BAG=""
USE_RVIZ=1
DOMAIN="${ROS_DOMAIN_ID:-42}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --no-rviz) USE_RVIZ=0; shift ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$BAG" ]] || { echo "--bag required" >&2; exit 2; }
[[ -e "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 2; }

if [[ "$DOMAIN" == "7" ]]; then
  echo "refusing ROS_DOMAIN_ID=7 (live sim / arm). Use --domain 42." >&2
  exit 2
fi

unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN"
export DISPLAY="${DISPLAY:-:1}"

RVIZ_CFG="$WS/src/luggage_gazebo/rviz/sim_full.rviz"
if [[ "$USE_RVIZ" == "1" ]]; then
  setsid rviz2 -d "$RVIZ_CFG" >/tmp/motion_occ_bag_rviz.log 2>&1 &
  echo "rviz2 pid $!  log /tmp/motion_occ_bag_rviz.log  domain $DOMAIN"
  echo "Add displays: TF, RobotModel (needs robot_description), PlanningScene on /monitored_planning_scene"
  sleep 2
fi

echo "playing $BAG on ROS_DOMAIN_ID=$DOMAIN (clock published)"
ros2 bag play "$BAG" --clock --rate 1.0
