#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
# Perception host (Orin): Mid-360 + D555 + YOLO-World + DetectLuggage.
# Topic names are unchanged. Laptop planning subscribes the same names
# on ROS_DOMAIN_ID=7.
#
#   ./perception_site.sh
#   ./perception_site.sh semantic_device:=cpu start_mid360:=false
#   ros2 run luggage_perception perception_site.sh
#
# Laptop subscribe (do not remap):
#   /camera/d555/color/image_raw/compressed
#   /camera/d555/color/camera_info
#   /camera/d555/aligned_depth_to_color/image_raw/compressed
#   /camera/d555/aligned_depth_to_color/camera_info
#   /livox/lidar
#   /livox/imu
#   /luggage/preprocessed/camera/color/image
#   /luggage/preprocessed/camera/color/camera_info
#   /luggage/preprocessed/camera/depth/image
#   /luggage/preprocessed/camera/depth/camera_info
#   /luggage/preprocessed/status
#   /luggage/semantic/mask
#   /luggage/semantic/overlay
#   /luggage/semantic/instance_mask
#   /luggage/semantic/yolo_detections
#   /luggage/semantic/cargo_points
#   /luggage/semantic/obstacle_points
# Service: /luggage_detector/detect_luggage
#
# Orin also needs laptop /joint_states, /tf, /tf_static (same domain).
# One D555 owner. Do not run hardware_pick.sh with start_d555/perception
# true on the laptop at the same time. No CPS on this host.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO=""
if [[ -f "$HERE/../package.xml" ]] && grep -q '<name>luggage_perception</name>' "$HERE/../package.xml"; then
  PERC_PKG="$(cd "$HERE/.." && pwd)"
  if [[ -d "$PERC_PKG/../../src/luggage_perception" ]]; then
    REPO="$(cd "$PERC_PKG/../.." && pwd)"
  elif [[ -d "$PERC_PKG/../luggage_perception" ]]; then
    REPO="$(cd "$PERC_PKG/.." && pwd)"
  fi
fi

_pkg_on_path() {
  python3 - <<'PY' >/dev/null 2>&1
from ament_index_python.packages import get_package_share_directory
get_package_share_directory("luggage_perception")
PY
}

_source_ws_install() {
  local ws
  for ws in "$@"; do
    if [[ -n "$ws" && -f "$ws/install/local_setup.bash" ]]; then
      # shellcheck disable=SC1091
      source "$ws/install/local_setup.bash"
    fi
  done
}

_source_livox() {
  local cand
  for cand in \
    "${LIVOX_WS:-}" \
    "${REPO:+$REPO/deployment_ws/livox_ws}" \
    "${HOME}/ros2_ws/livox_ws"
  do
    if [[ -n "$cand" && -f "$cand/install/local_setup.bash" ]]; then
      # shellcheck disable=SC1091
      source "$cand/install/local_setup.bash"
      if [[ -d "$cand/sdk_prefix/lib" ]]; then
        export LD_LIBRARY_PATH="$cand/sdk_prefix/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
      fi
      return 0
    fi
  done
  return 0
}

_has_nvidia_gpu() {
  command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1
}

set +u
if _pkg_on_path; then
  :
elif [[ -n "$REPO" && -f /opt/ros/jazzy/setup.bash && -f "$REPO/deployment_ws/scripts/env_jazzy_real.sh" ]]; then
  # shellcheck disable=SC1091
  source "$REPO/deployment_ws/scripts/env_jazzy_real.sh"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  _source_ws_install "${ROS2_WS:-}" "${HOME}/ros2_ws" "$REPO"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  _source_ws_install "${ROS2_WS:-}" "${HOME}/ros2_ws" "$REPO" "${REPO:+$REPO/deployment_ws}"
else
  echo "need ROS 2 Humble or Jazzy on PATH" >&2
  exit 1
fi
_source_livox
set -u

if ! _pkg_on_path; then
  echo "luggage_perception is not on AMENT_PREFIX_PATH. colcon build it, then source install/local_setup.bash" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_LOCALHOST_ONLY

if [[ -z "${LUGGAGE_CLIP_VENDOR_DIR:-}" ]]; then
  if [[ -n "${PERC_PKG:-}" && -d "$PERC_PKG/vendor" ]]; then
    export LUGGAGE_CLIP_VENDOR_DIR="$PERC_PKG/vendor"
  else
    _share="$(python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
print(get_package_share_directory("luggage_perception"))
PY
)"
    if [[ -d "$_share/vendor" ]]; then
      export LUGGAGE_CLIP_VENDOR_DIR="$_share/vendor"
    fi
  fi
fi

launch_args=()
has_semantic_device=false
has_mid360_config=false
for arg in "$@"; do
  if [[ "$arg" == "--" ]]; then
    continue
  elif [[ "$arg" == --*:=* ]]; then
    arg="${arg#--}"
  fi
  launch_args+=("$arg")
  case "$arg" in
    semantic_device:=*) has_semantic_device=true ;;
    user_config_path:=*) has_mid360_config=true ;;
  esac
done

if [[ "$has_semantic_device" == false ]] && ! _has_nvidia_gpu; then
  echo "no NVIDIA GPU (nvidia-smi failed); defaulting semantic_device:=cpu" >&2
  launch_args+=("semantic_device:=cpu")
fi

if [[ "$has_mid360_config" == false && -n "${MID360_CONFIG:-}" ]]; then
  launch_args+=("user_config_path:=${MID360_CONFIG}")
elif [[ "$has_mid360_config" == false && -n "$REPO" && -f "$REPO/deployment_ws/config/MID360s_config.json" ]]; then
  launch_args+=("user_config_path:=$REPO/deployment_ws/config/MID360s_config.json")
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "perception_site: Mid-360 + D555 + YOLO. No CPS. Same topic names."
echo "Laptop subscribe: /camera/d555/... /livox/lidar /luggage/preprocessed/... /luggage/semantic/yolo_detections /luggage_detector/detect_luggage"

exec ros2 launch luggage_perception perception_site.launch.py "${launch_args[@]}"
