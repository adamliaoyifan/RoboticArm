# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
# Source this (do not execute) in every Jazzy real-arm terminal:
#   source /path/to/deployment_ws/scripts/env_jazzy_real.sh
#
# Covers the executor and luggage_planning hardware_pick_driver.
# Overlay: jazzy -> repo install -> livox_ws -> deployment_ws.
# ROS_DOMAIN_ID=7. Waypoint only. No ServoEsJ. No Gazebo.
#
# Colcon setups read empty COLCON_* vars; this file clears nounset while
# sourcing, then restores the caller's nounset flag.

_ENV_HAD_NOUNSET=0
case $- in
  *u*) _ENV_HAD_NOUNSET=1 ;;
esac
set +u

_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
_DEPLOY="${_REPO}/deployment_ws"
_SDK="${_REPO}/third_party/huayan_python_sdk"
if [[ -d "${_REPO}/src/luggage_planning" ]]; then
  _HUMBLE_WS="${_REPO}"
else
  _HUMBLE_WS="${_REPO}/elfin_humble_ws"
fi

unset PYTHONPATH COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# local_setup only — setup.bash can chain another ws and shadow this tree.
if [[ -f "${_HUMBLE_WS}/install/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_HUMBLE_WS}/install/local_setup.bash"
else
  echo "env_jazzy_real: missing ${_HUMBLE_WS}/install/local_setup.bash (luggage_planning)" >&2
fi
if [[ -f "${_DEPLOY}/livox_ws/install/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_DEPLOY}/livox_ws/install/local_setup.bash"
fi
if [[ -f "${_DEPLOY}/install/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_DEPLOY}/install/local_setup.bash"
else
  echo "env_jazzy_real: missing ${_DEPLOY}/install/local_setup.bash (executor)" >&2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_LOCALHOST_ONLY
export PYTHONPATH="${_SDK}${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
if [[ -d "${_DEPLOY}/livox_ws/sdk_prefix/lib" ]]; then
  export LD_LIBRARY_PATH="${_DEPLOY}/livox_ws/sdk_prefix/lib:${LD_LIBRARY_PATH}"
fi
export ROBOT_IP="${ROBOT_IP:-192.168.0.10}"
export ROBOT_PORT="${ROBOT_PORT:-10003}"

unset _REPO _DEPLOY _SDK _HUMBLE_WS
if [[ "${_ENV_HAD_NOUNSET}" == 1 ]]; then
  unset _ENV_HAD_NOUNSET
  set -u
else
  unset _ENV_HAD_NOUNSET
fi
