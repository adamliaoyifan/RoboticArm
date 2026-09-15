# Site ROS 2 environment. Source this before ros2 launch / ros2 run.
# Does not set ROS_DOMAIN_ID — export that yourself in this shell.
#
#   source /home/adamliao/work/RoboticArm/deployment_ws/scripts/site_env.sh
#   export ROS_DOMAIN_ID=7          # or whatever you want
#   ros2 launch ...
#
# Nested tree (ros2_humble): overlays elfin_humble_ws + deployment_ws.
# Flat master tree: overlays repo-root install + deployment_ws.
# shellcheck shell=bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "source this file, do not run it:" >&2
  echo "  source $0" >&2
  exit 1
fi

_site_env_prepend() {
  local var="$1"
  local val="$2"
  local cur="${!var-}"
  if [[ -z "$val" ]]; then
    return 0
  fi
  if [[ -z "$cur" ]]; then
    export "$var=$val"
    return 0
  fi
  case ":$cur:" in
    *":$val:"*) ;;
    *) export "$var=$val:$cur" ;;
  esac
}

_SITE_ENV="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY="$(cd "$_SITE_ENV/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
SDK="${REPO}/third_party/huayan_python_sdk"
if [[ -d "${REPO}/src/luggage_planning" ]]; then
  LUGGAGE_WS="$REPO"
else
  LUGGAGE_WS="${REPO}/elfin_humble_ws"
fi
unset _SITE_ENV

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "site_env: need /opt/ros/jazzy" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ -n "${ROS_DISTRO-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "site_env: ROS_DISTRO=${ROS_DISTRO} (need jazzy). Open a new shell." >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -f "$DEPLOY/install/setup.bash" ]]; then
  echo "site_env: build deployment_ws first:" >&2
  echo "  cd $DEPLOY && colcon build --packages-select elfin_trajectory_executor" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -f "$LUGGAGE_WS/install/setup.bash" ]]; then
  echo "site_env: missing ${LUGGAGE_WS}/install/setup.bash" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -d "$SDK" ]]; then
  echo "site_env: missing Huayan SDK at $SDK" >&2
  return 1 2>/dev/null || exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "$DEPLOY/livox_ws/env.sh" ]]; then
  # shellcheck disable=SC1091
  source "$DEPLOY/livox_ws/env.sh"
fi
# shellcheck disable=SC1091
source "$LUGGAGE_WS/install/setup.bash"
# shellcheck disable=SC1091
source "$DEPLOY/install/setup.bash"
set -u

unset ROS_LOCALHOST_ONLY
export HUAYAN_SDK="$SDK"
export ROBOTARM_REPO="$REPO"
export ROBOTARM_DEPLOY="$DEPLOY"
_site_env_prepend PYTHONPATH "$SDK"
# Apt librealsense 2.58.4 (D555 DDS). Keep ahead of Jazzy's copy.
_site_env_prepend LD_LIBRARY_PATH "/lib/x86_64-linux-gnu"

echo "site_env: jazzy + ${LUGGAGE_WS##*/} + deployment_ws + huayan sdk"
if [[ -n "${ROS_DOMAIN_ID-}" ]]; then
  echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (unchanged)"
else
  echo "  ROS_DOMAIN_ID unset (DDS default 0). export ROS_DOMAIN_ID=<n> yourself."
fi
echo "  ROS_DISTRO=${ROS_DISTRO}"
echo "  HUAYAN_SDK=${HUAYAN_SDK}"
