#!/usr/bin/env bash
# Clone Livox-SDK2 + livox_ros_driver2 and build against Jazzy.
# Overlay workspace is gitignored (deployment_ws/livox_ws).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${LIVOX_WS:-$ROOT/livox_ws}"
BRANCH="${LIVOX_BRANCH:-master}"
SDK_PREFIX="${LIVOX_SDK_PREFIX:-$WS/sdk_prefix}"
JOBS="${LIVOX_JOBS:-$(nproc)}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [[ "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "ERROR: ROS_DISTRO=${ROS_DISTRO} (need jazzy)" >&2
  exit 1
fi

mkdir -p "$WS/src" "$SDK_PREFIX"

if [[ ! -d "$WS/Livox-SDK2/.git" ]]; then
  git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$WS/Livox-SDK2"
fi

if [[ ! -f "$SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so" ]]; then
  echo "Building Livox-SDK2 -> $SDK_PREFIX"
  cmake -S "$WS/Livox-SDK2" -B "$WS/Livox-SDK2/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$SDK_PREFIX"
  cmake --build "$WS/Livox-SDK2/build" -j"$JOBS"
  cmake --install "$WS/Livox-SDK2/build"
fi

DRIVER="$WS/src/livox_ros_driver2"
if [[ ! -d "$DRIVER/.git" ]]; then
  git clone --depth 1 -b "$BRANCH" https://github.com/Livox-SDK/livox_ros_driver2.git \
    "$DRIVER"
fi

# Official build.sh copies these, then runs colcon from $WS. We skip
# build.sh so we can point cmake at the prefix-installed Livox-SDK2
# without writing /usr/local.
cp -f "$DRIVER/package_ROS2.xml" "$DRIVER/package.xml"
rm -rf "$DRIVER/launch"
cp -rf "$DRIVER/launch_ROS2" "$DRIVER/launch"

export LD_LIBRARY_PATH="$SDK_PREFIX/lib:${LD_LIBRARY_PATH:-}"
cd "$WS"
rm -rf build install log
colcon build --packages-select livox_ros_driver2 --cmake-args \
  -DROS_EDITION=ROS2 \
  -DDISTRO_ROS=jazzy \
  -DLIVOX_LIDAR_SDK_LIBRARY="$SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$SDK_PREFIX/include"

cat > "$WS/env.sh" <<EOF
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export LD_LIBRARY_PATH="$SDK_PREFIX/lib:\${LD_LIBRARY_PATH:-}"
EOF
echo "OK: source $WS/env.sh"
echo "Then: ros2 launch luggage_description mid360.launch.py"
