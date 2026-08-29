# Gate 0 Jazzy environment record

Date: 2026-08-29  
Host: native `/opt/ros/jazzy` (not Docker)  
Branch: `ros2_jazzy`  
Workspace: `elfin_jazzy_ws`

## Platform

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.4 LTS (`noble`) |
| Architecture | `amd64` / `x86_64` |
| Kernel | `6.8.0-138-generic` |
| CPU | host default |
| GPU | NVIDIA GeForce RTX 2080 SUPER, driver 595.84, 8192 MiB |
| Python | 3.12.7 (`/usr/bin/python3`) |
| ROS distro | Jazzy Jalisco (`/opt/ros/jazzy`) |
| Gazebo | Harmonic (`gz sim --versions` → 8.11.0) |
| RMW | `rmw_fastrtps_cpp` (default; `RMW_IMPLEMENTATION` unset) |

`mesa-utils` / `glxinfo` are not installed (apt needs sudo). GPU gate falls back to `nvidia-smi` until `mesa-utils` is present. Do not install Gazebo from `packages.osrfoundation.org` next to `ros-jazzy-ros-gz` vendor Harmonic.

## Installed Gate 0 apt packages

| Debian package | Version |
|---|---|
| `ros-jazzy-desktop` | 0.11.0-1noble.20260616.084553 |
| `ros-jazzy-moveit` | 2.12.4-1noble.20260617.161956 |
| `ros-jazzy-ros2-control` | 4.45.2-1noble.20260615.175135 |
| `ros-jazzy-ros2-controllers` | 4.40.1-1noble.20260616.074625 |
| `ros-jazzy-gz-ros2-control` | 1.2.19-1noble.20260615.171757 |
| `ros-jazzy-ros-gz` | 1.0.22-1noble.20260616.074726 |
| `ros-jazzy-ros-gz-sim` | 1.0.22-1noble.20260615.173223 |
| `ros-jazzy-xacro` | 2.1.1-1noble.20260519.011123 |
| `ros-jazzy-rviz2` | 14.1.22-1noble.20260615.174609 |
| `ros-jazzy-joint-trajectory-controller` | 4.40.1-1noble.20260615.171409 |
| `ros-jazzy-joint-state-broadcaster` | 4.40.1-1noble.20260615.171040 |
| `ros-jazzy-hardware-interface` | 4.45.2-1noble.20260615.155429 |
| `ros-jazzy-control-msgs` | 5.9.0-1noble.20260615.113124 |
| `ros-jazzy-ros2controlcli` | 4.45.2-1noble.20260615.165650 |

Also present: `ros-jazzy-moveit-msgs`, `ros-jazzy-geometric-shapes`, `ros-jazzy-octomap-msgs`, `ros-jazzy-tf2-eigen`, `python3-colcon-common-extensions`, `python3-rosdep`, `python3-vcstool`, `python3-numpy` 1.26.4, `python3-yaml`, `python3-pyqt5`.

`ros-jazzy-octomap-rviz-plugins` and `ros-jazzy-image-view` were not confirmed installed. They are only required by the ignored ROS 1 `luggage_bringup` package.

## Prefix checks

```text
moveit_ros_move_group          /opt/ros/jazzy
controller_manager             /opt/ros/jazzy
joint_trajectory_controller    /opt/ros/jazzy
hardware_interface             /opt/ros/jazzy
```

`mock_components/GenericSystem` is still inside `hardware_interface`:

- plugin: `/opt/ros/jazzy/share/hardware_interface/mock_components_plugin_description.xml`
- library: `/opt/ros/jazzy/lib/libmock_components.so`

## Workspace notes

- `luggage_bringup` has `COLCON_IGNORE` (still catkin/rospy).
- `yolov8s-world.pt` is gitignored; CMake skips installing it when missing.
- Product sim is Gazebo Harmonic, not Fortress.

## Build

`colcon list` sees 11 ament packages (`luggage_bringup` ignored).

This host has a broken `/usr/local/bin/cmake` (OpenSSL 1.1) and Anaconda `base` on PATH. Successful builds use:

```bash
export PATH="/usr/bin:$PATH"
unset CONDA_PREFIX CONDA_DEFAULT_ENV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3
```

Result (2026-08-29): **11 packages finished**. Mock and Harmonic runtime evidence: [jazzy_validation.md](jazzy_validation.md).

