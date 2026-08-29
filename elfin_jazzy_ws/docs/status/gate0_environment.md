# Gate 0 environment record

Date: 2026-08-14  
Host: native `/opt/ros/humble` (not Docker)  
Result: **passed**, with one Humble packaging difference noted below.

## Platform

| Item | Value |
|---|---|
| OS | Ubuntu 22.04.5 LTS (`jammy`) |
| Architecture | `amd64` / `x86_64` |
| Kernel | `6.8.0-111-generic` |
| CPU | AMD Ryzen 9 9950X3D 16-Core Processor (32 hardware threads) |
| ROS distro | Humble Hawksbill (`/opt/ros/humble`) |
| RMW | `rmw_fastrtps_cpp` (default; `RMW_IMPLEMENTATION` unset) |
| Build type for later measurement | `Release` (`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`) |

## Installed Gate 0 apt packages

| Debian package | Version |
|---|---|
| `ros-humble-desktop` | 0.10.0-1jammy.20260804.223343 |
| `ros-humble-moveit` | 2.5.9-1jammy.20260804.232923 |
| `ros-humble-ros2-control` | 2.54.0-1jammy.20260804.213423 |
| `ros-humble-ros2-controllers` | 2.53.3-1jammy.20260804.212633 |
| `ros-humble-controller-manager` | 2.54.0-1jammy.20260804.205225 |
| `ros-humble-xacro` | 2.1.1-1jammy.20260304.195513 |
| `ros-humble-rviz2` | 11.2.28-1jammy.20260804.222726 |
| `ros-humble-joint-trajectory-controller` | 2.53.3-1jammy.20260804.210837 |
| `ros-humble-joint-state-broadcaster` | 2.53.3-1jammy.20260804.211034 |
| `ros-humble-hardware-interface` | 2.54.0-1jammy.20260724.031959 |
| `ros-humble-control-msgs` | 4.9.0-1jammy.20260718.015803 |
| `ros-humble-ros2controlcli` | 2.54.0-1jammy.20260804.212938 |

Also installed because `elfin_humble_ws` `package.xml` files declare them and they exist on Humble: `ros-humble-moveit-msgs`, `ros-humble-geometric-shapes`, `ros-humble-octomap-msgs`, `ros-humble-octomap-rviz-plugins`, `ros-humble-image-view`, `ros-humble-tf2-eigen`. `python3-colcon-common-extensions`, `python3-rosdep`, `python3-vcstool`, `python3-numpy`, `python3-yaml`, and `python3-pyqt5` were already present.

## Gate 0 prefix checks

```text
moveit_ros_move_group          /opt/ros/humble
controller_manager             /opt/ros/humble
joint_trajectory_controller    /opt/ros/humble
mock_components                not a standalone ROS package on Humble 2.54
hardware_interface             /opt/ros/humble
```

`mock_components/GenericSystem` is shipped inside `hardware_interface`:

- plugin: `/opt/ros/humble/share/hardware_interface/mock_components_plugin_description.xml`
- library: `/opt/ros/humble/lib/libmock_components.so`

URDF can still use `plugin="mock_components/GenericSystem"`. There is no `ros-humble-mock-components` Debian package in this apt index.

## Workspace rosdep

`elfin_humble_ws/src` currently contains seven Catkin ROS 1 packages (`luggage_*`) plus `pointcloud/` (not a ROS package). There are no ROS 2 packages yet.

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
  --skip-keys "catkin rospy roscpp actionlib tf rosbag moveit_commander message_generation message_runtime rviz gazebo_ros gazebo_msgs elfin_gazebo"
```

Result: all remaining resolvable Humble keys were already installed.

Skipped on purpose:

- ROS 1 keys (`catkin`, `rospy`, `roscpp`, `actionlib`, `tf`, `rosbag`, `moveit_commander`, `message_generation`, `message_runtime`, `rviz`) cannot be satisfied on Humble.
- `gazebo_ros` / `gazebo_msgs` resolve to Gazebo Classic (`libgazebo11`). MVP does not use Gazebo; later simulation uses `gz_ros2_control` / Fortress. Installing Classic now would mix two Gazebo stacks.

`elfin_s_robot` is not in this workspace. Robot meshes/Xacro live in `robot_assets/elfin_description/`. The workspace `Dockerfile` is still `osrf/ros:noetic-desktop-full` and was not used for this Gate 0 host install.

## Next

Gate 1: isolate ROS 1 sources (`COLCON_IGNORE`, remove Noetic `src/CMakeLists.txt` symlink) so `colcon list` only sees new ROS 2 packages.
