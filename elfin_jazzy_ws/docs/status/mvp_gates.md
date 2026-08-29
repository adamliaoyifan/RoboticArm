# Phase 1 MVP gates (1–5)

Date: 2026-08-14  
Workspace: `elfin_humble_ws`  
Build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`  
RMW: `rmw_fastrtps_cpp`  
Result: **passed**

Humble xacro on this host does not implement `$(find-pkg-share ...)`. `$(find elfin_description)` already resolves through `ament_index_python.get_package_share_directory`.

Joint names: `elfin_joint1`–`elfin_joint6`. Planning group: `elfin_arm`.  
Mock initial / home: `[0, -0.5, 0, 0, 0, 0]` (joint 2 URDF upper limit is `0.1745`).  
Demo goal: `[0.3, -1.0, 0.5, -0.8, 0.4, 0.3]`.

## Gate 1 isolate ROS 1

- Removed `src/CMakeLists.txt` Noetic `toplevel.cmake` symlink.
- `COLCON_IGNORE` on all seven `luggage_*` packages.
- `src/pointcloud/` left in place (no `package.xml`).
- ROS 1 sources were not deleted.

```text
colcon list
elfin_control       src/elfin_control       (ros.ament_cmake)
elfin_description   src/elfin_description   (ros.ament_cmake)
elfin_moveit_config src/elfin_moveit_config (ros.ament_cmake)
elfin_mvp_bringup   src/elfin_mvp_bringup   (ros.ament_cmake)
```

## Gate 2 `elfin_description`

- S20 STL, `materials.xacro`, `S20.urdf.xacro` copied from `robot_assets/elfin_description`.
- Classic `elfin_robot.gazebo` is not included on the mock path.
- `hardware_plugin` defaults to `mock_components/GenericSystem`.
- Six joints: position command, position/velocity state.

```text
xacro .../urdf/S20.urdf.xacro hardware_plugin:=mock_components/GenericSystem
check_urdf /tmp/elfin_s20.urdf
# robot name is: S20 — Successfully Parsed XML — root Link: world
```

RViz2 config: `src/elfin_description/rviz/view_robot.rviz` (RobotModel + TF only). Interactive RViz was not required for this record; GPU renderer is NVIDIA (see [gpu_runtime.md](gpu_runtime.md)).

## Gate 3 `elfin_control`

Launch: `ros2 launch elfin_mvp_bringup control.launch.py use_rviz:=false`

```text
ros2 control list_controllers
joint_state_broadcaster  JointStateBroadcaster           active
elfin_arm_controller     JointTrajectoryController       active

ros2 action list
/elfin_arm_controller/follow_joint_trajectory

ros2 topic hz /joint_states
average rate: 100.003   # controller_manager update_rate; JTC state topic is 50 Hz
```

`/joint_states` contains all six `elfin_joint*` names. No `sleep` in launch; spawners wait on controller_manager.

## Gate 4 direct trajectory

```text
ros2 run elfin_mvp_bringup send_joint_trajectory.py --repeat 20 --max-error 0.001 --duration 3
```

Log: `/home/adamliao/.ros/log/python3_2027842_1786675288608.log`

- 20/20 outbound + return succeeded (`error_code=0`).
- Worst absolute joint error: **0.000027 rad** (limit 0.001).
- Typical round-trip: 6.103 s.

## Gate 5 MoveIt 2

```text
ros2 launch elfin_mvp_bringup demo.launch.py use_rviz:=false
ros2 run elfin_mvp_bringup move_to_joint_goal --ros-args -p repeat:=20 -p max_error:=0.01
```

- `move_group` loaded OMPL (`RRTConnect`) and reported `You can start planning now!`
- 20/20 plan+execute round trips succeeded.
- Worst absolute joint error: **0.000100 rad** (limit 0.01).
- Humble process environment: `ROS_DISTRO=humble`, `ROS_VERSION=2`. No ROS 1 bridge. A host Docker Noetic `move_group` may still exist for the Noetic workspace; it is not in the Humble graph.

The client node logs `No kinematics plugins defined` because it only needs joint-space `MoveGroupInterface`. KDL is loaded inside `move_group` via `elfin_moveit_config/config/kinematics.yaml`.

## MVP Definition of Done

- [x] Build and run with ROS 2 Humble only (this workspace).
- [x] S20 model and TF load.
- [x] Two `ros2_control` controllers active.
- [x] Direct trajectory 20/20.
- [x] MoveIt 2 plan+execute 20/20.
- [x] Final joint error within limits.
- [x] Deterministic demo nodes (`send_joint_trajectory.py`, `move_to_joint_goal`).
- [x] Launch/stop leaves no required `sleep`; spawners exit after activate.
- [x] This record.

Humble Docker draft: `Dockerfile.humble` + `docker/run.sh` (`USE_GPU=1` required).
