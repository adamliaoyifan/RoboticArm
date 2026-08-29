# Phase 7A (part 1): Gazebo Fortress backend switch — acceptance record

Date: 2026-08-19
Scope: Fortress world + gz_ros2_control backend switch + D435 sensor (M1 of
[sim closed-loop plan](../plans/ros2_sim_closed_loop_plan.md)).
Single-box pick-place (Phase 7A full exit) still pending M2–M4.

## What was done

- `luggage_gazebo` rewritten as an `ament_cmake` ROS 2 package (was catkin,
  `COLCON_IGNORE` removed): Fortress world `worlds/airport_loading.sdf`
  (inlined sun/ground_plane; Physics + UserCommands + SceneBroadcaster +
  Sensors/ogre2 systems), noetic-era `models/` reused verbatim,
  `config/` dropped (Classic PID configs obsolete under gz_ros2_control).
- `elfin_description/urdf/S20.urdf.xacro`: new `fixed_world` arg — `false`
  drops the `world` link/`world_base` joint so the model spawns floating at
  the scene base pose (pedestal contact holds the base), matching noetic
  behavior.
- `luggage_description/urdf/realsense_d435.urdf.xacro`: Classic
  `libgazebo_ros_openni_kinect` sensor replaced by Fortress `rgbd_camera`
  sensor (same FOV 1.5184, clip 0.105–3.0 m, 30 Hz, 640x480;
  `gz_frame_id=camera_depth_optical_frame`).
- `luggage_description/urdf/elfin_s20_with_camera.urdf.xacro`: new
  `use_gz_sim`/`controllers_yaml` args emit the in-model
  `ign_ros2_control::IgnitionROS2ControlPlugin` (class name verified from
  `libign_ros2_control-system.so`; the README name
  `ignition::gazebo::systems::Ros2Control` does NOT exist in this build).
- `elfin_control/config/elfin_controllers_sim.yaml`: Gazebo-relaxed JTC
  tolerances (noetic `elfin_arm_control_sim.yaml` lesson: goal 0.3/0.5 rad,
  trajectory check off, goal_time 5 s).
- `xacro_robot_with_scene_base` helper: forwards `key:=value` xacro args and
  tolerates a missing `world_base` joint (floating gz URDF).
- `luggage_gazebo/launch/sim_world.launch.py`: GPU hard gate (llvmpipe →
  refuse to start), ros_gz_sim world, robot spawned via `ros_gz_sim create`
  from a floating URDF written to `/tmp/elfin_s20_gazebo.urdf`, scene models
  (pedestal/pickup_platform/container) as per-model `create` nodes driven by
  `scene_tf.yaml`, `/clock` bridge, controller spawners chained
  JSB → arm controller.
- `elfin_mvp_bringup/scripts/send_joint_trajectory.py`: replaced
  `Future.result(timeout=…)` (not Humble rclpy API) with polling wait.

## Key findings

- libsdformat rewrites URDF `package://<pkg>/…` mesh URIs to
  `model://<pkg>/…`; each package's `install/<pkg>/share` must be on
  `GZ_SIM_RESOURCE_PATH` (set by the launch; also exported by the package
  env hook).
- The `/world/<w>/create` ROS service only exists while a `ros_gz_sim
  create` process is alive; persistent spawners must not rely on it.
- Fortress `rgbd_camera` publishes image, depth_image, camera_info AND
  points — no `depth_image_proc` needed for M2.

## Results

| Check | Result |
|---|---|
| `colcon build` (10 pkgs, Release) | all pass |
| Fortress spawn (S20 + pedestal + pickup_platform + container) | 4/4 entities created, 0 mesh errors |
| Controllers in gz (`joint_state_broadcaster`, `elfin_arm_controller`) | both `active`; joint2 holds −0.5 initial value (no gravity sag) |
| FJT in Fortress, MVP goal, 20 repeats | **20/20, worst abs err 0.002709 rad** (budget ≤ 0.05 sim) |
| D435 gz topics | `/d435/{image,depth_image,camera_info,points}` live, XYZRGB, frame_id `camera_depth_optical_frame` |
| Mock backend regression (demo.launch.py, 3 repeats, 0.001 rad) | 3/3, pass |
| `luggage_description` unit tests | 77 passed |
| GPU gate | NVIDIA renderer required by launch; `ogre2` engine loads |

## Open items (next milestones)

- Bridge camera topics to ROS with ros_gz_bridge (M2) and align QoS/frame
  semantics with the noetic `/camera/*` names.
- Observe pose init (deferred to M3 where the business layer needs it).
- Vacuum/suction sim, single-box pick-place loop (M4) = Phase 7A full exit.
