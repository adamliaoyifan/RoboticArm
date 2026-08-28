# ROS 2 Humble migration TODO

Workspace: `elfin_humble_ws` only. `elfin_noetic_ws` is read-only.  
Parent plan: [ros2_humble_mvp_and_migration_plan.md](ros2_humble_mvp_and_migration_plan.md)  
Sim research: [sim_backends_ros2.md](sim_backends_ros2.md)

Unchecked items below are **not approved for coding in this batch** except Gate 0.5 (done), Phase 1 Gates 1–5 (done), Phase 2 (done), Phase 3 (done), and Phase 4 (done). Phase 5–10 stay documented until separately approved.

## Gate 0 environment

- [x] Ubuntu 22.04 + Humble desktop
- [x] `ros-humble-moveit` / `ros2-control` / `ros2-controllers` / `controller-manager` / xacro / rviz2
- [x] `hardware_interface` ships `mock_components/GenericSystem`
- [x] rosdep skips ROS 1 keys and Gazebo Classic
- [x] Record RMW=`rmw_fastrtps_cpp`, kernel, CPU, Release — [gate0_environment.md](../status/gate0_environment.md)

## Gate 0.5 simulators and GPU

- [x] Install Fortress: `ros-humble-ros-gz`, `ros-humble-ros-gz-sim`, `ros-humble-gz-ros2-control`
- [x] Install MuJoCo: `ros-humble-mujoco-ros2-control`, demos, msgs
- [x] GPU hard gate — [gpu_runtime.md](../status/gpu_runtime.md)
- [x] `gz sim --versions` / `ign gazebo --versions` is Fortress (`6.18.0`)
- [x] `ros2 pkg prefix mujoco_ros2_control` succeeds
- [x] Do not install Isaac this batch; do not install `ros-humble-gazebo-ros`. Host already has `~/isaacsim` 5.1; Kit not started.

## Gate 1 isolate ROS 1

- [x] Delete `src/CMakeLists.txt` Noetic `toplevel.cmake` symlink
- [x] `COLCON_IGNORE` on `luggage_bringup` `luggage_description` `luggage_gazebo` `luggage_msgs` `luggage_packing` `luggage_perception` `luggage_planning`
- [x] Keep `pointcloud/` in place; it is not a ROS package
- [x] Do not delete ROS 1 sources
- [x] Exit: `colcon list` contains only ROS 2 packages

## Gate 2 `elfin_description`

- [x] Copy S20 STL, `materials.xacro`, `S20.urdf.xacro` from `robot_assets/elfin_description`
- [x] `ament_cmake` package installing urdf, meshes, rviz, launch
- [x] Humble xacro: `$(find elfin_description)` (ament share; this xacro has no `find-pkg-share`)
- [x] Drop Classic `elfin_robot.gazebo` from the mock path
- [x] xacro arg `hardware_plugin`, default `mock_components/GenericSystem`
- [x] Six joints: position command, position/velocity state
- [x] Joint names `elfin_joint1`–`elfin_joint6`
- [x] RViz2 config: RobotModel + TF only
- [x] Exit: xacro expands; URDF has no broken links — [mvp_gates.md](../status/mvp_gates.md)

## Gate 3 `elfin_control`

- [x] YAML: `joint_state_broadcaster` + `elfin_arm_controller` (`JointTrajectoryController`)
- [x] Update 100 Hz, state 50 Hz, non-zero goal/path tolerances
- [x] `control.launch.py`: `ros2_control_node` + spawners, no `sleep`
- [x] Exit: both controllers `active`; `follow_joint_trajectory` exists; `/joint_states` has six joints at a stable rate

## Gate 4 direct trajectory

- [x] Conservative six-joint goal (not near limits, not all zeros as the only test)
- [x] Outbound + return to a safe start
- [x] Log goal, feedback, duration, final error under `docs/status/`
- [x] Exit: 20/20 success; mock max abs error 0.000027 rad ≤ 0.001; no lifecycle faults

## Gate 5 `elfin_moveit_config` + bringup

- [x] SRDF group `elfin_arm`; EE link; disabled collisions
- [x] `joint_limits.yaml` `kinematics.yaml` `ompl_planning.yaml`
- [x] MoveIt Simple Controller Manager → the same `FollowJointTrajectory`
- [x] C++ `MoveGroupInterface` node `move_to_joint_goal`
- [x] `demo.launch.py`: mock + controllers + move_group; `use_rviz`
- [x] Release: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
- [x] Exit: plan+execute 20/20; error 0.000100 rad ≤ 0.01; Humble-only test graph
- [x] Humble Dockerfile draft + GPU passthrough (`Dockerfile.humble`, `docker/run.sh`)

Phase 1 exit: original MVP Definition of Done — [mvp_gates.md](../status/mvp_gates.md). Sim smokes: [sim_smoke.md](../status/sim_smoke.md).

## Phase 2 `luggage_msgs`

Keep field semantics. Use `rosidl_generate_interfaces`. Acceptance: [phase2_interfaces.md](../status/phase2_interfaces.md).

Msg (5): `DetectedLuggage` `SlotSpec` `ContainerOpeningEstimate` `MotionSegment` `LoadTaskStatus`

Keep as short services: `GetCurrentBox` `ClearCurrentBox` `SyncPickupBox` `FinalizeCurrentBox` `ResetCargoMap` `GetCargoMapStats` `GetNextSlot` `VacuumCommand` `AddPlacedBox` `RemovePlacedBox` `SyncStaticScene` `InspectContainer` `DetectLuggage` `VerifyPlacedBox` `IntegrateCargoView` `EvaluateCargoViews` `OrchestratorStep` `SpawnNextBox` `ComputePlacement` `BuildMotionSequence`

Convert to Actions (cancel / feedback / timeout / idempotency required):

- [x] `PlanMotion` / `GoToJointValues` / `GoToRobotPose` / `ValidateMotionSequence` (`BuildMotionSequence` stays a Service; geometric only)
- [x] `PlanNextCargoView` / `AimCameraAtContainer`
- [x] Packing main flow: orchestrator-only state machine (`OrchestratorStep` + `LoadTaskStatus`; no cycle Action)
- [x] `SpawnNextBox`: sim-backend service, not a hardware plugin
- [x] Exit: package builds; Python/C++ typesupport loads; semantics recorded in `docs/status/phase2_interfaces.md`

## Phase 3 algorithms (no ROS graph)

Acceptance: [phase3_algorithms.md](../status/phase3_algorithms.md).

- [x] packing: EMS, placement solver, scoring, free-space, insertion corridor
- [x] perception: voxels/2.5D, geometry, detection postprocess, depth filter (non-rclpy)
- [x] planning: viewpoint generation, constraints, atlas, settle criterion
- [x] description: YAML/geometry/`scene_tf` parsing without `rospkg` / `sys.path`
- [x] `ament_python` or hybrid; resources via `ament_index`
- [x] Exit: packing/perception unit tests pass on the ROS 2 package path

## Phase 4 scene truth

- [x] One ROS 2 node reads `scene_tf.yaml`, `StaticTransformBroadcaster`, transient-local
- [x] TF chain matches ROS 1: `world → pedestal_link / elfin_base_link / container_link → container_opening_frame`
- [x] `robot_description`, SRDF, limits are node parameters
- [x] Remove `description_params_node` broadcasting into other nodes
- [x] Install suction/camera origin xacro with description
- [x] Exit: TF/geometry numerically match the ROS 1 baseline

Acceptance: [phase4_scene_tf.md](../status/phase4_scene_tf.md).

## Phase 5 perception

- [ ] C++ `task_cloud_filter` → `rclcpp` component
- [ ] Other cloud nodes as `rclcpp_components` + intra-process
- [ ] Image/PointCloud2: `SensorDataQoS` best_effort keep_last(1)
- [ ] `message_filters` pairing; TF by stamp; fail-closed
- [ ] Torch may stay; `rclpy` only shuttles data
- [ ] Per-frame `set_param` → diagnostics topic
- [ ] Exit: fixed bag semantics; P95/throughput meet targets; queues do not grow

## Phase 6 MoveIt 2 business planning

- [ ] Split `motion_planner_node.py`: C++ adapter, execute Action, constraint lib, settle/hold
- [ ] PlanningScene / AttachedCollisionObject / ACM
- [ ] Batch IK/validity via library calls
- [ ] MultiThreadedExecutor + callback groups
- [ ] Exit: fixed-scene success/constraints/collision/execution match ROS 1 baseline

## Phase 7A Fortress product sim (GPU hard gate)

- [x] `luggage_gazebo` → `ros_gz_sim` world (2026-08-19 - [phase7a_gz_backend.md](../status/phase7a_gz_backend.md))
- [x] Container/pedestal/box SDF for Fortress; STL URIs valid (noetic models reused, resource-path fix)
- [x] spawn/delete/get_state → `ros_gz` (per-model `ros_gz_sim create` nodes)
- [ ] D435 → gz RGB-D + bounded bridge (sensor live: gz `/d435/{image,depth_image,camera_info,points}`; ROS bridge pending M2)
- [x] `hardware_plugin:=gz_ros2_control/GazeboSimSystem` (mock→Fortress switch works; FJT 20/20 @ 0.0027 rad)
- [ ] Suction as gz plugin or a sim-only node; vacuum service name matches hardware
- [x] Launch runs the GPU check and exits on `llvmpipe`
- [ ] Exit: same MoveIt/controller config switches mock→Fortress; single-box pick-place repeats; renderer is NVIDIA

## Phase 7B MuJoCo contrast

- [ ] Expand S20 URDF → hand-edit `scene.xml`; joint names unchanged
- [ ] Container/pedestal geoms posed from `scene_tf.yaml`
- [ ] Package `ros2_control_node` + `MujocoSystemInterface`
- [ ] Same fixed joint goal, same error budget as mock
- [ ] One contact place; record physics delta vs Fortress
- [ ] Do not switch perception unless camera quality is measured
- [ ] Exit: GPU window; `FollowJointTrajectory` 20-shot contrast passes

## Phase 7C Isaac (docs and smoke; does not block packing)

- [ ] Do not install in Gate 0.5; install Isaac Sim in this phase
- [ ] URDF/STL → USD; `scene_tf.yaml` → Xform
- [ ] Two processes: Sim (bundled Humble libs) + `elfin_humble_ws`
- [ ] Read-only `/clock`, image, `joint_states` first; then FJT adapter
- [ ] Load the same USD in Isaac Lab as one env smoke
- [ ] Exit: Humble sees image and joints; one conservative joint goal. Lab does not run MoveIt

## Phase 8 Huayan hardware

- [ ] Gate 8A SDK check (headers, `.so`, ABI, e-stop) before motion
- [ ] Gate 8B `SystemInterface`; no blocking I/O on the control loop
- [ ] Gate 8C: read-only → enable → single joint → six joints → FJT → MoveIt → business
- [ ] If Jammy ABI fails, isolated-process IPC; never hard-link a bad `.so`
- [ ] Keep the mock backend

## Phase 9 orchestrator / GUI

- [ ] rospy → rclpy; hot paths rclcpp
- [ ] State machine + Action; no global parameter shared memory
- [ ] lifecycle / ready services; delete `sleep 5/20/25`
- [ ] GUI subscribes to feedback/status only
- [ ] Exit: mock + Fortress full cycle is repeatable

## Phase 10 cleanup

- [ ] rosbag2; RViz2; CLI is all `ros2`
- [ ] Dockerfile = Ubuntu 22.04 / Humble / colcon / GPU
- [ ] Delete ROS 1 packages only after the ROS 2 equivalent is accepted
- [ ] 5.1 GB `elfin-noetic.tar` needs an explicit user confirm
- [ ] Exit: no live catkin/rospy/roscpp/actionlib/roslaunch/rosbag/Noetic build deps
