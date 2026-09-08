# Elfin Humble Workspace Context

Last generated: 2026-09-04.

This repository is a ROS 2 Humble colcon workspace for an Elfin S20 luggage
loading system. The active target is a closed-loop simulated workflow:

```text
pickup_observe -> spawn suitcase -> perceive box -> pick with vacuum
  -> compute placement -> place into container -> commit occupancy map
  -> repeat until BIN_FULL / MAX_BOXES / ABORT
```

The workspace is a git repository as of 2026-09-04 (`git init`).
`build/`, `install/`, `log/`, and model weights (`*.pt`) are gitignored.

## Top-Level Layout

| Path | Role |
|---|---|
| `src/` | ROS packages and Python algorithm libraries |
| `docs/architecture/` | Normative architecture rules; treat violations as defects |
| `docs/plans/` | Migration and feature implementation plans |
| `docs/status/` | Validation records, current state, and evidence pointers |
| `docs/agents/` | Multi-agent notes by role; cross-CLI mailbox is `docs/agents/discuss/OPEN.md` |
| `scripts/` | Standalone eval/helper scripts |
| `robot_assets/` | Original Elfin mesh/URDF assets |
| `third_party/` | Suitcase visual candidates and sources |
| `build/`, `install/`, `log/` | Existing colcon outputs; do not edit manually |
| `yolov8s-world.pt` | YOLO-World checkpoint present at workspace root |

## Package Map

| Package | Build | Main responsibility |
|---|---|---|
| `elfin_description` | `ament_cmake` | S20 URDF/xacro, meshes, RViz display |
| `elfin_control` | `ament_cmake` | `ros2_control` controller YAML for mock/Gazebo |
| `elfin_moveit_config` | `ament_cmake` | MoveIt 2 config for planning group `elfin_arm` |
| `elfin_mvp_bringup` | `ament_cmake` + C++ | MVP control/demo launch and MoveIt joint-goal demo |
| `luggage_msgs` | `ament_cmake` + rosidl | System message/service/action contracts |
| `luggage_description` | `ament_cmake_python` | Scene config, static TF, container geometry, suitcase catalog |
| `luggage_gazebo` | `ament_cmake_python` | Gazebo Fortress launch, spawners, eval drivers, metrics |
| `luggage_perception` | `ament_cmake_python` | Sensor preprocessing, detection, semantic chain, cargo map algorithms/nodes |
| `luggage_packing` | `ament_cmake_python` | EMS/free-space placement and `ComputePlacement` node |
| `luggage_planning` | `ament_cmake_python` | Waypoint generation, MoveIt motion execution, vacuum backend |
| `pymoveit2` | vendored `ament_cmake_python` | Python MoveIt 2 helper library |
| `luggage_bringup` | Catkin/ROS 1 legacy | Unported top-level ROS 1 launch/orchestrator references |
| `pointcloud` | no `package.xml` | Offline point-cloud/CAD/container asset generation tools |

`luggage_bringup` is still ROS 1/catkin. Several `scripts/` files under
`luggage_planning` and `luggage_packing` are also unported ROS 1 references.
Do not extend `rospy` code for ROS 2 work; port the needed behavior into thin
`rclpy` nodes around importable algorithm modules.

## Build And Test

Typical build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Focused build for the active packing stack:

```bash
colcon build --packages-select \
  luggage_description luggage_msgs luggage_perception luggage_packing \
  luggage_planning luggage_gazebo \
  --symlink-install
source install/setup.bash
```

Core algorithm/package tests:

```bash
colcon test --packages-select \
  luggage_description luggage_packing luggage_perception luggage_planning \
  luggage_gazebo
colcon test-result --verbose
```

Historical records:

- Phase 1 MVP gates passed: `docs/status/mvp_gates.md`
- Phase 2 interfaces passed: `docs/status/phase2_interfaces.md`
- Phase 3 algorithm libraries passed with 435 pytest cases:
  `docs/status/phase3_algorithms.md`
- Sensor preprocessor live baseline passed:
  `docs/status/preprocessor_baseline.md`

## Main Launches

Minimal scene TF/robot description:

```bash
ros2 launch luggage_description scene.launch.py use_rviz:=false
```

Primary Gazebo Fortress closed-loop launch:

```bash
export ROS_DOMAIN_ID=7 DISPLAY=:1
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true \
  use_cargo_map:=true use_packing:=true \
  visual_kind:=mesh size_mode:=catalog sequence_ids:=carryon \
  observe_pose_name:=pickup_observe semantic_require_backend:=bbox_fill
```

`sim_world.launch.py` starts:

- Gazebo Fortress world `airport_loading.sdf`
- `/clock` and camera/Livox bridges
- D435 depth metre-to-millimetre republisher
- `sensor_preprocessor_node.py`
- scene static TF and `robot_state_publisher`
- pickup box spawner and scene visualization
- `luggage_detector_node.py`
- optional semantic segmenter + semantic point filter
- optional `scene_manager`, `waypoint_generator`, `motion_planner`
- optional `cargo_volume_mapper`, `placement_planner`, `vacuum_controller`
- controller spawners and MoveIt 2 `move_group`

Important launch args:

| Arg | Meaning |
|---|---|
| `use_semantic` | Start YOLO semantic chain and use semantic cargo cloud |
| `semantic_require_backend` | Fail startup unless backend prefix matches, e.g. `bbox_fill` |
| `use_motion` | Start waypoint + motion planner nodes |
| `use_vacuum` | Start vacuum controller |
| `use_cargo_map` | Start cargo occupancy mapper |
| `use_packing` | Start placement planner |
| `visual_kind` | `box` primitive or `mesh` suitcase visual |
| `size_mode` | `catalog` or `continuous` |
| `sequence_ids` | Catalog sequence, e.g. `carryon,standard,large` |
| `observe_pose_name` | Use `pickup_observe` for detection/eval |

`sim_world.launch.py` has a GPU hard gate and rejects llvmpipe. Use NVIDIA GPU
rendering (`--gpus all` in Docker; `DISPLAY=:1` in existing records).

## ROS Interfaces

`luggage_msgs` defines the cross-package contracts:

- Messages: `DetectedLuggage`, `YoloBox`, `YoloDetections`, `DetectionFrame`,
  `SlotSpec`, `MotionSegment`, `LoadTaskStatus`, `ContainerOpeningEstimate`,
  `VacuumState`
- Services include `DetectLuggage`, `GetCurrentBox`, `SpawnNextBox`,
  `ClearCurrentBox`, `FinalizeCurrentBox`, `ComputePlacement`,
  `BuildMotionSequence`, `VacuumCommand`, `AddPlacedBox`, `RemovePlacedBox`,
  `ResetCargoMap`, `GetCargoMapStats`, `EvaluateCargoViews`,
  `OrchestratorStep`
- Actions include `PlanMotion`, `GoToRobotPose`, `GoToJointValues`,
  `AimCameraAtContainer`, `ValidateMotionSequence`, `PlanNextCargoView`

Common live endpoints:

| Endpoint | Type | Owner |
|---|---|---|
| `/luggage/preprocessed/camera/color/image` | `sensor_msgs/Image` | preprocessor |
| `/luggage/preprocessed/camera/depth/image` | `sensor_msgs/Image` | preprocessor |
| `/luggage/preprocessed/camera/depth/points` | `sensor_msgs/PointCloud2` | preprocessor |
| `/luggage/preprocessed/status` | JSON `std_msgs/String` | preprocessor |
| `/luggage/semantic/mask` | image | semantic segmenter |
| `/luggage/semantic/overlay` | image | semantic segmenter |
| `/luggage/semantic/cargo_points` | `PointCloud2` | semantic point filter |
| `/semantic_segmenter/stats_json` | JSON `std_msgs/String` | semantic segmenter |
| `/luggage/perception/detection_frame` | `DetectionFrame` | detector |
| `/luggage/perception/detection/latest` | JSON `std_msgs/String` | detector |
| `/luggage_detector/detect_luggage` | `DetectLuggage` | detector |
| `/pickup_box_spawner/spawn_next_box` | `SpawnNextBox` | gazebo spawner |
| `/pickup_box_spawner/get_current_box` | `GetCurrentBox` | gazebo spawner |
| `/pickup_box_spawner/clear_current_box` | `ClearCurrentBox` | gazebo spawner |
| `/pickup_box_spawner/finalize_current_box` | `FinalizeCurrentBox` | gazebo spawner |
| `/waypoint_generator/build_motion_sequence` | `BuildMotionSequence` | planning |
| `/motion_planner/plan_motion` | `PlanMotion` action | planning |
| `/vacuum_controller/command` | `VacuumCommand` | planning |
| `/cargo_map/add_placed_box` | `AddPlacedBox` | cargo mapper |
| `/cargo_map/remove_placed_box` | `RemovePlacedBox` | cargo mapper |
| `/cargo_map/reset` | `ResetCargoMap` | cargo mapper |
| `/cargo_map/get_stats` | `GetCargoMapStats` | cargo mapper |
| `/luggage/cargo_map/surface_2d` | JSON `std_msgs/String` | cargo mapper |
| `/luggage/cargo_map/committed` | JSON `std_msgs/String` | cargo mapper |
| `/placement_planner/compute_placement` | `ComputePlacement` | packing |
| `/placement_planner/last_result` | JSON `std_msgs/String` | packing |

Coordinate contract to keep straight:

- `ComputePlacement` returns `SlotSpec.place_pose` in `elfin_base_link`.
- `/cargo_map/add_placed_box` expects the slot pose in `world`; the mapper
  converts world to `container_link`.
- `DetectionFrame.header.frame_id` is the 3D PCA/centroid frame, normally
  `world`; `yolo_optical_frame` names the image plane frame.

## Architecture Rules

The normative source is `docs/architecture/`.

Hard rules for perception/planning changes:

- Algorithm modules may import stdlib/numpy/other algorithm modules only.
  They must not import `rclpy`, `rospy`, `tf2_ros`, or ROS message types.
- Node modules own ROS topics, services, actions, parameters, TF, and message
  conversion. Nodes should stay thin.
- Importable package modules should remain usable under plain `pytest` without
  sourcing ROS when they are algorithm modules.
- Algorithm classes with state use `update(..., stamp, frame_id)` plus
  `copy_output()` and return copies, not internal buffers.
- Every world-describing output carries the acquisition stamp and real frame.
  Republishers inherit input stamps; do not stamp derived data with `now()`.
- Multi-stream alignment belongs only in the sensor preprocessor. Downstream
  algorithm nodes consume preprocessed topics and may exact-join by stamp.
- TF lookups must be at the data stamp. Latest-transform fallback is a defect.
- Gazebo D435 point cloud headers are misleading: cloud data is really in
  `camera_link` although the header says `camera_depth_optical_frame`.
- Far-plane misses are `inf`; filter with `np.isfinite(...).all(axis=1)`.
- Simulated Livox is a raster `gpu_lidar` with no per-point times; treat it as
  `deskewed=false`. Real Mid-360 IMU acceleration is in g and must be converted.

Tracked architecture deviations currently documented:

- `semantic_segmenter.py` still has legacy `segment(rgb_image)` and an
  `instance_map` accessor concern.
- `robot_self_point_filter.py` imports/uses ROS-like TF fallback behavior.
- `cargo_volume_mapper.py` and some planning helpers still build ROS messages
  in algorithm-adjacent code paths.
- Unported `rospy` files remain outside `scripts/ros1_reference/`.

## Active Workflow State

Validated status as of the docs:

- Closed-loop pick/retreat N=20: planning and retreat passed for all
  detect-pass trials; end-to-end 11/20 due to upstream detection misses.
  See `docs/status/closed_loop_eval.md`.
- Vacuum pick Todo 4: evidence exists under `docs/status/evidence/`.
- Todo 5 slice A place action: passed. Vacuum payload N=3 placed 3/3 into the
  container, `descend` fraction 1.0, no payload loss. See
  `docs/status/todo5_place_action.md`.
- Todo 5 B-D code paths are wired: `cargo_volume_mapper_node.py`,
  `placement_planner_node.py`, and `pack_eval_driver.py` exist and are in
  `sim_world.launch.py`.
- Packing smoke carryon n=2 passed with `termination_reason=MAX_BOXES`, two
  boxes committed, `floor_coverage=0.15`. See
  `docs/status/packing_eval_carryon_n2.md`.
- A previous carryon n50 capacity attempt was invalid because dual Gazebo/RTF
  problems caused an abort, not a capacity conclusion.

Likely next meaningful validation:

```bash
ros2 run luggage_gazebo pack_eval_driver.py \
  --sequence-ids carryon --max-boxes 50 --goto-timeout 60 \
  --out docs/status/evidence/packing_eval_carryon_n50
```

Expected capacity run outcome should be `BIN_FULL` with a non-empty
`reject_histogram`; otherwise inspect `PLACE_*`, detection, or graph errors
before calling it a packing capacity result.

## Evaluation Drivers

Useful drivers and their intent:

| Command target | Purpose |
|---|---|
| `scripts/detection_gt_gate_run.py` | N-trial `DetectLuggage` vs GT sampling |
| `scripts/yolo_two_class_window.py` | YOLO two-class window stats |
| `scripts/todo3_pick_driver.py` | Detect -> build pick sequence -> 4x `PlanMotion` |
| `ros2 run luggage_gazebo pick_retreat_eval_driver.py` | Pick/retreat eval baseline |
| `ros2 run luggage_gazebo place_smoke_driver.py` | Single-slot place smoke/regression |
| `ros2 run luggage_gazebo pack_eval_driver.py` | Multi-box pack-to-full eval |
| `ros2 run luggage_packing packing_replay_eval.py` | Offline packing replay/ablation |

Evidence usually lands under `docs/status/evidence/<run_name>/` with JSONL,
summary JSON/Markdown, and optional image/PLY/HTML dumps.

## Important Implementation Notes

- Joint names are `elfin_joint1` through `elfin_joint6`.
- Planning group is `elfin_arm`.
- `pickup_observe` is the detection/eval pose; the generic `observe` pose is
  not a clean top-down suitcase view.
- `move_group` must use the camera/suction SRDF override in
  `luggage_description/config/S20_with_camera.srdf` to avoid self-collision
  between added wrist hardware and `elfin_link6`.
- MoveIt 2 Humble serves `MoveGroup` action at `/move_action`.
- `PlanMotion` actually plans and executes, matching the ROS 1 semantics.
- `keep_camera_down` and `lock_wrist` are still not implemented in the motion
  stack; successful messages may carry `NOT_IMPLEMENTED` notes.
- Vacuum sim backend follows at about 30 Hz through Gazebo `/world/.../set_pose`.
  Do not detach before place `descend`/release.
- `FinalizeCurrentBox` keeps the Gazebo box model after successful place;
  `ClearCurrentBox` deletes the current pickup model and is not the right
  commit operation for pack eval.
- Running launch reads the install tree. After editing launch/nodes, rebuild
  and re-source before testing.
- Avoid `pkill -f "ign gazebo"` in shared sessions. Existing docs prefer an
  isolated `ROS_DOMAIN_ID` and tracking launch process groups. After an
  agent-run eval, stop with `scripts/stop_sim.sh` (SIGINT launch, then
  leftover PIDs). Inspect `dumps/` / `final_layout/`; do not leave GUI sim
  occupying GPU.

## Docker / Runtime

- `Dockerfile.humble` is the ROS 2 draft image based on
  `osrf/ros:humble-desktop`.
- The root `Dockerfile` is Noetic-era and not the Humble target.
- Interactive Gazebo/RViz requires NVIDIA GPU runtime. The launch GPU gate runs
  `scripts/check_gpu_renderer.sh` and fails if the renderer is llvmpipe.

