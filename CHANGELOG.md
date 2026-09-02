# Changelog

## 2026-09-02 — Humble closed-loop pick / place / pack-eval

Work since the 08-28 snapshot, on `ros2_humble` only (`elfin_humble_ws`).

- **Perception**: YOLO detections + stamp-joined `DetectionFrame`, cargo instance
  tracker, occupancy mapper node (`cargo_volume_mapper_node`).
- **Planning**: vacuum attach/follow, pick–retreat, place waypoint sequence
  (staging / insert / descend / retreat) and scene-manager mesh ACM.
- **Gazebo**: `place_smoke_driver`, `pack_eval_driver` (spawn once, keep placed
  via `FinalizeCurrentBox`), physics collision STL.
- **Packing**: `ComputePlacement` returns `elfin_base_link`; floor-prior z is
  usable-volume center; corridor G1 + 1 mm full-Y wall slop; dumps/ledger.
- **Verified**: packing/corridor/waypoint/vacuum unit tests; mapper B1 and
  planner B2 (empty + full-width wall `corridor_blocked`) without Gazebo.
  B4 pack-to-full and B5 place N=3 still need a live `sim_world`.

## 2026-08-28 — ROS 2 Humble workspace snapshot

- Add `elfin_humble_ws/`: colcon workspace for the ROS 2 Humble luggage stack
  (perception, planning, packing, Gazebo, MoveIt 2, docs).
- ML weights, CLIP vendor blobs, and raw LAS scans remain local-only.

## 2026-05-28 — Container inspect pipeline (Phase 1)

- **Container aim**: `aim_camera_at_container` service — look-at IK on `camera_depth_optical_frame`, optional `elfin_link6` XY path constraints, MoveIt plan/execute.
- **Container TF**: `container_tf_publisher` publishes `world → elfin_base_link → container_link → container_opening_frame` from `container.yaml`.
- **Interior inspection**: `container_inspector` (Gazebo GT grid) + `InspectContainer.srv`; orchestrator `AimContainer → InspectContainer` states.
- **MoveIt**: `moveit_with_camera.launch` uses global `/robot_description`; `move_group` action at `/move_group`.
- **Fixes**: IK success detection (`set_joint_value_target` returns `None` on success); `PlanningSceneInterface.add_box` uses `PoseStamped`.

## 2026-05-28 — RealSense D435 mount v1 (production)

- Fixed side mount on `elfin_link6`: xyz `[-0.017202, 0.129806, 0.101650]`, rpy `[π/2, -π/2, 0]` in `camera_mount_origin.xacro`.
- Intel optical frame convention; `elfin_s20_with_camera.urdf.xacro` + `S20_with_camera.srdf`.
- Tune workflow: 6-DOF Gazebo chain, GUI save, `mount_config_utils` tune↔fixed conversion.
- MoveIt bringup: `moveit_with_camera.launch`, `moveit_for_camera_tune.launch`, observe pose reset.

## 2026-05-26 — Luggage loading workspace (Phase 0 skeleton)

- Catkin packages: `luggage_msgs`, `luggage_description`, `luggage_gazebo`, `luggage_planning`, `luggage_perception`, `luggage_packing`, `luggage_bringup`.
- Gazebo sim world, orchestrator state machine, motion planner stubs, vacuum sim, bin packer stub.
- Docker: Noetic desktop-full image with bind-mounted workspace.

## Earlier — Mount design & visualization

- OpenSCAD parametric EOF bracket (`camera_mount.scad`).
- Plotly FK visualizer (`visualize_robot.py`).
- Vendored [Huayan elfin_s_robot](https://github.com/huayan-robotics/elfin_s_robot) (Noetic).
