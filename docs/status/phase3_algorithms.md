# Phase 3 algorithm libraries

Date: 2026-08-14  
Workspace: `elfin_humble_ws`  
Build:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select luggage_description luggage_packing luggage_perception luggage_planning
```

Tests:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -c "from luggage_packing.ems import EMS; from luggage_description.scene_tf_config_utils import load_scene_tf_config"
colcon test --packages-select luggage_description luggage_packing luggage_perception luggage_planning
colcon test-result --verbose
```

Result: **passed** (435 pytest cases: description 59, packing 73, perception 136, planning 167). No skips.

Four Catkin packages were converted in place to `ament_cmake` + `ament_cmake_python`. Algorithms are in-process importable libraries (`from luggage_packing.ems import EMS`). No ROS 2 nodes, no perception C++, no Gazebo/MoveIt. `COLCON_IGNORE` remains on `luggage_bringup` and `luggage_gazebo`.

`colcon list` is nine ROS 2 packages: the four MVP packages, `luggage_msgs`, and these four algorithm packages.

## Package layout

Each package:

```text
src/<pkg>/
  package.xml              # format 3, ament_cmake
  CMakeLists.txt           # ament_python_install_package + share install
  <pkg>/                   # importable modules (moved from scripts/)
  scripts/                 # leftover ROS 1 nodes, not installed
  test/                    # copied from noetic, package imports
```

## Resource lookup

`rospkg.RosPack()` and `rospy.get_param` are gone from the libraries.

- Default YAML paths: `ament_index_python.get_package_share_directory("luggage_description")` via `luggage_description._share`.
- Explicit `path=` arguments win.
- Optional env `LUGGAGE_SCENE_TF_CONFIG` (not the parameter server).
- Optional env `LUGGAGE_GAZEBO_SHARE`. If `luggage_gazebo` is still `COLCON_IGNORE`, mesh helpers walk to `src/luggage_gazebo` (resolving install-space symlinks).

Share installs:

- `luggage_description`: `config/` (excluding `backups/`) and `urdf/`
- `luggage_planning`: `data/` (reachability atlas `.npz` / YAML)

## Message decoupling

`luggage_planning.waypoint_generator` no longer imports `geometry_msgs` or `luggage_msgs`. It uses dataclasses in `luggage_planning.pose`:

| Type | Fields |
|---|---|
| `Point` | `x y z` |
| `Quaternion` | `x y z w` |
| `Pose` | `position`, `orientation` |
| `MotionSegment` | same names as the ROS 1 msg: `name`, `type`, `target_pose`, `waypoints`, `keep_tool_down`, `keep_camera_down`, `lock_wrist`, `allow_ompl_fallback` |

Node-side conversion to ROS 2 msgs is Phase 6/9.

`downward_constraint_utils.compute_downward_orientations` takes `cam_to_suc_xyzw` (x, y, z, w). TF lookup stays in the Phase 6 node.

`cargo_volume_mapper.publish_params(rospy_module)` is still duck-typed; the module does not import rospy.

Two conversion helpers still lazily import `geometry_msgs` if called (`vacuum_attach_utils.lists_to_pose`, `container_aim_utils.optical_pose_look_at`). The unit tests do not call them. Phase 6 can switch those to `luggage_planning.pose`.

## Migrated modules

### `luggage_description`

| Module | Role |
|---|---|
| `scene_tf_config_utils` | scene YAML + container/pedestal geometry |
| `box_catalog_utils` | catalog, size sampling, mass model |
| `exploration_config_utils` | exploration YAML |
| `scene_mesh_utils` | Gazebo mesh path resolution |
| `joint_angle_utils` | wrap-equivalent joint rewrite |
| `log_level_utils` | env log-level constants (no rospy import) |
| `_share` | ament_index / env path helpers |

Not installed: `description_params_node.py`, `gazebo_urdf_utils.py`, `xacro_robot_with_scene_base.py`.

### `luggage_packing`

`ems`, `placement_solver`, `placement_scoring`, `free_space_model`, `insertion_corridor`, `value_estimator`, `placement_reachability`, `packing_replay`.

Not installed: `bin_packer_node.py`, `placement_planner_node.py`, `placement_markers.py`, `packing_replay_eval.py`, `packing_score_ablation.py`.

### `luggage_perception`

`cargo_volume_mapper`, `voxel_log_odds`, `world_scene_mapper`, `luggage_box_estimator`, `detect_overlay`, `depth_realism_filter`, `robot_self_point_filter`, `motion_stability_filter`, `known_scene_point_filter`, `semantic_point_filter`, `semantic_segmenter`, `container_opening_estimator`.

Not built: `src/task_cloud_filter_node.cpp`, `task_roi.cpp`, `urdf_robot_self_mask.cpp` (Phase 5).  
Not installed: all `*_node.py`.

`exec_depend` on `luggage_description` because the mapper and known-scene filter read `scene_tf` helpers.

### `luggage_planning`

`waypoint_generator`, `pose`, `settle_criterion`, `smart_explore_termination`, `downward_constraint_utils`, `geometry_view_generator`, `container_aim_utils`, `constrained_view_planner`, `interior_probe_planner`, `interior_view_scorer`, `cargo_nbv_planner`, `reachability_atlas`, `reachability_wavefront`, `layout_atlas`, `container_floor_geometry`, `vacuum_attach_utils`, `vacuum_retention`.

`reachability_wavefront` holds the MoveIt-free wavefront helpers extracted from `reachability_atlas_builder.py`. The builder class and IK probes stay in `scripts/`.

Not installed: `*_node.py`, `*_viz.py`, `reachability_atlas_builder.py`, `ik_probe_ablation.py`.

`exec_depend` on `luggage_description` (`geometry_view_generator` reads `scene_tf`).

## Tests not copied (ROS graph / node mocks)

| Package | Skipped files |
|---|---|
| planning | `test_smart_explore_state`, `test_replan_logic`, `test_dynamic_scene_manager`, `test_scene_manager_pickup`, `test_placement_motion_filter` |
| perception | `test_motion_gated_pointcloud_relay` |
| packing | `test_bin_packer` (node-bound) |

`test_atlas_builder_wavefront` covers the extracted wavefront helpers only. Tests that instantiated `ReachabilityAtlasBuilder` were dropped.

Missing atlas `.npz` would skip rather than fail; this checkout has `data/reachability_atlas/s20_container_collision_aware*.npz`, so those cases ran.

## Dependencies

| Package | `exec_depend` |
|---|---|
| `luggage_description` | `ament_index_python`, `python3-yaml` |
| `luggage_packing` | `ament_index_python`, `python3-numpy`, `python3-yaml` |
| `luggage_perception` | `ament_index_python`, `luggage_description`, `python3-numpy` |
| `luggage_planning` | `ament_index_python`, `luggage_description`, `python3-numpy`, `python3-yaml` |

Perception tests that call `luggage_packing.placement_solver` / `luggage_planning.waypoint_generator` declare those as `test_depend`.

## Not in this phase

Phase 4 TF broadcast node, Phase 5 `rclcpp` cloud components, Phase 6 MoveIt adapter, deleting leftover ROS 1 node files under `scripts/`.
