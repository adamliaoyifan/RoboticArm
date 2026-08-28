# Phase 4 scene truth and static TF

Date: 2026-08-14  
Workspace: `elfin_humble_ws`  
Package: `luggage_description` (first Humble `rclpy` node in `luggage_*`)

Build / test:

```bash
cd elfin_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select luggage_description
source install/setup.bash
xacro $(ros2 pkg prefix luggage_description)/share/luggage_description/urdf/elfin_s20_with_camera.urdf.xacro \
  > /tmp/s20_cam.urdf
ros2 launch luggage_description scene.launch.py
# other terminal:
#   ros2 run tf2_ros tf2_echo world container_link
#   ros2 run tf2_ros tf2_echo world elfin_base_link
colcon test --packages-select luggage_description
colcon test-result --verbose
```

Result: **passed** (`luggage_description` 77 pytest cases, 0 skipped). Live lookups in `test_scene_tf_live.py` match the library pins below.

Out of scope (unchanged): `COLCON_IGNORE` on `luggage_bringup` / `luggage_gazebo`; perception C++; `move_group` / SRDF camera–suction pairs (Phase 6); Fortress; rewriting `description_params_node` to inject `/task_cloud_filter`.

## Who publishes which edge

Default YAML: share `config/scene_tf.yaml.example` (same numbers as the ROS 1 launch baseline).

| Edge | Publisher | Source |
|---|---|---|
| `world → pedestal_link` | `container_tf_publisher` | `static_transforms(config)` |
| `world → pickup_platform_link` | same | same |
| `pickup_platform_link → pickup_platform_top` | same | same |
| `world → container_link` | same | same |
| `container_link → container_opening_frame` | same | same |
| `world → elfin_base_link` | `robot_state_publisher` | URDF `world_base` patched by `urdf_world_base_pose` |
| arm / `suction_panel` / `camera_depth_optical_frame` | `robot_state_publisher` | `elfin_s20_with_camera.urdf.xacro` |

`elfin_base_link` is **not** on `/tf_static` from the scene publisher. That matches ROS 1 `static_transforms()`.

Humble `robot_state_publisher` only emits revolute/prismatic TF after `/joint_states`. `scene.launch.py` therefore runs `zero_joint_state_publisher` (all non-fixed joints at 0). That is not `ros2_control`.

## QoS and parameters

`tf2_ros.StaticTransformBroadcaster` in Humble is transient-local + reliable (latched `/tf_static` equivalent). `republish_period` defaults to `0.0`. ROS 1 defaulted to `30.0` because latched late-joiners were less reliable; do not restore a timer unless a subscriber still misses the snapshot.

Node name stays `container_tf_publisher`. Parameters are **this node only**:

| Param | Default | Meaning |
|---|---|---|
| `scene_tf_config` | `""` → `resolve_scene_tf_config_path()` | explicit path, else `LUGGAGE_SCENE_TF_CONFIG`, else share example |
| `republish_period` | `0.0` | seconds; `0` means publish once |
| `task_roi.*` | derived | logged and `ros2 param get`-able on this node |

There is no `/luggage/scene_tf_config` and no write into another node’s namespace. Phase 5 C++ nodes must declare their own parameters.

`robot_description` is a parameter on `robot_state_publisher` (and on the zero-joint publisher / optional RViz), not a global dump. SRDF and joint limits stay on the Phase 1 `elfin_moveit_config` nodes; this phase does not start `move_group`.

## Launch

| File | What it starts |
|---|---|
| `launch/scene_tf.launch.py` | publisher only |
| `launch/scene.launch.py` | publisher + xacro helper → RSP + zero joint states; `use_rviz` default false |

```python
Command([helper, xacro_path, scene_tf_config])
```

`xacro_robot_with_scene_base` expands the xacro and patches `world_base` origin. Plain `xacro …/elfin_s20_with_camera.urdf.xacro` still has `world_base` at the origin; that is expected.

ROS 1 XML under `launch/*.launch` is not installed (`PATTERN "*.launch" EXCLUDE`).

## ROS 1 numeric pins (`scene_tf.yaml.example`)

| Quantity | Value |
|---|---|
| `world → container_link` translation | `[1.5, 0, 0]` |
| `robot_base_in_world` / `urdf_world_base_pose` | xyz `[0, 0, 0.86]`, rpy yaw `1.5708` |
| `container_in_base_link` translation | `≈ [0, -1.5, -0.86]` |
| `container_opening_target_point` | `≈ [-0.27, -0.755, 0.44]` |
| task ROI usable span | `1.49 × 1.97 × 1.48` m (center not at the origin) |
| RPY → quat | same formula as ROS 1 `_make_transform` |

Live `tf2_buffer.lookup_transform` in `TestSceneLaunchGraph` checks `world→container_link`, `world→elfin_base_link`, `elfin_base_link→container_opening_frame`, and that `camera_depth_optical_frame` and `suction_panel` exist.

## Code map

| Path | Role |
|---|---|
| `luggage_description/scene_tf_config_utils.py` | `task_roi_from_scene`, `static_transforms`, `urdf_world_base_pose` |
| `luggage_description/scene_tf_publisher.py` | quat + `TransformStamped` + rclpy body |
| `luggage_description/xacro_robot_with_scene_base.py` | expand + patch `world_base` |
| `luggage_description/zero_joint_state_publisher.py` | zeros for Humble RSP |
| `scripts/container_tf_publisher` | installed entry |
| `scripts/xacro_robot_with_scene_base` | installed entry |
| `scripts/zero_joint_state_publisher` | installed entry |
