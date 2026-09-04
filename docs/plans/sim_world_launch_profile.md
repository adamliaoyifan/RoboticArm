# sim_world Launch Profile

Date: 2026-09-04

`luggage_gazebo/launch/sim_world.launch.py` supports a grouped YAML launch
profile:

```bash
ros2 launch luggage_gazebo sim_world.launch.py \
  profile_config:=/home/adamliao/work/elfin_humble_ws/src/luggage_gazebo/config/sim_world.profile.yaml
```

Precedence:

```text
built-in launch fallback < profile_config YAML < CLI launch arg
```

Example CLI override:

```bash
ros2 launch luggage_gazebo sim_world.launch.py \
  profile_config:=/path/to/sim_world.profile.yaml \
  use_rviz:=true sequence_ids:=carryon
```

The profile can be grouped for readability. The launch file flattens nested
YAML keys, so `scene.observe_pose_name` maps to the launch argument
`observe_pose_name`.

## Parameter Reference

| YAML group | Parameter | Function |
|---|---|---|
| CLI only | `profile_config` | Grouped YAML profile path. This selects the file to read. |
| `scene` | `scene_tf_config` | Scene TF YAML path. Omit to use the package example fallback. |
| `scene` | `robot_poses_config` | Named robot poses YAML path. Omit to use the package example fallback. |
| `scene` | `observe_pose_name` | Named observe pose used for initial spawn/hold, usually `observe` or `pickup_observe`. |
| `runtime` | `gui` | Start Gazebo with GUI when `true`; headless server when `false`. |
| `runtime` | `use_rviz` | Start RViz with `sim_full.rviz`. |
| `runtime` | `use_moveit` | Start `move_group` after arm controller activation. |
| `runtime` | `spawn_at_observe` | Spawn the simulated arm at `observe_pose_name` instead of joint zero. |
| `features` | `use_semantic` | Start YOLO segmenter and semantic point filter; detector consumes cargo cloud. |
| `features` | `use_motion` | Start scene manager, waypoint generator, and motion planner. |
| `features` | `use_vacuum` | Start vacuum controller simulation backend. |
| `features` | `use_cargo_map` | Start cargo volume mapper. |
| `features` | `use_packing` | Start placement planner. |
| `perception` | `semantic_require_backend` | Require a semantic backend prefix, e.g. `bbox_fill`; empty disables the guard. |
| `spawner` | `visual_kind` | Pickup box visual type: `box` primitive or `mesh` suitcase model. |
| `spawner` | `size_mode` | Spawn sizes from `catalog` or `continuous` sampling. |
| `spawner` | `sequence_ids` | Ordered catalog IDs for spawned boxes; empty means weighted random. |
| `spawner` | `yaw_mode` | Override catalog yaw behavior, e.g. `discrete` or `continuous`; empty uses catalog/default. |
| `spawner` | `yaw_range` | `[min, max]` yaw range used by continuous yaw mode. |
| `spawner` | `xy_jitter_range` | `[x_half_width, y_half_width]` pickup source jitter in metres. |
| `motion` | `named_pose_duration` | Max named-pose trajectory duration in seconds. |
| `motion` | `named_pose_max_vel` | Nominal named-pose joint speed in rad/s. |

## Notes

- Unknown YAML keys are ignored with a launch warning.
- CLI args still work without a profile; omitted YAML keys use the same
  fallback values the launch file had before profile support.
- Path-like values should normally be absolute when edited by hand.
- This profile controls launch-level wiring. Node parameter YAML files such as
  `sensor_preprocessor.yaml` remain separate.
