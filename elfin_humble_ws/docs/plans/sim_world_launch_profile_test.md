# sim_world Launch Profile Test Flow

Date: 2026-09-04

Scope: validate the `profile_config` YAML support in
`luggage_gazebo/launch/sim_world.launch.py`.

This test flow does not validate perception, planning, or Gazebo physics
quality. It validates that launch-level parameters can be edited through a
grouped YAML profile, that CLI overrides still work, and that invalid profiles
fail clearly.

## Preconditions

Run from the workspace root:

```bash
cd /home/adamliao/work/elfin_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Use a writable ROS log directory when running inside a restricted environment:

```bash
export ROS_LOG_DIR=/tmp/ros2-launch-log
```

Reference profile:

```text
src/luggage_gazebo/config/sim_world.profile.yaml
```

## Test 1: Python Syntax

Command:

```bash
python3 -m py_compile src/luggage_gazebo/launch/sim_world.launch.py
```

Expected:

- Command exits with code `0`.
- No syntax errors are printed.

Acceptance:

- Pass if the launch file compiles.
- Fail if Python reports syntax, indentation, import, or parse errors.

## Test 2: Profile YAML Parse

Command:

```bash
python3 - <<'PY'
import yaml
path = "src/luggage_gazebo/config/sim_world.profile.yaml"
data = yaml.safe_load(open(path, encoding="utf-8"))
assert isinstance(data, dict)
assert data["runtime"]["gui"] is False
assert data["features"]["use_semantic"] is True
assert data["spawner"]["sequence_ids"] == ["carryon", "standard", "large"]
print("profile ok")
PY
```

Expected:

```text
profile ok
```

Acceptance:

- Pass if the YAML loads as a mapping and the key sample matches the reference
  profile.
- Fail if YAML syntax is invalid or expected groups are missing.

## Test 3: Launch Arguments Are Exposed

Command:

```bash
ros2 launch luggage_gazebo sim_world.launch.py --show-args
```

Expected:

- Output lists `profile_config`.
- Existing launch arguments are still listed, including:
  - `scene_tf_config`
  - `gui`
  - `use_rviz`
  - `use_semantic`
  - `use_motion`
  - `use_vacuum`
  - `use_cargo_map`
  - `use_packing`
  - `visual_kind`
  - `size_mode`
  - `sequence_ids`
  - `observe_pose_name`

Acceptance:

- Pass if `--show-args` exits with code `0` and the argument surface is
  backwards compatible.
- Fail if launch loading fails or a previously supported argument disappears.

## Test 4: Built-In Fallback Without Profile

Command:

```bash
python3 - <<'PY'
import importlib.util
from launch import LaunchContext

path = "src/luggage_gazebo/launch/sim_world.launch.py"
spec = importlib.util.spec_from_file_location("sim_world_launch", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

ctx = LaunchContext()
ctx.launch_configurations["profile_config"] = ""
for name in mod._default_launch_values():
    ctx.launch_configurations[name] = mod._PROFILE_SENTINEL

cfg = mod._resolved_launch_config(ctx)
assert cfg["gui"] == "true"
assert cfg["use_rviz"] == "true"
assert cfg["use_semantic"] == "false"
assert cfg["visual_kind"] == "box"
assert cfg["observe_pose_name"] == "observe"
print("fallback ok")
PY
```

Expected:

```text
fallback ok
```

Acceptance:

- Pass if omitted profile values resolve to the same launch defaults as before
  the profile change.
- Fail if a default silently changes.

## Test 5: Profile Values Override Built-In Fallback

Command:

```bash
python3 - <<'PY'
import importlib.util
from launch import LaunchContext

path = "src/luggage_gazebo/launch/sim_world.launch.py"
spec = importlib.util.spec_from_file_location("sim_world_launch", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

ctx = LaunchContext()
ctx.launch_configurations["profile_config"] = (
    "src/luggage_gazebo/config/sim_world.profile.yaml"
)
for name in mod._default_launch_values():
    ctx.launch_configurations[name] = mod._PROFILE_SENTINEL

cfg = mod._resolved_launch_config(ctx)
assert cfg["gui"] == "false"
assert cfg["use_rviz"] == "false"
assert cfg["use_semantic"] == "true"
assert cfg["use_motion"] == "true"
assert cfg["use_vacuum"] == "true"
assert cfg["use_cargo_map"] == "true"
assert cfg["use_packing"] == "true"
assert cfg["visual_kind"] == "mesh"
assert cfg["sequence_ids"] == "carryon,standard,large"
assert cfg["observe_pose_name"] == "pickup_observe"
print("profile override ok")
PY
```

Expected:

```text
profile override ok
```

Acceptance:

- Pass if values edited in YAML replace built-in fallback values.
- Fail if the launch file ignores grouped profile values.

## Test 6: CLI Overrides Profile

Command:

```bash
python3 - <<'PY'
import importlib.util
from launch import LaunchContext

path = "src/luggage_gazebo/launch/sim_world.launch.py"
spec = importlib.util.spec_from_file_location("sim_world_launch", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

ctx = LaunchContext()
ctx.launch_configurations["profile_config"] = (
    "src/luggage_gazebo/config/sim_world.profile.yaml"
)
for name in mod._default_launch_values():
    ctx.launch_configurations[name] = mod._PROFILE_SENTINEL

ctx.launch_configurations["use_rviz"] = "true"
ctx.launch_configurations["sequence_ids"] = "carryon"

cfg = mod._resolved_launch_config(ctx)
assert cfg["use_rviz"] == "true"
assert cfg["sequence_ids"] == "carryon"
assert cfg["use_semantic"] == "true"
print("cli override ok")
PY
```

Expected:

```text
cli override ok
```

Acceptance:

- Pass if explicit CLI values override YAML values.
- Pass only if unrelated YAML values still apply.
- Fail if CLI values are ignored or if CLI override disables the rest of the
  profile unexpectedly.

## Test 7: Missing Profile Fails Clearly

Command:

```bash
python3 - <<'PY'
import importlib.util
from launch import LaunchContext

path = "src/luggage_gazebo/launch/sim_world.launch.py"
spec = importlib.util.spec_from_file_location("sim_world_launch", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

ctx = LaunchContext()
ctx.launch_configurations["profile_config"] = "/tmp/missing_sim_world_profile.yaml"
for name in mod._default_launch_values():
    ctx.launch_configurations[name] = mod._PROFILE_SENTINEL

try:
    mod._resolved_launch_config(ctx)
except RuntimeError as exc:
    assert "profile_config not found" in str(exc)
    print("missing profile rejected")
else:
    raise AssertionError("missing profile was accepted")
PY
```

Expected:

```text
missing profile rejected
```

Acceptance:

- Pass if a missing file raises a clear `RuntimeError`.
- Fail if launch silently falls back when the user explicitly selected a
  profile path.

## Test 8: Unknown YAML Keys Warn And Are Ignored

Command:

```bash
tmp_profile=/tmp/sim_world_unknown_key.profile.yaml
cp src/luggage_gazebo/config/sim_world.profile.yaml "$tmp_profile"
cat >> "$tmp_profile" <<'YAML'

debug:
  typo_parameter_should_warn: true
YAML

python3 - <<'PY'
import importlib.util
from launch import LaunchContext

path = "src/luggage_gazebo/launch/sim_world.launch.py"
spec = importlib.util.spec_from_file_location("sim_world_launch", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

ctx = LaunchContext()
ctx.launch_configurations["profile_config"] = "/tmp/sim_world_unknown_key.profile.yaml"
for name in mod._default_launch_values():
    ctx.launch_configurations[name] = mod._PROFILE_SENTINEL

cfg = mod._resolved_launch_config(ctx)
assert cfg["use_semantic"] == "true"
print("unknown key test ok")
PY
```

Expected:

- Output contains:

```text
WARN: sim_world profile ignored unknown keys: typo_parameter_should_warn
unknown key test ok
```

Acceptance:

- Pass if unknown YAML keys do not break launch and produce a visible warning.
- Fail if unknown keys silently change behavior.

## Test 9: Optional Headless Launch Smoke

This test starts Gazebo and should be run only on a machine with the required
GPU/rendering setup.

Command:

```bash
ros2 launch luggage_gazebo sim_world.launch.py \
  profile_config:=/home/adamliao/work/elfin_humble_ws/src/luggage_gazebo/config/sim_world.profile.yaml \
  gui:=false use_rviz:=false
```

Expected:

- GPU hard gate passes.
- Gazebo server starts with `airport_loading.sdf`.
- Camera and world service bridges start.
- `pickup_box_spawner_node.py`, `luggage_detector_node.py`,
  `semantic_segmenter_node.py`, `semantic_point_filter_node.py`,
  `scene_manager_node.py`, `waypoint_generator_node.py`,
  `motion_planner_node.py`, `cargo_volume_mapper_node.py`,
  `placement_planner_node.py`, and `vacuum_controller_node.py` are launched
  according to the reference profile.

Recommended quick probes in another shell:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 service list | grep -E 'detect_luggage|spawn_next_box|compute_placement|vacuum_controller'
```

Acceptance:

- Pass if the launch graph starts without Python launch exceptions and the
  expected nodes/services are visible.
- Fail if profile-controlled nodes do not match the YAML feature flags.

## Overall Acceptance Standard

The launch profile feature is accepted when:

- Tests 1 through 7 pass.
- Test 8 produces a visible warning for unknown YAML keys.
- `--show-args` confirms the old launch argument surface is still present.
- A profile user can edit `sim_world.profile.yaml` to change the launch graph
  without editing Python.
- A CLI user can still override any listed launch argument for one run.
- The optional smoke test passes on a valid Gazebo/GPU machine, or is recorded
  as not run with the environment reason.
