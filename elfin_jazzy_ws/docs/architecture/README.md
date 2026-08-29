# Architecture

Normative architecture for this workspace. Unlike `plans/` (what we intend to
build) and `status/` (what we measured), the documents here define **how code
must be structured**. A change that contradicts them is a defect, not a style
preference.

- [Perception module architecture](perception_architecture.md): algorithm
  classes versus ROS nodes, state ownership, output contracts.
- [Sensor data pipeline](sensor_data_pipeline.md): per-frame data structures,
  multi-rate alignment, buffering, the `SyncedObservation` snapshot.
- [Motion compensation](motion_compensation.md): eye-in-hand 6DOF deskew for
  depth and Livox Mid-360 point clouds.

## Enforcement

Short, machine-facing versions of these rules live in `.cursor/rules/` and are
injected into every agent session:

| Rule | Scope | Enforces |
|---|---|---|
| `ros2-node-structure.mdc` | `src/**/*.py` | [perception_architecture.md](perception_architecture.md) |
| `perception-data-pipeline.mdc` | `luggage_perception`, `luggage_planning` | [sensor_data_pipeline.md](sensor_data_pipeline.md) |
| `sensor-frames-and-timing.mdc` | perception, description, gazebo | [motion_compensation.md](motion_compensation.md) |

The rule files carry only the hard "must / must not" lines. Rationale, tables,
sizing numbers, and vendor protocol details stay here. When the two disagree,
these documents win and the rule file is the bug.

## Changing these documents

1. Update the document first, in the same change that updates code.
2. Update the matching `.cursor/rules/*.mdc` if a must-rule changed.
3. Record measured evidence in `status/`, not here. This directory states
   intent and contracts; it does not log test runs.

## Known deviations

Existing code predates these documents. The deviations below are **tracked
exceptions**, not precedent. New code must not copy them, and any change that
touches these files should move them toward compliance.

| Deviation | Location | Target |
|---|---|---|
| `SemanticSegmenter.segment()` takes no stamp, returns no frame, and exposes `instance_map` as an internal reference | [semantic_segmenter.py](../../src/luggage_perception/luggage_perception/semantic_segmenter.py) | `update(rgb, stamp, frame_id)` + `copy_output()` |
| Mid-360 is a reserved empty link only: no sensor, no driver, no frames in sim | [eef_sensor_mount.urdf.xacro](../../src/luggage_description/urdf/eef_sensor_mount.urdf.xacro) `mid360_mount_frame` | add sensor + driver per [motion_compensation.md](motion_compensation.md) |
| Algorithm class imports `rospy` and implements a latest-TF fallback, both forbidden | [robot_self_point_filter.py](../../src/luggage_perception/luggage_perception/robot_self_point_filter.py) `_lookup_transform` | node resolves transforms at the data stamp and passes them in; delete `allow_latest_tf_fallback` |
| Algorithm class builds `Marker` / `ColorRGBA` via deferred imports | [cargo_volume_mapper.py](../../src/luggage_perception/luggage_perception/cargo_volume_mapper.py) | return geometry; assemble messages in the node |
| Planning utilities build `geometry_msgs` types via deferred imports | `vacuum_attach_utils.py`, `container_aim_utils.py` in `luggage_planning` | return tuples; convert in the node |
| Unported `rospy` files sit directly in `scripts/`, not only in `scripts/ros1_reference/` | `luggage_planning` (16), `luggage_packing` (2), `luggage_bringup` (COLCON_IGNORE) | reference only; identify by absence from `install(PROGRAMS ...)` |
