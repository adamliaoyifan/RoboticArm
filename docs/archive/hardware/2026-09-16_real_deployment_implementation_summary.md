# 2026-09-16 Real Deployment Implementation Summary

This work was implemented in the local workspace at
`/home/adamliao/work/elfin_humble_ws`. It was not pushed or copied to an SSH
host, Orin, or Lenovo deployment machine.

## Implemented

- Split `hardware_pick.launch.py` into role-level switches:
  - `start_perception`: sensor preprocessor, semantic segmenter, semantic point
    filter, and luggage detector.
  - `start_planning`: scene manager, waypoint generator, motion planner, vacuum
    controller, and MoveIt when `use_moveit:=true`.
- Preserved the existing low-level ownership switches:
  - `start_d555`
  - `start_executor`
  - `start_scene`
  - `use_moveit`
- Added the real-deployment runbook:
  - Current ThinkPad all-in-one launch.
  - Future Orin perception / Lenovo planning split.
  - Shared DDS assumption: `ROS_DOMAIN_ID=7`.
  - One owner each for D555, CPS executor, and scene TF.
- Documented pickup XY strategy:
  - No `DetectedLuggage.msg` schema change.
  - Suction contact XY remains in both `DetectedLuggage.pose.position.x/y` and
    `DetectedLuggage.top_surface_pose.position.x/y`.
  - Manual RGB labels are the benchmark ground truth.
- Added a ROS-free offline pickup XY benchmark scorer:
  - Scores candidate world-XY strategies against manual labels.
  - Reports median error, P95 error, improvement vs current PCA, and outliers.
  - Encodes the acceptance gate: median <= 30 mm, P95 <= 60 mm, improvement
    over PCA on at least 80% of comparable frames.
- Added guarded `servo_j` support in the Huayan executor:
  - Production default remains `execution_backend:=waypoint`.
  - `servo_j` is opt-in only.
  - Uses `StartServo` once followed by `PushServoJ` absolute joint-degree
    targets on a fixed time grid.
  - Does not use `servo_esj`.
  - Does not use `MovePathJOL`.
  - Cancel path calls the existing stop path.

## Files Changed

- `src/luggage_planning/launch/hardware_pick.launch.py`
- `src/luggage_planning/test/test_hardware_pick_launch.py`
- `docs/hardware/real_deployment_pickup_control.md`
- `src/luggage_perception/luggage_perception/eval/pickup_xy_benchmark.py`
- `src/luggage_perception/test/eval/test_pickup_xy_benchmark.py`
- `deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/huayan_interface.py`
- `deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/trajectory_executor_node.py`
- `deployment_ws/src/elfin_trajectory_executor/launch/jazzy_real.launch.py`
- `deployment_ws/src/elfin_trajectory_executor/test/test_hardware_safety.py`

## Verification

Passed locally:

```bash
python3 -m pytest -q src/luggage_planning/test/test_hardware_pick_launch.py
PYTHONPATH=src/luggage_perception python3 -m pytest -q src/luggage_perception/test/eval/test_pickup_xy_benchmark.py
PYTHONPATH=deployment_ws/src/elfin_trajectory_executor python3 -m pytest -q deployment_ws/src/elfin_trajectory_executor/test/test_hardware_safety.py
python3 -m py_compile <edited Python files>
```

## Not Done

- No SSH deployment was performed.
- No remote build was performed.
- No Orin-specific script was created.
- No live hardware ServoJ gate was run.
- No pickup strategy was enabled in production based on benchmark results.
- No ROS message schema was changed.

## Important TODO

- Sync these local changes to the SSH/deployment machine and rebuild both the
  main workspace and `deployment_ws`.
- Run a remote smoke test for the default production path:
  `hardware_pick.launch.py execution_backend:=waypoint`.
- Validate the split launch shape on the real network:
  - Orin: `start_perception:=true`, `start_planning:=false`,
    `start_executor:=false`, `start_scene:=false`.
  - Lenovo: `start_perception:=false`, `start_planning:=true`,
    `start_executor:=true`, `start_scene:=true`.
- Confirm there is exactly one owner for D555, CPS executor, and scene TF during
  split deployment.
- Collect and label real RGB frames from `~/robotarm_bags` plus the kept
  floor-pick bag.
- Generate candidate pickup XY outputs for `pca_center`,
  `yolo_bbox_center_top_plane`, `lid_inlier_center`, and
  `robust_blended_center`.
- Run the offline pickup benchmark and preserve outlier/failure artifacts under
  `docs/status/evidence/`.
- Only change the live pickup XY strategy after the benchmark passes median <=
  30 mm, P95 <= 60 mm, and >= 80% improvement over current PCA.
- Run ServoJ hardware gates S0-S3 before any dry pick with
  `execution_backend:=servo_j`.
- Keep production pick on `execution_backend:=waypoint` until ServoJ gates pass
  and a separate hardware acceptance note records the result.

## Deployment Next Step

To use this on the SSH/deployment machine, sync or commit these changes, then
build and test on that host. The default production launch should still use:

```bash
ros2 launch luggage_planning hardware_pick.launch.py execution_backend:=waypoint
```

Only use `execution_backend:=servo_j` for the documented hardware qualification
gates S0-S3.
