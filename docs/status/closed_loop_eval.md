# Closed-loop pick / retreat eval

Date: 2026-08-28. Workspace has no git HEAD. `ROS_DOMAIN_ID=0`.

Loop per trial: spawn catalog suitcase (carryon / standard / large, mesh loafbrr or vintage) → `GoToRobotPose(pickup_observe)` → wait live `geometry_ok` + new cargo cloud → `DetectLuggage` vs `GetCurrentBox` → if the detection gate passes, `BuildMotionSequence(pick)` and four `PlanMotion` segments → check `suction_contact_frame` ΔZ on `pick_retreat` → `ClearCurrentBox`.

Config: `use_semantic:=true`, `visual_kind:=mesh`, `sequence_ids:=carryon,standard,large`, `observe_pose_name:=pickup_observe`, backend **`bbox_fill:yolov8s-world.pt`**. Detection tols unchanged from todo 2 (`xy=0.03`, `z=0.02`, `size=0.05`, `yaw=0.15`, `iou=0.60`). Retreat expected ΔZ `0.35 m`, pass band `±0.08 m`.

Raw jsonl / summary: [docs/status/evidence/pick_eval/](evidence/pick_eval/). An earlier N=20 that died on controller wrap + a settle crash is archived under [pick_eval_run1_control_failed](evidence/pick_eval_run1_control_failed/).

## Rates (independent denominators)

Do not mix these. A detect miss never enters the plan rate; a plan abort never enters the retreat rate.

| Metric | Rate | n |
|---|---|---|
| Detect pass (`DetectionAccuracy.ok`) | **57.9%** | 11 / 19 compared |
| Plan pass (4/4 segments, among detect-pass) | **100%** | 11 / 11 |
| Retreat height (among plan-pass) | **100%** | 11 / 11 |
| End-to-end (retreat-ok / N) | **55%** | 11 / 20 |

One trial (`DETECT_ESTIMATION_FAILED`) is not a compare, so the detect denominator is 19 not 20.

## Geometric pick accuracy (no vacuum)

There is no suction / ACM. “Pick” here means: the four segments executed, and the tool lifted ~0.35 m after attach.

After attach, `suction_contact_frame` vs the **measured** box is millimetres of tracking (the motion was planned to that box). True pick accuracy vs spawn GT is essentially the detection error.

| | n | mean | std | p50 | p95 |
|---|---|---|---|---|---|
| attach XY vs GT (m) | 11 | 0.0094 | 0.0096 | 0.0052 | 0.0287 |
| attach \|Z\| vs GT (m) | 11 | 0.0076 | 0.0046 | 0.0053 | 0.0138 |
| attach XY vs measured (m) | 11 | ~0 | ~0 | ~0 | ~0 |
| retreat ΔZ (m) | 11 | 0.3499 | ~0 | 0.3499 | 0.3499 |

Stability: among the 11 executed picks, retreat ΔZ std is `1.6e-5 m`. Attach-vs-GT XY std is `9.6 mm` (two large/loafbrr trials sit at ~28 mm, still inside the 30 mm gate).

## Where it fails

All motion failures in this run are **upstream of planning**. Fail codes:

| Code | n | Notes |
|---|---|---|
| `DETECT_GATE:xy` | 6 | Five of six are **large + vintage**. Measured XY ~4 cm vs GT, just over `tol_xy=0.03`. |
| `DETECT_GATE:size` | 1 | standard / loafbrr |
| `DETECT_GATE:iou` | 1 | carryon / vintage |
| `DETECT_ESTIMATION_FAILED` | 1 | standard / loafbrr |

Per catalog detect-ok: carryon 5/7, standard 4/6, large **2/7**. Per visual: loafbrr 7/10, vintage 4/10.

Detection tols were **not** loosened to inflate pick rate.

## Fixes that made 4/4 plan possible

This stack did not pick until these were in:

1. `move_group` loads `S20_with_camera.srdf` rewritten to `name="S20"` so suction/camera are not FCL-colliding with `elfin_link6`.
2. `motion_planner` no longer `wait_for_server(60s)` in `__init__` (that raced the executor).
3. `SettleTracker` is fed `{joint: value}` dicts, not lists (`'list' object has no attribute 'items'` aborted every GoTo after the first plan).
4. `pre_grasp` uses current-seeded IK + nearest ±2π wrap. Bare OMPL pose sampling commanded joint1 ~4 rad the long way; the controller aborted `GOAL_TOLERANCE_VIOLATED` (`MoveGroup error_code=-4` = `CONTROL_FAILED`).
5. Cartesian `GetCartesianPath` now returns the Humble `RobotTrajectory` (`solution.joint_trajectory`) into `/execute_trajectory`. The old helper expected a ROS1-shaped `joint_state` and crashed `'str' object has no attribute 'solution'`.
6. Eval waits for a **new** cargo cloud stamp after spawn so DetectLuggage cannot reuse the previous suitcase.

Velocity/accel scaling on MoveGroup is 0.3 (same as the ROS 1 planner).

## Known deviations

- No vacuum: success is geometric, not grasp retention. The suitcase stays on the platform; `ClearCurrentBox` deletes it.
- MoveIt has no suitcase collision object; attach can physically kiss the mesh in Gazebo.
- `keep_camera_down` / `lock_wrist` are not implemented.
- Fast-DDS ghost node names linger after SIGTERM; the driver uniqueness check is on **process** counts, not `ros2 node list`.
- `ROS_DOMAIN_ID=0` (interactive sim), not the plan’s isolated 42.

## Trials

See [evidence/pick_eval/closed_loop_eval.md](evidence/pick_eval/closed_loop_eval.md) for the 20-row table and [evidence/pick_eval/trials.jsonl](evidence/pick_eval/trials.jsonl) for raw records.
