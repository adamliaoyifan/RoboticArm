# 2026-09-04 12:16 — Platform-free height E0-E5 implementation complete

- role: eng
- agent: claude
- model: glm-5.2
- cli: claude-code
- status: open
- checkpoint: E0-E5
- revision: 0674f84 + working tree (uncommitted)
- thread: `docs/agents/discuss/2026-09-04_1143_platform-free-height-eng.md`

## Summary

Implemented the full engineering TODO
(`docs/plans/platform_free_height_eng_todo.md`) in dependency order.
The online detector no longer reads `pickup_source.z`, spawner state, or
`GetCurrentBox` geometry. Top surface, XY, yaw, width, and depth come from
the semantic cargo cloud; height/center-Z are valid (FULL_3D) only when the
local support plane was fitted from the same acquisition stamp with settled
geometry. GT fallback and GT comparison were removed from the online node
(E5); eval drivers keep their own spawner reads.

## Changed files

| File | Change |
|---|---|
| `src/luggage_msgs/msg/DetectedLuggage.msg` | E0: `header`, `top_surface_pose`, `top_surface_valid/confidence`, `height_valid/confidence`, `height_source` enum (UNAVAILABLE/MEASURED_SUPPORT/CONFIGURED_SUPPORT/CATALOG_PRIOR) |
| `src/luggage_msgs/msg/DetectionFrame.msg` | E0: `support_valid/reason/z/confidence/residual/side_coverage/inliers`, `geometry_level` (TOP_ONLY/FULL_3D) |
| `src/luggage_perception/luggage_perception/top_support_estimator.py` | E1 (new, zero ROS): `TopSupportConfig`, `TopSurfaceEstimate`, `SupportPlaneEstimate`, `BoxGeometryEstimate`, `estimate_top_surface`, `estimate_local_support`, `compose_box_geometry`, `SupportStabilityFilter` (update/copy_output); failure-code constants; height = top_z − support_z, center_z = (top_z + support_z)/2 |
| `src/luggage_perception/luggage_perception/platform_free_pipeline.py` | E2 (new, zero ROS): `PlatformFreeDetector` gate — hold_track never fuses with fresh raw depth (`DETECT_SUPPORT_STAMP_MISMATCH`), geometry-not-settled skips support, support modes `auto/configured/auto_then_configured/top_only`, catalog width/depth match fills a prior height with `height_valid=false`; per-frame rejected supports never enter the Z-stability window |
| `src/luggage_perception/scripts/luggage_detector_node.py` | E2+E5: exact-stamp raw-depth buffer (`OrderedDict` keyed by `(sec,nanosec)`, bounded) decoded+TF'd to world; detector path = platform-free pipeline; publishes new `DetectionFrame` support fields + `DetectedLuggage` validity fields; GT fallback / `evaluation_compare_gt` / `GetCurrentBox` client removed; E3 params declared |
| `src/luggage_perception/test/test_top_support_estimator.py` | Gate-1 synthetic suite, 21 tests: 3 sizes × 3 yaw × 3 support heights (0.62/0.90/1.10, never 0.86), 3 mm noise, 10% outliers, lower floor exclusion, 1/3/4-side occlusion, sparse/non-finite, determinism, compose semantics, stability filter |
| `src/luggage_planning/luggage_planning/waypoint_generator.py` | E4: `pick_contact_top_z()` — pick Z = `top_surface_pose.z` when valid, else measured `center + height/2`; prior-only height raises `DETECT_FULL_GEOMETRY_REQUIRED` |
| `src/luggage_planning/test/test_waypoint_generator.py` | contract tests: top-surface priority, prior-only rejection |
| `src/luggage_packing/scripts/placement_planner_node.py` | E4: `ComputePlacement` rejects `height_valid=false` with `DETECT_FULL_GEOMETRY_REQUIRED` |
| `src/luggage_gazebo/scripts/pickup_box_spawner_node.py` | E4: GT box carries full validity (eval-side truth stands in for a perfect sensor; online nodes never read it) |
| `src/luggage_perception/luggage_perception/detect_overlay.py` | E4: overlay label tags `[TOP-ONLY]` when `height_valid=false` |
| `src/luggage_gazebo/launch/sim_world.launch.py` | E3: `support_mode=auto`, `platform_z=""` (omitted is valid); removed `allow_gt_fallback`/`evaluation_compare_gt` |

## Verification already run (engineering environment)

```bash
cd ~/work/elfin_humble_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select luggage_msgs luggage_perception \
  luggage_planning luggage_packing luggage_gazebo
cd src/luggage_perception && python3 -m pytest test/ -q   # 367 passed
cd ../luggage_planning && python3 -m pytest test/ -q      # 215 passed
cd ../luggage_packing && python3 -m pytest test/ -q       #  75 passed
```

Detector/spawner/placement nodes import cleanly (module exec without spin).
Note: `test_cargo_map_confidence.py` fails with `ModuleNotFoundError:
luggage_packing` only when run unsourced from the perception directory;
with the workspace sourced it passes.

## Design decisions worth review

1. **Stability window before FULL_3D.** `SupportStabilityFilter`
   (default window 5, max Z spread 15 mm) returns
   `DETECT_SUPPORT_UNSTABLE` until the window fills (~1.2 s at 4 Hz),
   so the first frames after a spawn are TOP_ONLY by design.
2. **Raw-path support source.** With `use_semantic=false` the cargo cloud
   IS the depth-topic cloud; it is passed as its own raw source when the
   duplicate subscription delivers late (avoids a systematic
   stamp-mismatch artifact from callback ordering).
3. **geometry_ok gating.** Support fitting requires
   `flags.geometry_ok` from the preprocessor status. When no status
   payload was ever received (no preprocessor in the deployment), the
   gate is treated as settled — absence of motion evidence is not motion.
4. **GT box validity.** The spawner sets `height_valid=true` +
   `HEIGHT_SOURCE_MEASURED_SUPPORT` on its GT box: eval-only truth standing
   in for a perfect sensor. Flag if you want a dedicated GT source enum.
5. **Catalog prior.** Matched on width/depth only (height is the unknown);
   tolerance is the existing `catalog_match_tolerance` (0.08).

## Blockers

None for E0-E5 scope. Gate 4 (Gazebo 30-trial online/eval separation) and
Gate 6 (performance/lifecycle) need a running simulation stack and belong
to the test role per the companion test plan.

## Commands for the test role

Gates 0-6 per `docs/plans/platform_free_height_test_plan.md`; evidence
root `docs/status/evidence/platform_free_height/<run_id>/`. Suggested
launch for Gates 2-4:

```bash
cd ~/work/elfin_humble_ws && source install/setup.bash
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true visual_kind:=mesh \
  sequence_ids:=standard observe_pose_name:=pickup_observe
```

## Result

- pass (engineering scope): E0-E5 implemented, 657 unit tests green,
  no online node reads simulation truth for geometry.

## Test Handoff

- Notified `test` via discuss thread
  `2026-09-04_1143_platform-free-height-eng.md`: E0-E5 ready, start
  Gates 0-6 per the companion test plan.

## Pointers

- `docs/plans/platform_free_height_eng_todo.md`
- `docs/plans/platform_free_height_test_plan.md`
- `docs/agents/discuss/2026-09-04_1143_platform-free-height-eng.md`

## Open

- None.
