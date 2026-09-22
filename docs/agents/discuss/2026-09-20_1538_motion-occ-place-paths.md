# 2026-09-20 -- MOTION-OCC occupancy-aware place paths generation 2

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.6
- kind: subtask
- parent: REAL-SCENARIO-20260920
- subtask: MOTION-OCC
- depends_on: none
- revision: dbc751f8
- generation: 2
- plan_revision: dbc751f8
- dispatch_ready: yes

## Post -- reviews/cursor -- 2026-09-20 15:38 -- cursor/grok-4.6

Generation 2: top-K ComputePlacement candidates, occupancy inject + FK re-sweep, selector over (slot, path) with C_place/C_dim/C_safe/C_eff. dispatch_ready stays no until the plan file is committed as plan_revision. OCC-1 coverage 0.40 stays a separate gate.

## Pointers

- `docs/plans/motion_occ_occupancy_aware_place_paths.md`
- `docs/architecture/placement.md`

## Open

- Implement occupancy-aware cartesian/OMPL plus multi-slot trajectory selector (RS-27). A0 pytest then exclusive Gazebo A1 n=2. Do not close OCC-1.

## Claim -- eng/cursor -- 2026-09-20 15:40 -- cursor/grok-4.6

- started_at: 2026-09-20T15:40:00+08:00
- claimed_generation: 2
- claimed_plan_revision: dbc751f8
- claimed_dependencies: none

User-directed execution with a dirty plan file. `agent_start.sh` refused a generation/plan_revision match; this Claim records owner intent so A0/A1 can proceed.

## Reply -- eng/cursor -- 2026-09-20 20:33 -- cursor/grok-4.6

A1 `2026-09-20_1603` was not stuck in place execution. Box 0 reached `HOME` (7/7 segments). `GOTO_FAILED` is `_home_arm` / `place_exit` after commit.

Root cause (from that dump, T2 was incomplete): after `AddPlacedBox`, MoveIt world objects are `airport_container_real`, `placed_0_0_0`, `robot_pedestal` — occupancy `cargo_occ_*` boxes were already cleared. `_exit_to_portal` rebuilt a dummy-slot **transit** (still inside the suitcase) as cartesian `place_exit`. That path is collision-checked against the newly committed box. Trial 0: cartesian accepted, `ExecuteTrajectory -4` (`CONTROL_FAILED`). Trial 1: start likely in collision, cartesian `0.000` then IK `-2` in ~0.1 s. `place_exit` was not in `segments.jsonl`.

Fix in progress: dump planned traj + scene geometry + joint ring into `replay/` on every trial; point `place_exit` at the opening portal. New A1: `docs/status/evidence/motion_occ/2026-09-20_2032/`.

A1 `2026-09-20_2032`: `place_exit` now succeeds (cartesian fraction 1.0). Trial 0 still `COVERAGE_LOW` after a good place. Trial 1 selector ran occupancy-aware (`raised_0.10`, 5 slots / 20 rows) then `PLACE_PLAN_transit` MoveGroup `99999` with `occupancy_checked=true`, 12 occupancy objects, attached `pickup_box`, world `placed_0_0_0`. Replay bundle: `dumps/trial_01_PLACE_PLAN_transit/replay/` (`replay_possible: true`). Capture-health complete; MOTION-OCC gate still fails because the second box did not place.

## Reply -- eng/cursor -- 2026-09-21 15:25 -- cursor/grok-4.6

Trial 1 transit did not execute. MoveIt `FAILURE` 99999 after collision-aware IK plus RRTConnect pose-constraint fallback; planned traj has 0 points.

Hanging payload at the transit goal is 2.8 mm above `placed_0_0_0` with 0.51 m × 0.21 m XY overlap. Occupancy column AABBs miss that goal. 10 mm pad makes the payload/placed pair collide. Scene plot: `docs/status/evidence/motion_occ/2026-09-20_2032/dumps/trial_01_PLACE_PLAN_transit/replay/scene_xy_xz.png`.

## Reply -- eng/cursor -- 2026-09-21 16:38 -- cursor/grok-4.6

User-directed n=3 after dynamic `clear_top` lift. Unscored `2026-09-21_1632`: `wait_graph` missed `move_group` (gz_ros2_control URDF timeout; joint_state_broadcaster died). Scored `2026-09-21_1638`: `scripts/motion_occ_place_path_run.sh --n 3 --sequence-ids carryon,standard,large`. `place_ok=1/3`. Box 0 HOME + `COVERAGE_LOW`. Box 1 selector `clear_top` dest z=1.107 (payload bottom 0.83 m). Transit planned (IK joint goal, 105 pts, 10.3 s) then `ExecuteTrajectory -4`; retry `MoveGroup -2`. Driver stopped; box 2 not attempted. Clock publishers=1, `capture_complete=true`, teardown residual 0.

## Reply -- eng/cursor -- 2026-09-21 16:50 -- cursor/grok-4.6

`2026-09-21_1638` box 1 `CONTROL_FAILED` is not a MoveIt execution-monitor collision abort and not the old 2.8 mm payload-on-`placed_0_0_0` hover.

First execute (`0021_transit`): plan fraction 1.0, 105 pts, 10.34 s, start 1789979870.799. At 1789979886.172 the sim JTC aborted: `State tolerances failed for joint 1` (controller index for `elfin_joint2`) `Position Error: 0.785571` vs `goal: 0.30`, `goal_time_tolerance exceeding by 5.009210 seconds`. MoveIt maps that to `GOAL_TOLERANCE_VIOLATED` / `CONTROL_FAILED` (-4). Planned end vs abort joints: j1/j4/j5/j6 at goal; j2 stuck at -1.866 vs commanded -1.081 (error 0.786 rad). Path is a configuration-space flip (j5 1.57→4.73, j3 sign change). Abort suction (0.747, -0.320, 1.924) is still at the opening, not the slot (1.08, -0.56, 1.107). Payload bottom at that Z cannot be the packed-box contact.

Retry (`0022_transit`) start is already in collision: `airport_container_real` vs `suction_panel`, then vs attached `pickup_box`. `Fix Start State In Collision` then the 86-pt path is invalid at indices 0–14 → `INVALID_MOTION_PLAN` (-2).

Dump cannot timestamp first Gazebo contact (`joint_ring` is the frozen abort pose). Remaining split: j2 lag from open-loop sim servo vs physical jam producing the lag. Both end with the same mixed-joint FK in the container mesh. MOTION-OCC stays open.

## Reply -- eng/cursor -- 2026-09-21 16:54 -- cursor/grok-4.6

j2 missed because the sim position servo lagged a reverse at the scaled joint-velocity cap, not because the panel opened.

Lift (`stage_mid`/`stage_late`) is cartesian, `keep_tool_down`, j5 held at 1.565, quat `[0,1,0,0]`, vacuum attached, tilt 0. `suction_panel_joint` is fixed. Transit IK then locked a 180 deg wrist-flip goal (j5 1.57→4.73, tool-down yaw 0 `[1,0,0,0]`). j2 commanded -1.275 → -2.375 (t=7.0) → -1.081 (t=10.34). Recovery mean 0.387 rad/s, peak 0.471 = `1.57*0.3`. Abort: j1/j3/j4/j5/j6 at goal, j2 at -1.866 (0.786 rad short). j3 also peaked at 0.471 and arrived; j2 is the axis with URDF damping/friction 40, gravity, and the reverse. Open-loop JTC + P=0.8 + `goal_time` 5 s produces `GOAL_TOLERANCE_VIOLATED`. Same freeze is why `place_only_profile.yaml` dropped scaling to 0.15; this run used default 0.3. Box 0 transit succeeded at 18.6 s with j2 peak only 0.124 rad/s (no reverse). Gazebo strips panel ODE collision, so the panel cannot jam the suitcase in physics. FCL `suction_panel` vs container is the mixed abort FK. Attached-box ODE contact during the swing is still untimestamped. MOTION-OCC stays open.

## Reply -- eng/cursor -- 2026-09-21 17:04 -- cursor/grok-4.6

Place-leave is descend → `VacuumCommand(false)` → cartesian `retreat` at the same slot yaw. Box 1 never got there (`ABORT_CARRYING` on inbound transit, vacuum still on). URDF panel has no open joint; "open" is vacuum detach. Post-place lift does not need a wrist flip. The j5 1.57→4.73 path is inbound: staging tool-down at pick yaw π (`[0,1,0,0]`) vs transit/slot yaw 0 (`[1,0,0,0]`). For a rectangular box those yaws are equivalent; IK still locked a flipped-wrist joint goal. MOTION-OCC stays open.

## Reply -- eng/cursor -- 2026-09-21 17:08 -- cursor/grok-4.6

Dump-reconstructed 3D HTML + process video (no rosbag on these runs):

- `docs/status/evidence/motion_occ/2026-09-20_2032/replay_viz/` (220 frames, 18.3 s). Trial 1 transit never moved; ghost is selector waypoints onto the 2.8 mm hover goal.
- `docs/status/evidence/motion_occ/2026-09-21_1638/replay_viz/` (192 frames, 16.0 s). Trial 1 transit executed to suction (0.747, -0.320, 1.924) then `PLACE_PLAN_transit`; dashed goal stays above `placed_0_0_0`.

Open `index.html`. Rebuild: `PYTHONPATH=src/luggage_planning python3 scripts/render_motion_occ_3d.py --run DIR`.

## Reply -- eng/cursor -- 2026-09-21 17:49 -- cursor/grok-4.6

First replay drew a cuboid from `inner_size`. That is the forbidden legacy AABB. TCIG usable space is the scene_tf 7-face hull: cuboid minus the +Y triangular prism, ±X ends pentagons, purple rectangle is the door aperture on the -X pentagon (not the full face). Regenerated both `replay_viz/` trees (15 hull edges). MoveIt collision is still the STL; this outline is the packing kernel, not a cube.

## Reply -- eng/cursor -- 2026-09-21 18:46 -- cursor/grok-4.6

User-directed stop then n=3 restart. `scripts/stop_sim.sh` found no launch. Rerun `scripts/motion_occ_place_path_run.sh --n 3 --sequence-ids carryon,standard,large` wrote `docs/status/evidence/motion_occ/2026-09-21_1838/`. `place_ok=1/3`, fail codes all `GOTO_FAILED`. Clock publishers=1. Teardown residual 0 at `2026-09-21T18:41:51+08:00`. Sim is down.

Box 0 place itself succeeded (7/7 segments): cartesian lift held `j5≈1.57`; `transit` IK joint goal flipped `j5` 1.570→4.707 (`Δ=3.14`, 82 pts, 8.06 s) and still executed; insert/descend/retreat stayed at `j5≈4.71`; vacuum detached (`pickup_box_0001_carryon`); retreat fraction 1.0; `AddPlacedBox placed_0_0_0`. Then `_exit_to_portal` cartesian `place_exit` planned the reverse wrist hop (`j5` 4.707→1.565, 72 pts, 19.25 s) and aborted `ExecuteTrajectory -4`. Trials 1 and 2 died in `_home_arm` `place_exit` (`MoveGroup -2`, cartesian fraction 0.000) with empty catalog ids.

Yaw-fold aligned stage/transit heading; collision-aware IK still selected the flipped-wrist branch, so `|Δj5|<0.5` on transit failed. Trial 0 `joint_ring` n=2001 span=20.0 s (`capture_complete` on that dump); suite-level `capture_complete` is false because trials 1–2 rings are ~0.13 s. MOTION-OCC stays open. OCC-1 not closed.

## Reply -- eng/cursor -- 2026-09-21 19:21 -- cursor/grok-4.6

Offline diagnosis of `2026-09-21_1838` trial 0 return-to-observe. No new sim.

Return is pose-only. `_exit_to_portal` builds a new cartesian pose (portal XYZ, orientation hardcoded `[1,0,0,0]`) and does not copy `_segments_log` waypoints. `GetCartesianPath` used 1 waypoint, `jump_threshold=0`. `_segments_log` is cleared at each `run_trial`. `goto_observe` is a 2-point FJT to YAML `pickup_observe` joints; `_wrap_near` is ±2π only. That stage was not called: `_home_arm` returns on `place_exit` failure.

Observation (abort): JTC `joint 0` (`elfin_joint1`) position error -0.912 vs 0.30, `goal_time` +5.007 s → `CONTROL_FAILED` -4. Planned `place_exit` 72 pts / 19.25 s, `j1` 4.234→-1.728 (`Δ=-5.96`, peak 0.471 = vel cap), `j5` 4.707→1.565 (`Δ=3.14`, unflip starts t=6.54). Measured abort also over-tol on `j5` (0.480) and `j6` (1.078). Tracking `|meas-plan|>0.30` first at t=17.2 (j1). Retreat quat `[0,1,0,0]`; exit quat `[1,0,0,0]`. Vacuum already off.

Observation (post-abort): FCL `eef_mount_adapter` vs `airport_container_real` is the retry from mixed abort FK, not a timestamped contact during the 19 s execute. Trials 1–2 cartesian 0.000 then `MoveGroup -2`.

Counterfactual (no runtime reverse exists): reverse of inbound joints `Δj1=-2.48` / `Δj5=-3.14` in 11.8 s vs pose-only exit `Δj1=-5.96` / `Δj5=-3.14` in 19.3 s. Reverse of descend/transit suction sits on the placed-box top (`z≈0.777`); that reverse after commit would re-enter occupancy. Reverse of post-flip insert/retreat only keeps `j5≈4.71` (`Δj5≈0`) and still leaves `goto_observe` to fold 4.71→1.77 in one FJT (`wrap_near` cannot fold π).

Mechanism: cartesian interpolation of a new portal pose with a heading-equivalent tool-down quaternion produced a 6 rad `j1` + π wrist-unflip path at the scaled vel cap; open-loop JTC missed `j1` (and `j6`) past `goal_time`. Observe named-pose FJT never ran. MOTION-OCC stays open.

Numbers: `docs/status/evidence/motion_occ/2026-09-21_1838/observe_return_diagnosis.md`.

## Reply -- eng/cursor -- 2026-09-21 20:45 -- cursor/opus-5

Implemented the observe-return fix (defects 1, 2, 3, 4 of the 19:21
diagnosis). Offline gates pass; sim verification is blocked by an
unrelated, uncommitted hull change. No simulation claim for this fix.

Exit pose now keeps the wrist: `_exit_to_portal` builds tool-down at
`nearest_box_yaw(tool_down_yaw(current_quat), 0.0)` instead of the hardcoded
`[1,0,0,0]`. Added `waypoint_generator.tool_down_yaw`; lifted
`_lookup_xyz_quat` into `PickRetreatEvalDriver` so all three consumers share
it. `_plan_cartesian` gained a joint-excursion-per-metre gate that falls
through to OMPL like a low fraction (`cartesian_excursion_rad_per_m`, default
3.0). `_home_arm` now settles and still attempts `goto_observe` after an exit
abort, reporting `EXIT_FAILED_RECOVERED` or `EXIT_FAILED_STUCK`;
`place_metrics.RETURN_FAIL_CODES` treats both like `GOTO_FAILED` after
`HOME`. Joint ring horizon grows from elapsed segment time (ceiling 120 s).

Threshold from data, 1838 trial 0: accepted cartesian segments 0.931
(`insert`), 0.939 (`descend`), 0.975 (`retreat`), 1.089 (`stage_mid`), 1.501
(`stage_late`) rad/m; `traverse` 2.067 is a 9 mm hop judged against the 0.05 m
floor; `place_exit` 4.720 rad/m (`j1` 6.58 rad over 1.39 m) rejected. Bound
3.0 rad/m. `jump_threshold` would not catch it: largest adjacent step 0.157
rad. Live rehearsal pick segments measured 1.330 / 1.383 / 1.385 rad/m, all
accepted, FK denominator resolved every time.

A0 offline: 614 passed across both packages, rebuilt before sim.

A1 blocked. Two non-scored runs
(`docs/status/evidence/motion_occ/2026-09-21_2030_rehearsal_a1`, `_a1b`) both
ended `PLACE_PATH_INFEASIBLE` at `place_state: INIT` with 20/20 candidates
`outside_hull`, so `place_exit` never ran. Clean-room residuals 0 before and
after both, one `/clock` publisher, sim down. A2 not run.

Cause is not perception variance and not this change.
`occupancy_place_paths.sweep_polyline` hull-tests the aperture entry waypoint
at `x = -0.745`, exactly `-length/2` of the 1.49 m hull, so
`contains_oriented_box` rejects it for any payload including 1 cm. Replaying
the 1838 winner's own `replay/surface_2d.json` and waypoints through today's
code yields `outside_hull` where that run recorded `ok`, `geometry_hash`
`cb41f55f243786d5` unchanged. `occupancy_place_paths.py` (mtime 20:00) and
the new `container_geometry.py` hull helpers were edited after the 18:40 run
and before this work began at 20:10; `luggage_planning` is
`--symlink-install`, so the live source ran. That is OCC hull scope.

Reproducer, numbers, and dump inventory:
`docs/status/evidence/motion_occ/2026-09-21_2030_rehearsal_a1b/a1_block_report.md`.
Eng note: `docs/agents/eng/2026-09-21_2045_observe-return-exit-yaw-and-excursion-gate.md`.
MOTION-OCC stays open. OCC-1 stays open.

## Reply -- eng/cursor -- 2026-09-21 21:05 -- cursor/opus-5

User-directed: fix the hull block, then rerun A1/A2. Hull block cleared. A1
now stops one stage later, at `transit`. `place_exit` still not reached, so
the observe-return fix remains unverified in sim. A2 not run.

Hull fix: `contains_point` enforced all seven faces including the -X door,
so the insertion entry waypoint at `x = -half_x` put payload corners past
that face for any size. Added `contains_point_through_aperture` /
`contains_oriented_box_through_aperture` to `container_geometry` (+X, ±Y,
floor, ceiling, +Y chamfer enforced; -X open) and pointed `sweep_polyline`
at the aperture variant. Which part of the -X face is actually open stays
with the packing aperture Y-shadow gate and
`insertion_corridor.corridor_blocked`. 20/20 previously-rejected rows are
feasible; 885 tests pass across `luggage_description`, `luggage_planning`,
`luggage_gazebo`, `luggage_packing`, including two new tests pinning both
directions.

Second blocker, reproduced twice
(`docs/status/evidence/motion_occ/2026-09-21_2100_rehearsal_a1c`, `_a1d`):
pick, `stage_mid`, `stage_late` all ok at fraction 1.000, then `transit`
fails `MoveGroup error_code=99999` (FAILURE) with `(pose constraint
fallback)`, so collision-aware IK joint goal failed and the pose-constraint
plan failed after it. Goal pose `[1.03, -0.735, 0.7772]` / `[-0.0, 1.0, 0.0,
0.0]`, selector winner `direct` slot 0, and measured payload `0.507 x 0.360
x 0.247` are all identical to `2026-09-21_1838`, where the same goal planned
as `ompl ok (IK joint goal)`. Inbound transit has regressed from "plans with
a wrist flip" to "does not plan".

Not the observe-return change: `transit` is a `pose_target`, never enters
`_plan_cartesian`, and its boundary carries `cartesian_excursion: null`. The
gate ran on the six cartesian segments that executed and reproduced the
offline fixture values exactly (`approach` 1.635, `attach` 1.383,
`pick_retreat` 1.385, `stage_mid` 1.089, `stage_late` 1.501 rad/m; all
accepted at limit 3.0).

Capture gap worth closing before another `transit` run: no dump records the
attached `pickup_box` dimensions at the failing plan, so attached-payload
geometry cannot be compared across runs from evidence alone.

Four rehearsals total, all non-scored, clean-room residual 0 before and
after each, one `/clock` publisher each, sim down.
`docs/status/evidence/motion_occ/2026-09-21_2100_rehearsal_a1d/a1_second_blocker.md`.
MOTION-OCC stays open. OCC-1 stays open.


## Reply -- eng/claude-code -- 2026-09-21 20:55 -- claude-code/glm-5.3

User-directed: wired the selection-stage row probe that was designed but
never connected (`PlacePathPlanner.plan(probe_fn=...)` existed; the
motion_occ driver never passed it, so candidate selection was MoveIt-blind
and reachability failures only surfaced at execution via boundary dump).

`motion_occ_eval_driver` now probes every occupancy-feasible row through
`/motion_planner/probe_motion_segment` before `select_trajectory`. The
probe segment mirrors the traverse the winner executes: cartesian,
`keep_tool_down`, `allow_ompl_fallback`, orientation from the slot's
place_pose (tool-down at place yaw), waypoints map->world via
`_map_to_world`. Service route (not an in-process executor) so the probe
sees the same executor configuration execution uses, including the 3.0
rad/m excursion gate from 20:45.

Switch and failure policy: `--place-probe on|off`, default on (occupancy-only
selection restored with `off`). Fail-open everywhere — service down at
selection time records `mode: service_unavailable` and skips probing;
per-row timeouts leave the row unprobed, never infeasible. Circuit breaker
stops the sweep after 3 consecutive fail-opens so a dead service cannot
stall every row for the 10 s call timeout. `fraction = -1.0` (no cartesian
solution) is passed to the plan() gate as-is, not blanked to None, so it
rejects like a sub-threshold fraction.

Perf measurement rides along (the 20:45 question "was probe cost ever
measured" — it was not): every row records `probe_s`, and
`candidates.json` / `suite.json` carry `place_probe` aggregates
(mode, probed, ik_reject, fraction_reject, unavailable, sec).

Offline: 11 new unit tests
(`luggage_gazebo/test/test_motion_occ_place_probe.py`) + 40 regression
(planner, home-arm containment, place-only fixture) all pass. A1 sim not
re-run: still blocked by the uncommitted hull regression (20/20
`outside_hull`) reported at 20:45, so no simulation claim. MOTION-OCC
stays open. OCC-1 stays open.

Eng note:
`docs/agents/eng/2026-09-21_2055_selection-stage-place-probe.md`.

## Reply -- eng/claude-code -- 2026-09-21 21:20 -- claude-code/glm-5.3

User-directed (occupancy 注入只覆盖 transit/traverse 的 review finding):
occupancy injection now covers ALL planning segments, and plan/execute
failures freeze the occ snapshot plus the sensor data it came from.

Scope: the hardcoded `("transit", "traverse")` allowlist in
`_prepare_occupancy` is now the `occupancy_segments` parameter, default
`"*"` — insert, descend, retreat, place_exit, staging and pick segments
all get cargo occupancy injected (scene boxes + executor sweep), per the
placement.md "place motion through occupancy" contract. Set
`occupancy_segments:=transit,traverse` to restore the carry-only scope.

False-block prevention (this is why the allowlist existed):
`occupancy_collision_boxes` has no slot exclusion, so a descend onto its
own support column collides with that column plus every inflated
neighbour. Added a landing-footprint exemption (`exempt_footprint_locals`):
each in-hull waypoint anchors an exemption of the yaw-rotated payload
footprint expanded by `arm_radius + inflate` (+ tunable
`occupancy_exempt_margin_m`); exempt cells produce no scene box and do not
collide in the sweep. Hull containment is unchanged for every sample.
Residual, documented: within that corridor cargo-sweep blindness extends
arm_radius+inflate beyond the payload edge during landing segments; MoveIt
URDF + container STL (+ AddPlacedBox in committed flows) still protect,
and the corridor exists only for the segment aimed at that slot.
Probe service now injects occupancy too (same scope + exemption), so
selection-time probes and execution see identical occupancy — closing the
gap where the 20:55 row probe ran against a MoveIt scene without cargo
boxes.

Failure capture: on any failed segment `_emit_boundary` now carries an
`occ` block (present/map_revision/geometry_hash/age) and a `capture`
block; with MOTION_BOUNDARY_DUMP set, `debug_capture.write_failure_capture`
freezes `fail_<t>_<seg>_surface_2d.json`, `fail_..._cloud.ply`
(`/luggage/semantic/cargo_points_untracked`, BEST_EFFORT latest-cache,
decimated ≤20k pts, binary PLY) and `fail_..._depth.npy`
(`/luggage/preprocessed/camera/depth/image`) plus a sidecar. All
best-effort: missing topics land in `capture.missing`, never in a motion
abort. `_freeze_motion_replay` copies the artifacts into `replay/` and
records `capture_missing` in trial extras — deliberately NOT in the
manifest `missing` this wave, so `replay_possible` gates do not flip
before the capture proves reliable in sim. Subscriptions are behind
`debug_capture` (default true); `motion_occ_place_path_run.sh` already
exports MOTION_BOUNDARY_DUMP.

Offline: 642 passed across luggage_planning + luggage_gazebo (baseline
614 + 28 new: exemption semantics, debug_capture round-trip, scope
predicate, replay copy). Sim A/B (`occupancy_segments=transit,traverse`
vs `*`) not run — A1 remains blocked by the uncommitted hull regression
(outside_hull 20/20) from 20:45. No simulation claim. MOTION-OCC stays
open. OCC-1 stays open.

Eng note:
`docs/agents/eng/2026-09-21_2120_occupancy-all-segments-failure-capture.md`.

## Reply -- eng/claude-code -- 2026-09-21 21:50 -- claude-code/glm-5.3

Review follow-ups (items 6-10 of the wave review) + landing.

6/7. Selector stays eval-side this wave (production chain via
pack_to_full_run is single-slot; recorded as follow-up below). The
waypoint override is no longer hand-written in the driver:
`apply_variant_to_segment` gained `frame_convert` and
`motion_occ_eval_driver._execute_segment` now calls it (map->world via
the scene transform), so the production path-injection point is one
shared function.
8. Dead `n==3 -> 2` normalization removed; the `--n 2` default lives in
`parse_args` once (main()'s duplicate append removed). Explicit `--n 3`
survives — pinned by test.
9. `load_selector_weights` no longer resolves only against `__file__`:
src tree first, then the installed share dir
(`selector_weights_path`), and an unresolvable config now warns on
stderr instead of silently running DEFAULT_WEIGHTS (the copy-install
trap). Same bug class found pre-existing in
`luggage_description/handeye_layer3.py` (xacro via `__file__/../config`
missing under copy install) — left for its owning thread, not this wave.
10. `debug2026` root note deleted (D555 large-frame DDS conclusion kept
as a memory line); orin tarball gitignored.

Offline after the fixes: planning 475, gazebo 171, packing 110 passed;
perception 1012 passed + 1 pre-existing failure (PF-A1 static audit
fails on HEAD too, not this wave); description 168 passed + 3
pre-existing failures (handeye/livox xacro resolution, above). Wave
landed as thematic commits (msgs / payload-geom / hull+packing /
perception map / motion core / MOTION-OCC+eval).

Follow-up debt (this thread): selector not in the production chain —
acceptance is pack_to_full_run.sh placing through the selector; probe
service is the seam. MOTION-OCC stays open. OCC-1 stays open.
