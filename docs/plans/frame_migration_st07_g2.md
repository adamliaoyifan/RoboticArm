# ST07 generation 2 -- paths and selector service

Parent FRAME-MIGRATION-20260929. This is the generation-2 execution plan for
occupancy sweep, path candidates, grid search, and the place selector
service (F11–F13, and the last consumer of F08). It revises the ST07
section of Claude's 2026-09-29 phase-3 sketch
(`~/.claude/plans/codex-3-st06-atlas-st07-bright-mist.md`) against the
code on `/tmp/elfin-frame-migration-st06`. Module-task rules from
`docs/plans/frame_migration_stages.md` ST07 apply: no hardware motion, no
scored simulation, no master merge, no shared-worktree edits. v1
`SelectPlacePath` and `BuildMotionSequence` keep their names and fields.
v2 endpoints are new services on the same nodes.

## Base lineage

Accepted ST06 head:
`074daf34c4df1c42bf2e3a7f58711e367ba72375`
on `integration/frame-migration-st06-20260929`. It contains the repair of
the 2026-09-30 changes-required reply and the reissued F14 proof.
`102404d` is an ancestor and is not the base. `dff51dc` remains an
ancestor. At `074daf3` the migration checker is baseline 50/50, exit 2,
13 pending: F01, F08, F11, F12, F13, F15, F16, F17, F18, F21, F22, F23,
F24. F14 is verified. No `/tmp/elfin-frame-migration-st07` worktree
exists yet. Cut that worktree from this plan's commit, which is `074daf3`
plus this document.

## What stays from the phase-3 sketch

- F11 converts a container slot plus the carried grasp into a world TCP
  pose, orientation included.
- F12 sweeps container-frame paths against a v2 surface with a verified
  geometry hash and map revision. Directed atlas queries stay on one
  concrete yaw bin.
- F13 adds `SelectPlacePathV2` and `BuildMotionSequenceV2`. The node and
  the in-process caller share one conversion helper. v1 services stay.
- F08 flips only when both v2 consumers exist on the same commit as the
  ST05 producer. Expected migration result after a passing ST07 is exit 2
  with the nine rows that this stage does not own: F01, F15, F16, F17,
  F18, F21, F22, F23, F24.
- Invalid requests make zero MoveIt, probe, sweep-mutation, and commit
  calls. Infeasible place search is `PLACE_PATH_INFEASIBLE`, never
  `BIN_FULL`.
- The 64-case matrix is the parent matrix: container yaw
  `{0, π/2, π, atan2(4, 3)}`, translation `{(0,0,0), (1.5,-0.4,0.2)}`,
  floor `{0.53, 0.71}`, support `{0, 0.30}`, grasp offset
  `{(0,0,0.16), (0.10,-0.05,0.16)}`, payload WDH `(0.80, 0.50, 0.32)`.

## Corrections bound by the current code

### TCP conversion order

`box_to_tool_target` (`frame_transforms.py`) already implements
`T_frame_tool = T_frame_box * T_box_tool` for a `box_center` /
`box_axis_pi` record and a grasp whose source is `suction_contact_frame`
and whose target is `box`. `slot_from_contract`
(`waypoint_generator.py`) returns a `geometry_msgs` box-center pose. It
rebuilds orientation from `atan2(qz, qw)` and then composes the container
rotation by hand. Those two functions do not compose.

v2 place conversion is:

1. Admit the slot with the slot profile (`container_link`, `box_center`,
   `box_axis_pi`, scene stamp).
2. Admit the grasp. Its scene revision matches the slot. Its translation
   is the contact offset in box axes, including a lateral offset.
3. `box_to_tool_target` in the container frame.
4. `transform_pose` with the admitted container-to-world transform.

The world TCP position and quaternion are that result. `slot_from_contract`
remains the box-center world seam. On the same 64 cases it must match
`transform_pose` of the slot record within 1e-9. If the hand-rolled yaw
extract disagrees, `slot_from_contract` becomes a wrapper over
`transform_pose`. One world-conversion formula.

The place contact pose is that TCP. It is not `slot center + (0, 0, h/2)`.
Staging, transit, insertion, and retreat stay distinct lengths from the
existing clearance helpers. Each vertical lift is
`rotate_vector(container_quaternion, (0, 0, lift))` applied to the TCP
point. On an admitted horizontal container this equals world `+z`. The
grasp's lateral offset moves the TCP and does not move the lift direction.
The v1 place builder keeps the centered `h/2` shortcut and
`nearest_box_yaw`. Pick legs stay on the current builder; F09 stays
pending.

### Sweep frame: box footprint and TCP are different polylines

`sweep_polyline` treats each point as a suction point. The payload hangs
in `[z - height, z]`, and the XY footprint is centered on that point. That
matches a centered grasp, where TCP xy equals box-center xy. An off-center
grasp moves the TCP off the box center. Centering the footprint on the TCP
then checks the wrong cells.

Binding:

- Occupancy samples the box-center polyline.
- Motion waypoints and service poses are the TCP polyline.
- The two differ by the grasp translation rotated by the box quaternion.
- Centered-grasp fixtures may store one polyline and must still label it
  as both. Off-center cases in the 64-matrix assert the TCP xy differs
  from the box center and the swept cells follow the box center.

Occupied cells block at their measured top. Unknown cells whose height is
above the floor by more than `support_tol` block at full inner height.
Unknown cells at the floor are free. A column shorter than the carry
suction height can be flown over: `generate_path_variants` raises carry z
to `occupied_top + payload_height + margin`, clamped by
`suction_ceiling_z`. A short wall is not a detour fixture.

Legacy `floor_z` on a v1 surface dict is an offset added to
`center_base.z - inner_h/2`, not an absolute container floor. Putting
`0.53` in that field produced `floor_map_z = 1.06` in a probe. Fixture
surfaces are v2 `kind: surface` records (`floor_z_container` absolute,
`surface_z_container` absolute). They enter through
`OccupancySnapshot.from_surface_2d` and the existing v1 adapter.

### Output orientation is container yaw pasted onto world xyz

`trajectory_selector_node._segments` rotates waypoint xyz with container
yaw and then writes a tool-down quaternion built from the slot yaw alone
(`cos(yaw/2), sin(yaw/2), 0, 0`). At a nonzero container yaw the position
is in world and the quaternion is still in the container yaw. v2 response
poses are the `transform_pose` result: position and quaternion. The
yaw-only helper stays on the v1 service.

`_map_to_world` (yaw plus origin, z unrotated) matches `transform_pose`
for positions when roll and pitch are zero. Admitted scenes are
horizontal. The matrix yaw `atan2(4, 3)` and `π/2` are the position
checks. v2 code calls `transform_pose` anyway so the quaternion is not a
second formula.

The node already copies slot yaw into the selector dict. The missing
request fields are the grasp, the slot profile, and the container
transform. v2 carries those. v1 `SlotSpec` has no frame field and is not
relabelled.

### Directed atlas query is already 2π

`ReachabilityAtlas._nearest_yaw_index` picks the nearest bin over a
wrapped `2π` distance. `query` uses that index. `query_pi_equivalents`
and `is_reachable` consult the π twin on a v2 artifact. Grid search must
keep calling `query` with the slot's directed yaw so yaw `0` and yaw `π`
are different bins. It must not call `is_reachable` or
`query_pi_equivalents` on the path graph. `atlas_pi_fold_rejected`
remains the contract negative (`yaw_period_rad == π` raises
`ATLAS_SEMANTICS_MISMATCH` before any cell is expanded). The graph test
uses a tiny in-memory atlas, not a production NPZ and not Gazebo.

### The sketched wall does not produce a detour

A probe against `generate_path_variants` / `sweep_polyline` on the ST06
tree showed:

- Lateral offsets are `OFFSET_DELTAS = (0.10, 0.20, -0.10, -0.20)` m on
  a via point. The family cap keeps `offset_+0.10` once four variants
  exist.
- Occupying exactly the cells under the direct footprint made `direct`,
  `clear_top`, `corridor`, `offset_+0.10`, and `via_clear` all return
  `occupancy_collision`.
- A full-height column (top at inner height, suction ceiling 1.79 m under
  a 2.01 m hull) rejects fly-over. A two-cell gap at 0.10 m resolution is
  narrower than the 0.50 m payload axis.
- `grid_short` outranks `direct` in `_METHOD_RANK`. A fixture that leaves
  a live atlas attached can crown a grid path over the straight geometric
  path.

The three map fixtures therefore run with `atlas=None`, `inflate_m=0`,
`arm_radius_m=0`, and a probe spy. Grid behavior is the separate stub
atlas test below.

ST07 adds one geometric method, `gap_detour`, emitted only when the
straight footprint hits a full-height obstacle and that obstacle column
contains a free run at least as wide as the swept cross-axis (the payload
dimension perpendicular to travel, plus pads). The via sits at the center
of the nearest such run, at the clamped carry z. At most one `gap_detour`
row is emitted. It is absent on an open map. Its method rank stays at the
default 9, so it wins only when every better-ranked row is infeasible.

### Fixture contract

Surfaces are built in the test module as v2 records and written under
`src/luggage_planning/test/data/frame_migration/` after a local probe
shows the three outcomes. The checked-in arrays must satisfy the
following. Cell integers are whatever the probe records; the outcomes are
the gate.

Map: resolution 0.05 m, inner 2.0×2.0×1.48 m, floor 0.53 m, ceiling
2.01 m, payload `(0.80, 0.50, 0.32)`, centered grasp, yaw 0. Portal and
slot xy are inset so the 0.40 m half-length stays inside the hull
(the probe used portal `(-0.45, 0, 1.10)` and target `(0.45, 0, 0.90)`).
Column height for a blocking cell is the inner height, so the top equals
the ceiling and `clear_top` still collides.

| Fixture | Surface | Required outcome |
|---|---|---|
| `open_direct` | every cell free at the floor, including one unknown cell whose top is the floor | winner exists, `method == direct`, probe count 0 |
| `blocked_corridor_detour` | one full-height wall across the straight footprint, one free gap off that footprint wide enough for the 0.50 m cross-axis; one wall cell is `unknown` with top above the floor | `direct`, `clear_top`, `clear_headroom`, `corridor`, and `offset_*` are infeasible with reason `occupancy_collision`; winner `method == gap_detour`; an independent cell walk of the winner box-center polyline reports zero occupied or unknown-above-floor cells |
| `blocked_destination` | the same wall with no free run | winner is `None`, reason `PLACE_PATH_INFEASIBLE`, every rejected row reason is `occupancy_collision` or `outside_hull`, none is `BIN_FULL`, probe count 0 |

Repeat `blocked_corridor_detour` with box yaw `π/2`, so the cross-axis is
0.80 m. The gap that passed the 0.50 m axis must then be infeasible, and
a wider gap must make `gap_detour` win. That is the
`nonsquare_width_axis` check.

A short-wall control (column top 0.05 m above the floor on the straight
line) must remain feasible by a raised variant. It documents why the
detour fixture uses full height.

Tie-break, same open surface, two slots, atlas absent: equal cost and
different methods → lower `_METHOD_RANK` wins (`direct` before
`gap_detour`). Equal cost and equal method → smaller `slot_index` wins.
The third key, `path_hash`, is covered by two same-slot rows whose
waypoint lists differ only after the hash's 4-decimal rounding would
collide; the lexicographic hash order is the expected winner.

### Service shape

`SelectPlacePathV2.srv` request: `request_id`, `box_wdh[3]`,
`PoseContractV2[] slots` (slot profile), `slot_scores`,
`SurfaceContractV2 surface`, consumer `inner_size` and geometry hash,
`PoseContractV2 start_tcp` (container frame, suction contact),
`float64[] start_joints`, `PoseContractV2 grasp`, container-to-world
transform fields the adapter already uses for `transform_pose`,
`FrameIdentityV2 identity`, `excluded_ids`. Response: `success`,
`reason_code`, `slot_index`, `selection_id`, `path_source`,
`MotionSegment[] carry_segments` whose poses are world TCP (position and
quaternion), `PoseContractV2[] waypoint_poses` (motion profile, world),
`rows_json`. No field is left at the zero pose unless the oracle position
is zero.

`BuildMotionSequenceV2.srv` place phase consumes the same slot profile,
grasp, and container transform and returns place legs whose contact pose
is the TCP. Pick phase delegates to the existing pick builder and does
not flip F09.

`place_selection_client` and `scripts/select_place_path_probe.py --v2`
call the real service. In-process `select_in_process` and the service
return the same `slot_index`, `path_hash`, and world waypoint positions
and quaternions within 1e-6 on all three fixtures, with the same
tie-break config and `atlas=None`.

`PlacePathPlanner` stays behind `mode: legacy`. The task grid stays on
the ST06 v1 isolation marker.

## Decision-complete acceptance

Fresh worktree `/tmp/elfin-frame-migration-st07`, branch
`integration/frame-migration-st07-20260929`, base `<ST06-HEAD>`. CPU
only. Source only `/opt/ros/humble` and a clean colcon install of that
worktree. Commit sources before the verification commands.

```bash
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_selector.py -v
python3 -m unittest discover -s src/luggage_planning/test -p 'test_select_place_path*.py' -v
python3 -m unittest discover -s src/luggage_planning/test -p 'test_trajectory_selector.py' -v
python3 -m unittest discover -s src/luggage_planning/test -p 'test_occupancy_place_paths.py' -v
python3 -m unittest discover -s src/luggage_planning/test -p 'test_waypoint_generator.py' -v
python3 -m unittest discover -s src/luggage_planning/test -p 'test_place_path_planner.py' -v
```

Plus the planning battery at the base count, contracts 15/15, baseline
50/50, and `check_frame_contracts.py --mode migration` exit 2 with the
nine pending rows named above. F08, F11, F12, F13 flip to verified only
with runtime proofs whose `source_sha256` covers every listed row source
plus `CONTRACT_SOURCES`. Before hashing, extend the rows with the new
service files and the conversion helper if it is a new module. Do not
drop the existing sources.

`test_frame_migration_selector.py` records one JSONL line per case via
`FRAME_MIGRATION_CASE_LOG`. The oracle for the 64 TCP cases is a second
quaternion implementation in the test (SciPy or an independent Hamilton
product), not a second call to the production function. Pure Python error
≤ 1e-9 m and ≤ 1e-9 on quaternion components after sign alignment. After
real CDR serialization, ≤ 1e-6. Required case ids include
`offcenter_grasp` (TCP xy differs from the box top center),
`center_grasp`, `normal_not_translated` (a rotated lift adds no origin),
`translate_rotate90`, `translate_rotate53`, `tool_pi_distinct`,
`tool_sign_equivalent`.

Negatives, each once, each with zero probe and zero MoveIt client calls:

| Input | Code |
|---|---|
| empty frame or missing grasp | `MISSING_IDENTITY` |
| world frame on a slot record | `FRAME_CONTRACT_MISMATCH` |
| TCP record labelled `box_center` | `POSE_REFERENCE_MISMATCH` |
| tool quaternion labelled `box_axis_pi` on the slot | `ORIENTATION_SEMANTICS_MISMATCH` |
| contract version 1 on a v2 endpoint | `CONTRACT_VERSION_MISMATCH` |
| stale `map_revision` or geometry hash | `MAP_REVISION_MISMATCH` or the existing `OccupancyMismatch` text that already names `PLACE_PATH_INFEASIBLE` |
| nonfinite position | `INVALID_NUMERIC_PAYLOAD` |
| zero quaternion | `INVALID_QUATERNION` |
| v2 atlas record with `yaw_period_rad = π` passed to grid setup | `ATLAS_SEMANTICS_MISMATCH` |

Named inapplicable: acquisition-stamp mismatches on scene-stamped slots,
and `BIN_FULL`. An injected position-only world conversion (identity
quaternion, translated xyz) must fail `translate_rotate90`. An injected
π-fold that maps yaw `π` onto yaw `0` before `query` must fail the stub
atlas bin assertion.

v1 `SelectPlacePath` response bytes for a fixed request stay identical
to the base. That pins the centered shortcut and the yaw-only quaternion
on the old service.

## Evidence

`docs/status/evidence/frame_migration/ST07-<date>/` holds `summary.md`,
`manifest.json`, the case JSONL, the three fixture arrays, and the
service-versus-in-process agreement records. Cap 64 MiB. No Gazebo, no
atlas campaign, no bag. A failed unittest keeps the JSONL for that case
id; do not overwrite the directory.

## Bound at dispatch

- Base product commit: `074daf34c4df1c42bf2e3a7f58711e367ba72375`.
  The worktree base is the commit that adds this file, which must have
  that commit as its parent.
- Pending at the product head: F01, F08, F11, F12, F13, F15, F16, F17,
  F18, F21, F22, F23, F24. After ST07 the expected pending set is F01,
  F15, F16, F17, F18, F21, F22, F23, F24. Do not flip any other row.
- Atlas suite on the product head: 32/32, no skips. Full planning,
  packing, perception, and description batteries were not re-run on
  `074daf3`. Before the first product edit, record those counts on this
  base. The 102404d summary (planning 653, packing 126 plus 1
  pre-existing, perception 1048 plus 1 pre-existing, description 187) is
  history, not this base's count.
- `gap_detour` is the detour producer. The fixture section's outcomes are
  the gate. Cell index ranges are printed by the implementation probe and
  then checked in; they are not frozen in this plan.
- Real-message suite pattern: `test_select_place_path_service.py`, plus
  `test_frame_migration_selector.py`.

## Release

Generation 1 (`docs/agents/discuss/2026-09-29_1705_frame-stage-07.md`,
owner eng/codex/gpt-6) is superseded by the generation-2 dispatch whose
product base is `074daf3` and whose `plan_revision` is the commit that
adds this file. Owner stays eng/codex/gpt-6.
