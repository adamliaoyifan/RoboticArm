# Placement

How a box is proposed inside the container after detection. Hull identity,
clipping, and the `ComputePlacement` failure taxonomy stay in
[container_geometry.md](container_geometry.md). What may fill the occupancy
map and the payload box is [real_scenario.md](real_scenario.md): live
perception plus allowed priors, not catalog/GT/planned pose.

The live ROS 2 Humble planner is `placement_planner_node` calling
`placement_solver.generate_candidates` / `solve_placement` on
`surface_map_2d`. A separate FreeSpaceModel + `placement_scoring` path exists
for offline replay and the unported ROS 1 planner. That path is not the live
node until the node and this document switch together.

Hard rules:

- Candidate generation enumerates discrete support-surface poses. It is not
  Monte Carlo sampling from a pose prior.
- Each enumerated pose is a complete oriented payload box. Center-point
  placement checks are invalid (same rule as the hull document).
- Hull, overlap, and top-clearance gates run before ranking, retention,
  `top_n`, or first-feasible selection.
- Stacking on previously placed cargo is allowed only on **known** support.
  Unobserved columns may land on the a-priori floor (`peak ≈ floor_z`,
  `support_source=floor_prior`). Stacking on an unobserved surface
  (`peak > floor_z`) is `unknown_above_floor`.
- `surface_2d` occupancy must come from live depth and/or a verified
  measured commit ([real_scenario.md](real_scenario.md)). An unverified
  planned slot, catalog size, or `GetCurrentBox` is not occupancy. Current
  Humble planned-geometry commit is `RS-2` / `RS-3`, not the contract.
  Node-level depth integrate is closed (`RS-1`).
- `request.placed` is a second occupancy source for overlap and the
  insertion corridor. It must not be a duplicate of the same unverified plan
  used to hide commit–reality disagreement (`RS-3`).
- The floor prior is legal when no map has been published, as empty-floor
  **existence**. It must not claim the interior is empty of cargo (`RS-4`).
  A rejected map must fail closed, not answer from the floor prior.
- `ComputePlacement` must fail closed until `height_source=MEASURED_SUPPORT`.
  TOP_ONLY is a detection, not a placement payload (`RS-8` closed).
- Place **motion** (cartesian, OMPL, or any later search) must plan against
  the same cargo occupancy the solver reads. A collision world of only URDF
  plus `scene_manager` boxes is not occupancy. Current Humble traverse is
  tracked defect `RS-27`.

## Candidate generation (live Humble)

`generate_candidates` slides an axis-aligned footprint over the
container-local 2.5D height map:

- Yaw is snapped to `{0, π/2}` footprints.
- `peak` is the maximum `height` under the footprint.
- The box rests on that surface: `z = peak + box_height / 2` in
  floor-relative coordinates.
- Default map resolution is 0.05 m with `stride_cells=1`.

`floor_prior` is a synthetic all-unknown, height-zero grid used when the
planner has no accepted `surface_2d`. The floor's **existence** is geometric
(scene_tf), not a perception claim. It is not a sampling distribution.

## Stacking

A previously placed box becomes a support surface when its footprint columns
are `occupied` (commit rasterized as `SOURCE_GEOMETRY`, or sensor-occupied
once depth integration exists). Face contact with a placed AABB is allowed;
positive-volume overlap is a capacity reject.

A stack candidate is feasible when:

- the footprint has no unknown cells, or every unknown cell is still at the
  floor (`peak ≈ floor_z` within `support_tol`, default 0.05 m);
- `support_ratio ≥ min_support_ratio` (default 0.6): fraction of footprint
  cells within `support_tol` of `peak`;
- remaining top clearance is at least `clearance_margin` (default 0.03 m);
- hull, overlap, aperture, and insertion-corridor gates pass.

There is no API to target a specific prior box. Stacking is an automatic
candidate at XY cells whose `peak` is that box's top.

Boxes on the same support surface keep a horizontal gap of
`same_layer_clearance_m` (frozen at 0.05 m for this generation) between
yawed footprints. Each footprint is grown by half that gap and the grown
polygons must not intersect; a gap equal to the threshold passes.
Wall and corridor margins stay on the hull and are not added again here.
A box whose bottom rests on a placed box's top may cover that support
parent; it still keeps the gap against every other box on its own layer,
and positive-volume overlap is still rejected.

## Scoring (flatten first)

Live score, defaults in `placement_solver._default_params`:

```text
score = 0.5 * support_ratio
      + 0.3 * clearance_score      # remaining height above the box, in [0, 1]
      + 0.2 * compactness          # 1 - peak / inner_h
      + 0.15 * confidence_ratio    # fraction of sensor cells only
```

Weights do not change after a commit. The height map changes, so stack
candidates appear. `compactness` and remaining clearance both reward **low**
placements: fill the floor before climbing. `w_compactness`'s comment
("prefer low placements (good stacking)") means leave headroom for later
stacks, not stack now.

`confidence_ratio` counts only `confidence=sensor` cells. Geometry-committed
tops and `floor_prior` columns score 0 on that term.

`solve_placement` selects `max(feasible, key=score)`. The Humble node keeps
`top_n=2000` so corridor and aperture gates see deep slots, not only the
highest scores.

Stacking wins once no feasible floor slot remains (floor tiled, remaining
floor lost to overlap / hull / corridor, or clearance under a full-floor
occupied map).

## Reachability atlas on the live node

`placement_planner_node` may annotate solver candidates from the
reachability atlas. A high-confidence unreachable cell is a hard reject
(`atlas_unreachable`). A reachable or marginal cell adds
`w_reachability * prior` to the score. This gate is not the FreeSpaceModel
replay scorer, and it does not generate slots.

The active scene file selects the atlas cell. A scene hash that matches
`data/reachability_atlas` loads `config/atlas_runtime.yaml` (simulation,
`scene_tf.yaml.example`). A scene hash that matches
`data/reachability_atlas_deploy` loads `config/atlas_runtime_deploy.yaml`
(`scene_tf_deploy.yaml`). An explicit `atlas_runtime_config` overrides
that match. No match keeps the simulation cell.

The simulation runtime is the certification
cell: `hash_policy: strict`. Atlas identity is the reachability identity:
the MD5 of the place-reachability subset of the scene file
(`compute_reachability_scene_hash`: container, robot base and attachment
fields, numerically rounded) plus the MD5 of the live `robot_description`
on `robot_state_publisher` (the same MD5 the atlas builder stores). The
whole-file scene MD5 (`compute_scene_tf_hash`) is a diagnostic only;
comments or pickup-platform edits must not change admission. Strict mode loads the atlas only when both
hashes are present and match the atlas metadata. Otherwise
`ComputePlacement` returns `ATLAS_IDENTITY_UNVERIFIED` and does not
hard-reject or re-rank. `hash_policy: warn` on the deploy cell may keep a
mismatched atlas; that cell is not a certification result.

`TrajectorySelector` grid paths use the same two hashes. Under `strict`,
a missing or mismatched hash disables the grid
(`GRID_DISABLED_SCENE_TF` / `GRID_DISABLED_URDF`) instead of searching
stale connectivity. The motion planner refuses IK seeds from an atlas
that fails the same check.

### Atlas is a prior for route search

The `atlas_unreachable` slot reject above is the only hard use of the
atlas. `hash_policy: strict` only checks that the loaded artifact matches
this robot and scene. A matching atlas still does not reject unknown or
low-confidence unreachable cells before a route is built. Inside
place-route search the atlas ranks and seeds; it does not decide
feasibility. An atlas miss, a failed atlas seed, or a sampled-cell IK
failure is not proof that the continuous configuration space has no
route; online IK, collision checking and certification decide.

Route search (`grid_path_search` / `route_producer`) is state-conditioned:

- Every hop is solved from the **actual frontier joint state** (the real
  endpoint of the previous edge), not from a per-target seed chosen in
  isolation.
- Short constrained hops (`INSERT`, `DESCEND`) are first solved as
  continuous local IK from the frontier (exact refine seeded by the
  frontier q). Same-branch destinations are tried before any destination
  on another elbow, wrist or heading branch; those are deferred, not
  deleted.
- Each goal's branch is back-seeded up the entry hops to the last
  joint-space hop (`PORTAL_TRANSIT`), so a goal on another branch has a
  same-branch cartesian chain and the branch change is made in that
  joint-space transit.
- When a continuation fails, the search backtracks to the nearest upstream
  choice that can change the result (hover goal, then entry, then slot)
  instead of only retrying another final IK. Multi-branch goals stay as
  ordered alternatives of that full-route search.
- A 180-degree heading alternative is legal only when the frozen
  attachment transform and payload geometry are re-checked for that
  heading; an equal rectangular footprint is not sufficient.
- The request budget (10 s total, 2 s per edge) is shared across
  expansions. A failure class (goal, edge class, reason) that fails
  `MAX_SAME_FAILURE_PER_GOAL` (3) times stops building for that goal, so
  the next goal and slot get attempts.

The frozen Reason code (`NO_FEASIBLE_ROUTE`, `PLANNING_BUDGET_EXHAUSTED`)
is unchanged. Each refusal also carries `detail.search_outcome`:

| search_outcome | Meaning |
|---|---|
| `UNREACHABLE` | Every goal was tried, no failure class was capped, no budget refusal. |
| `SEARCH_INCOMPLETE` | Budget, a budget refusal or a failure-class cap left alternatives open, or no IK seed refined onto any goal. Not a reachability verdict. |

`UNREACHABLE` means only that the bounded alternative set was exhausted;
it is not a proof that no configuration-space route exists.

`BIN_FULL` is packing capacity only and is never either case.
`detail.coverage` reports edges built, goals tried and left open, capped
failure classes and budget refusals.

## Occupancy update after place

Required source of `surface_2d` is live depth integration and/or a verified
measured commit. Gazebo physics and MoveIt collision objects are not the
cargo map. See [real_scenario.md](real_scenario.md).

Current Humble path (tracked defect `RS-2`, `RS-3`, `RS-6`; `RS-1` closed
at the node/service):

`/cargo_map/integrate_cargo_view` → stamp-matched cloud →
`CargoVolumeMapper.integrate_points` (`confidence=sensor`). Fail-closed on
unsettled, hash/revision mismatch, stale/duplicate stamp, missing TF, or
empty view. The service still reads
`/luggage/semantic/cargo_points_untracked`, not the tracker-gated pickup
track.

Post-place eval views do not wait for a YOLO mask. The driver unprojects
the depth image, keeps points inside the planned resting box (slot center,
suitcase size, yaw, 0.05 m margin), and publishes that crop on the
untracked topic. The slot is a crop, not a painted occupancy: an empty crop
does not call the service and does not change `map_revision`.

View order after retreat is `aim` at the portal, then `lookback` (portal
eye raised 0.20 m and backed off 0.15 m along the opening normal). The
in-container lift is not a cargo view. Each view's
`expected_map_revision` is the previous view's `resulting_map_revision`
when that view was integrated, otherwise the stats revision. The sequence
stops at the first view with `footprint_hits >= 1`, integrate success, and
a revision increase. A view with no footprint hit keeps `empty_reason` in
`TARGET_NOT_IN_FRAME`, `TF_TIME_MISMATCH`, `DEPTH_INVALID`,
`BOX_CROP_EMPTY`, or `MAP_FILTERED`. An empty view is not a success.
YOLO bbox, confidence, and mask counts are diagnostic only. The observe
pose is not frozen (`cargo_view_policy.frozen` is false).

`/cargo_map/add_placed_box` → `CargoVolumeMapper.mark_placed_box`:

1. Record the box on the commit ledger (idempotent within 0.05 m center/size
   and matching yaw).
2. Rasterize the interior as `SOURCE_GEOMETRY` voxels (lock against
   free-space misses).
3. Bump `map_revision` and republish `/luggage/cargo_map/surface_2d` plus the
   commit ledger.

Eval pack drivers currently pass the **planned** slot into AddPlacedBox, not
a measured settled pose (`RS-2`). Humble has no `VerifyPlacedBox` node
(`RS-6`); eval labels `VERIFIED` from `ign model --pose`. Nothing in the
product loop calls `IntegrateCargoView` yet (`RS-7`); the OCC-1 eval driver
is the node-level caller.

`surface_map_2d` remains the solver join contract: per-column top occupied
height above the inner floor, plus `state`, `clearance`, `confidence`,
`geometry_hash`, and `map_revision`. Next `ComputePlacement` reads that
peak. The defect is what is allowed to raise the peak, not that the solver
reads a height field.

`/scene_manager/add_placed_box` only inserts a MoveIt collision object. It is
not a cargo-map commit.

A skipped map commit leaves the height field empty. The next plan can then
propose a floor slot through the real box. Duplicate commits do not bump
`map_revision`. Voxel tops are `(top_occ + 1) * resolution`, so a committed
top can sit slightly above the geometric lid.

## Place motion through occupancy

`ComputePlacement` choosing a feasible slot is not permission to carry.
Every place-motion planner (cartesian interpolation, OMPL, or a later
search) must query the **current** cargo occupancy at the request
`geometry_hash` / `map_revision`. Occupied cells and unknown-above-floor
cells are obstacles for the swept payload and arm. Empty-floor existence
(`floor_prior`) is legal only when no accepted map exists.

A single straight cartesian to the slot, collision-checked only against
MoveIt URDF and `AddPlacedBox` scene objects, is a defect (`RS-27`).
Gazebo contacts are not a substitute for that occupancy query.

### Candidate paths

When a place segment is planned, the planner must produce a **set** of
paths (N ≥ 2 whenever the first candidate is in collision, below the
cartesian fraction gate, or has no IK), not one discarded line plus one
failed OMPL call. Methods may include raised/offset cartesian waypoints,
alternative yaws already feasible at the slot, and free-space search.
Each candidate that collides with occupancy is infeasible, not a scored
backup.

### Path selector

A selector picks among remaining collision-free paths by a scalar cost
with three occupancy terms (weights live in planner config, not in
call-site constants). When `ComputePlacement` returns more than one
feasible slot, the selector scores every feasible `(placement, path)`
pair. Placement quality is a fourth cost term so a cheap path into a
worse slot does not beat a slightly longer path into a better slot.

| Term | Meaning |
|---|---|
| Placement | Solver score of the target slot, inverted so better slots cost less. |
| Dimension | Swept payload size versus remaining occupancy clearance and opening aperture. Penalize paths that need more width/height than free columns allow. |
| Safety | Minimum clearance from the swept payload and arm to occupied and unknown cells. Collision is infeasible; near-miss is a cost. |
| Efficiency | Path length, estimated duration, and cartesian fraction. Prefer a short high-fraction carry when it is still safe. |

Execute the minimum-cost feasible `(placement, path)`. If the set is empty, fail closed
with a motion/path reason (`PLACE_PLAN_*` or a dedicated path-infeasible
code). Do not relabel that as `BIN_FULL` or `PLACE_CANDIDATE_EXHAUSTED`.
Retain every tried path and rejection reason in the dump (same rule as
exhausted placement candidates).

`TrajectorySelector` keeps the scored set: geometric variants and, when a
reachability atlas is loaded, shortest paths on its reachable cells. The
atlas is a path prior here. It does not generate placement slots.
`motion_occ` applies the winner before `PlanMotion`. A `joint_path` segment
executes those joint knots with an occupancy sweep and does not fall back
to `RRTConnect`. `motion_planner_node` still injects occupancy and re-sweeps;
it does not select. `RS-27` stays open until a scored run shows every
transit and traverse took the selector path.

## Replay / redesign path (not the live node)

`FreeSpaceModel` replaces the sliding window with a persistent heightmap,
committed-box AABBs, LBCP support polygons, and exact stack-center poses.
`placement_scoring.score_candidates` adds `proxy_score` (EMS regularity,
corridor, reachability atlas prior, …) plus `w_floor_first=0.60`.
EMS tiles stay axis-aligned empty cuboids for indexing; regularity and the
blocked-deep term clip those tiles to the seven-face hull.

`proxy_score` without that term is structurally stack-biased: a committed box
top scores high on support and observation confidence, an unobserved floor
scores 0. The floor-first term restores flatten-first for replay. On this replay path the reachability atlas is a **score prior**, not the
candidate generator. The live selector may also use it as a path prior.

Offline replay (`packing_replay.FreeSpaceModelStrategy`) and
`scripts/ros1_reference/placement_planner_node.py` use this path. Humble
`placement_planner_node.py` does not.
