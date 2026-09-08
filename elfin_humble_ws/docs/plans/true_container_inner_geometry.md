# True Container Inner Geometry Plan

Date: 2026-09-04

Status: approved by Cursor technical consensus and distinct Codex consensus.
The Git commit containing this approved text is the required reproducible plan
revision for subsequent mailbox dispatch.

Parent task: `TCIG-20260904`

Base source revision: `76742a7c7f8c2360a1af89370861d362fe61098b`

## Objective

Make every maintained online algorithm, offline policy, reachability model, and
acceptance metric interpret container space as the configured physical inner
hull. The AABB may remain an implementation detail for indexing, broad-phase
search, and array allocation, but it must never be reported or scored as usable,
free, reachable, blocked, covered, or placeable space without clipping it to the
inner hull.

The current supported non-box geometry is the convex seven-face container in
`scene_tf.yaml`: an axis-aligned inner cuboid with the lower `positive_y`
triangular prism removed. This task must retain the no-chamfer cuboid fallback.
It does not add arbitrary mesh, non-convex container, or platform-tilt support.

## Fixed Requirements

1. One ROS-free geometry kernel in `luggage_description` is the only authority
   for inner-hull containment, cross-sections, area, and volume.
2. Simulation and hardware execute the same algorithm functions. Geometry may
   come from a calibrated deployment YAML or a validated sensor estimate. Eval
   or Gazebo truth may establish references, but it must not enter online
   geometry through a second API.
3. An online component must fail closed on a missing, invalid, unsupported, or
   mismatched geometry descriptor. Silent fallback from a seven-face descriptor
   to `inner_size` is forbidden.
4. Existing rectangular configurations remain valid: omission of `chamfer`
   explicitly describes a cuboid and is not an error.
5. Margins inset every relevant hull plane consistently. A caller must not
   apply an XYZ margin and accidentally leave the slanted face uninset.
6. Placement feasibility is evaluated on the complete oriented box, not only
   its center or suction contact point.
7. Hull feasibility runs before candidate ranking, stratification, `top_n`, or
   first-feasible selection. A hull-invalid candidate cannot consume a retained
   candidate slot.
8. Grid cells outside the hull are inactive. Boundary cells carry clipped
   physical volume, not one full voxel merely because their center is inside.
9. Utilization and floor-coverage denominators are derived from the same
   geometry revision used by the algorithm under evaluation. Hard-coded scene
   dimensions are forbidden.
10. Every output/evidence record that reports geometry-derived metrics includes
    a stable geometry hash/version and the resolved usable volume/floor area.

## Reference Geometry

For the checked-in scene:

```text
length      = 1.49 m
width       = 1.97 m
floor_z     = 0.53 m
ceiling_z   = 2.01 m
floor_y     = 0.55 m
wall_y      = 0.985 m
wall_z      = 0.90 m
```

The exact expected values are:

```text
cuboid_volume = length * width * (ceiling_z - floor_z)
removed_volume = length * 0.5 * (wall_y - floor_y) * (wall_z - floor_z)
usable_volume = 4.22433625 m^3
usable_floor_area = length * (floor_y + width / 2) = 2.28715 m^2
```

These values are test fixtures, not runtime constants.

## Geometry Contract

### Plain descriptor

The geometry kernel consumes and emits a plain, serializable descriptor. The
minimum schema is:

```yaml
schema_version: 1
frame_id: container_link
length: 1.49
width: 1.97
floor_z: 0.53
ceiling_z: 2.01
chamfer:
  side: positive_y
  floor_y: 0.55
  wall_y: 0.985
  wall_z: 0.90
geometry_hash: <canonical-content-hash>
```

`geometry_hash` is computed from normalized numeric geometry, not source path,
YAML formatting, backend name, or timestamp. Algorithm modules consume plain
numbers and do not import ROS messages, TF, Gazebo, or evaluation helpers.

### Required operations

The kernel must provide tested equivalents of:

- normalized descriptor construction and validation;
- point and oriented-box containment with margin;
- `y_max(z)` and the usable YZ cross-section;
- exact hull volume and horizontal-floor area;
- exact intersection volume between the supported hull and an axis-aligned
  cell/EMS AABB;
- floor-support intersection for an axis-aligned footprint;
- payload-center Y intervals and YZ cross-sections after eroding the hull by
  the payload half extents over a Z range, using the footprint at payload yaw;
- swept-box hull containment for a piecewise-linear, fixed-orientation path.

For the current hull, exact AABB intersection can clip the AABB's YZ rectangle
against the convex YZ hull and multiply by the clipped X span. A generic mesh
boolean library is not required.

## Subtask Decomposition

The owner assignments below are proposed until consensus closes. No row is a
runnable mailbox assignment yet.

| ID | Proposed owner | Depends on | Exclusive primary files | Outcome |
|---|---|---|---|---|
| TCIG-1 | `codex/gpt-5` | none | `luggage_description/.../container_geometry.py`, `scene_tf_config_utils.py`, geometry tests and architecture contract | Canonical exact seven-face geometry kernel |
| TCIG-2 | `codex/gpt-5` | TCIG-1 | `cargo_volume_mapper.py`, mapper node, mapper tests | Hull-weighted map statistics and geometry-bearing surface contract |
| TCIG-3 | `codex/gpt-5` | TCIG-1,TCIG-2 | `placement_solver.py`, ROS 2 `placement_planner_node.py`, placement tests | Hull-aware candidates before retention and final safety gate |
| TCIG-4 | `cursor/grok-4.6` | TCIG-1 | `insertion_corridor.py`, `corridor_audit.py`, existing ROS-free `waypoint_generator.py`, ROS 2 waypoint node, corridor tests | Hull-eroded insertion corridor and swept-payload audit |
| TCIG-5 | `cursor/grok-4.6` | TCIG-1 | new `geometry_metrics.py`, packing/bringup/gazebo metric entry points except `packing_replay.py`, metric tests | One exact geometry-derived denominator path for volume/floor metrics |
| TCIG-6 | `cursor/grok-4.6` | TCIG-1,TCIG-2,TCIG-4,TCIG-5,TCIG-7 | `ems.py`, `free_space_model.py`, `value_estimator.py`, `packing_replay.py`, offline tests | Hull-aware offline policy and replay; no rectangular comparison bias |
| TCIG-7 | `cursor/grok-4.6` | TCIG-1 | reachability atlas, `interior_view_scorer.py`, versioned opening geometry message/adapters/consumer tests | Hull-masked reachable/observable space and representable runtime geometry |
| TCIG-INTEGRATION | `codex/gpt-5` | TCIG-1..TCIG-7 | cross-package integration only | Resolve integration failures and run complete E2E acceptance |

Cursor tasks are independent in file ownership after TCIG-1, but runnable rows
must be dispatched one at a time to the same `cursor/grok-4.6` session. That
session already owns queued PF-R7, so TCIG work waits for PF-R7 or for an
explicitly registered second Cursor session. With one session, dispatch TCIG-4
and TCIG-5 sequentially after TCIG-1, then TCIG-7, then TCIG-6 after TCIG-2 and
all of TCIG-4/5/7. TCIG-7 reads the geometry kernel but does not edit it. The
dependency graph still permits Codex TCIG-2/3 work to continue independently.

## TCIG-1 - Canonical Geometry Kernel

### Scope

- Add a ROS-free geometry module under `luggage_description`.
- Move normalized hull calculations behind that module; preserve compatibility
  wrappers in `scene_tf_config_utils.py` where existing callers need them.
- Add `docs/architecture/container_geometry.md` and index it from architecture
  README. Add/update a machine-facing rule only if a new must-rule needs one.
- Reject invalid dimensions, non-finite numbers, unsupported chamfer sides,
  `wall_z <= floor_z`, cuts outside the AABB, and empty margin-eroded hulls.
- Preserve a cuboid descriptor when no chamfer exists.

### Details to protect

- Coordinate frame is `container_link`; mapper-local Z conversion remains an
  explicit adapter, not an implicit second geometry convention.
- Oriented-box containment checks all eight physical corners. It must support
  at least the current 0/90-degree yaws and remain correct for arbitrary yaw.
- Hash equality means geometry equality after normalization.
- Exact area/volume routines must not depend on voxel resolution.
- The old point predicate must delegate to the new kernel so two formulas
  cannot drift.

### Gate G1

- Checked-in seven-face volume equals `4.22433625 m^3` within `1e-9`.
- Floor area equals `2.28715 m^2` within `1e-9`.
- Cuboid fallback equals `L*W*H` and `L*W`.
- Points and boxes on each face, just inside, and just outside behave correctly.
- Margin insets the slanted face and all axis-aligned faces.
- AABB/hull intersection volumes cover disjoint, full, and boundary-crossing
  fixtures and sum to exact hull volume over a tiled AABB.
- Plain `pytest` imports the module without ROS.

Failure blocks every implementation subtask.

## TCIG-2 - Cargo Map and Surface Contract

### Scope

- Replace center-only `_active` semantics with per-voxel hull intersection
  weights. A zero-weight cell is inactive.
- Keep AABB `nx/ny/nz` indexing and coordinate transforms.
- Make `stats()` weight FREE/UNKNOWN/OCCUPIED by clipped physical volume.
- Extend `surface_map_2d()` with normalized geometry descriptor/hash and enough
  per-column information to distinguish true floor support from space that only
  exists above the chamfer.
- Make the floor-prior producer and consumer use the same surface semantics.
- Keep count fields as diagnostic cell counts; do not use them as physical
  volume denominators.

### Details to protect

- `ceil(inner/resolution)` creates partial edge cells; clip cell bounds to the
  AABB before hull clipping.
- Ray integration and placed-box rasterization must not reactivate a zero-weight
  cell.
- `free_volume + occupied_volume + unknown_volume` equals usable hull volume
  within numeric tolerance.
- Surface height remains floor-relative and carries an explicit frame/version.
- A geometry hash mismatch in a consumer fails closed rather than mixing maps.

### Gate G2

- Empty map at multiple non-divisor resolutions reports the same exact usable
  volume within `1e-8`.
- The chamfer wedge contributes no active volume or floor support.
- Boundary voxels contribute fractional volume.
- Fill-as-free reports full hull free volume, not AABB volume.
- Cuboid maps retain existing behavior.
- Mapper package regression passes.

Failure blocks TCIG-3 and TCIG-6.

## TCIG-3 - Online Placement

### Scope

- Extend `generate_candidates()` to accept the normalized geometry descriptor
  and apply oriented-box containment before sorting and retention.
- Compute physical boundary clearance against the hull, while retaining top
  clearance as a separate diagnostic.
- Update `_floor_prior_surface()` to create prior support only on the real
  horizontal floor.
- Use the TCIG-1 payload-yaw erosion API for boundary clearance and feasible
  center intervals; do not derive another fixed-width approximation.
- Replace node-private hull math with the shared box-containment operation.
- Keep a final node-level hull gate after candidate generation as defense in
  depth, but treat its rejection as an invariant violation in diagnostics.

### Details to protect

- The full footprint and full box height matter; center-inside is insufficient.
- Hull-invalid high-score candidates cannot consume `top_n`.
- Rejected-candidate visualization is retained separately from the feasible
  pool and cannot change feasibility ordering.
- Existing aperture and support gates remain independent and keep stable reason
  codes.
- Missing or mismatched geometry is a service failure, not `BIN_FULL`.
- Genuine no-candidate after all geometry gates remains `BIN_FULL`.

### Gate G3

- A fixture with more than `top_n` high-score wedge candidates still returns a
  lower-score valid hull candidate.
- Every returned feasible candidate's eight box corners are inside the inset
  hull.
- Floor candidates never occupy the removed floor wedge.
- Cuboid fixtures preserve deterministic ordering.
- Service diagnostics distinguish geometry-invalid from true bin exhaustion.
- Placement solver and ROS-node focused tests pass.

Failure blocks integration.

## TCIG-4 - Insertion Corridor and Swept Payload

### Scope

- Keep corridor AABBs as broad-phase structures only.
- Derive the payload-center lateral interval from the hull cross-section eroded
  by the payload dimensions at the relevant carry Z range.
- Make `corridor_blocked()` compare wall coverage with the physically available
  interval, not fixed `[-inner_w/2,+inner_w/2]`.
- Make `blocks_deep_space()` use clipped EMS/hull volume.
- Add a narrow-phase swept-box hull check shared by planner audit and waypoint
  generation.
- Update the existing reusable, ROS-free waypoint/corridor algorithm in
  `src/luggage_planning/luggage_planning/waypoint_generator.py`; keep the ROS 2
  node as a thin conversion/call layer.
- Do not expand `scripts/ros1_reference/waypoint_generator_node.py`; it remains
  reference-only and outside the production implementation.

### Details to protect

- Opening side remains `negative_x`; adding new opening orientations is out of
  scope, but unsupported values must fail explicitly.
- The chamfer is extruded along X, so containment along a fixed-Y/Z horizontal
  segment can be handled analytically; descent segments still need endpoint and
  face-crossing checks.
- Hull erosion uses the carried payload footprint at its actual yaw. It must
  not compare a catalog width with the fixed nominal container width.
- Do not convert the current single-box wall simplification into a multi-box
  path planner in this task.
- `corridor_audit.py` and `insertion_corridor.py` must call the same geometry
  operation so planning and verification cannot disagree.

### Gate G4

- A low-Z wall spanning the true eroded width blocks even when it does not span
  nominal AABB width.
- A box that only overlaps the broad-phase AABB but not usable hull does not
  create a false obstacle.
- Swept payload crossing the slanted face is rejected before motion planning.
- Safe center, side, and stacked corridors remain accepted.
- Existing obstacle and carry-height tests remain green.

Failure blocks integration but not TCIG-2, TCIG-3, or TCIG-5.

## TCIG-5 - Metrics and Evidence

### Scope

Remove hard-coded rectangular volume/floor constants from:

- a new ROS-free `luggage_packing/geometry_metrics.py` helper, which is the
  only metric-side adapter from a kernel descriptor to denominators;
- `luggage_gazebo/scripts/pack_eval_driver.py`;
- `luggage_bringup/scripts/active_loading_bag_harness.py`;
- `luggage_bringup/scripts/multi_box_gazebo_matrix.py`;
- `luggage_packing/scripts/packing_replay_eval.py`.

`packing_replay.py` is exclusively owned by TCIG-6. It calls the helper added
by TCIG-5 but is not edited in this subtask.

Every metric entry point resolves a geometry descriptor or receives explicit
geometry-derived values with hash/version. It must not silently retain old
defaults when the descriptor is present but invalid.

### Details to protect

- `volume_fraction = packed box volume / exact usable hull volume`.
- `floor_coverage` includes floor-contact boxes only and divides by exact usable
  horizontal floor area.
- Assert floor footprints do not overlap before summing them, or compute their
  union. Do not double-count overlapping evidence.
- Reports include both numerator and denominator for auditability.
- Historical evidence is not rewritten; schema/version identifies the new
  metric semantics.
- Eval-only identity and box-reference inputs, including an existing simulation
  `GetCurrentBox` call, may still establish packed-volume numerators. They must
  never establish usable-volume/floor denominators or feed online algorithms.

### Gate G5

- Checked-in scene reports `inner_volume_m3=4.22433625` before output rounding.
- A single known box produces analytically expected volume fraction.
- Full real-floor fixture reports floor coverage 1.0; the removed wedge does
  not appear in the denominator.
- Invalid/missing geometry fails with a stable reason.
- Metric tests prove there are no remaining active hard-coded `4.344` or
  `1.49*1.97` denominators.
- G5 covers `volume_fraction` and `floor_coverage`; reachable-volume metrics are
  explicitly owned and accepted by G7.

Failure invalidates utilization claims but does not block online development.

## TCIG-6 - Offline Policy, EMS, and Replay

### Scope

- Let EMS retain AABBs as a decomposition structure, but define usable space as
  `EMS AABB intersect hull` for usefulness, volume, and regularity.
- Initialize `FreeSpaceModel.lbcp` from the real horizontal floor.
- Apply oriented-box hull containment before FreeSpaceModel stratification.
- Propagate geometry through `ProxyStrategy`, `FreeSpaceModelStrategy`, rollout
  snapshots, and replay simulators.
- Update the unported BinPacker reference and its pure replay equivalent, or
  explicitly retire them from accepted comparisons. No maintained rectangular
  fallback path may be labelled geometry-feasible.

### Details to protect

- Generic `volume(aabb)` may keep its mathematical meaning; callers requiring
  usable volume must call the hull-clipped operation explicitly.
- `EMS.regularity()` normalizes against exact hull volume.
- TCIG-6 does not edit `insertion_corridor.py` or `blocks_deep_space()`; those
  are exclusively owned by TCIG-4. EMS callers use the TCIG-1 AABB/hull
  intersection operation for useful volume.
- Rollout snapshots preserve immutable geometry identity.
- Online and offline candidate predicates are the same function.

### Gate G6

- Initial EMS usable volume equals exact hull volume.
- EMS spaces wholly in the wedge contribute zero; crossing spaces contribute
  exact clipped volume.
- Offline candidates never leave the hull and do not lose valid candidates at
  pool truncation.
- Replay `V_container`, fill rates, and floor coverage use exact geometry.
- Cuboid baseline tests remain unchanged within existing tolerances.

Failure blocks using replay to tune or compare the online policy.

## TCIG-7 - Reachability, Exploration, and Runtime Interface

### Scope

- In reachability-atlas generation, mark hull-invalid contact/payload cells
  inactive before IK work and exclude them from total/rate statistics.
- Validate the complete nominal payload against the hull, not only its suction
  contact point.
- Persist geometry descriptor/hash and active mask in atlas artifacts.
- Make runtime atlas queries fail closed on geometry mismatch or hull-invalid
  payloads.
- Make `ContainerFloorModel` count and score only the true horizontal floor.
- Replace AABB-only exploration debug bounds with the real hull wireframe.
- Extend or version the runtime container-geometry interface so a chamfer is
  representable; `inner_size` alone is not a complete geometry contract.

### Details to protect

- Atlas array allocation may remain rectangular.
- `reachable_volume()` counts the union of reachable active cells and clips
  boundary-cell volumes.
- ROS message conversion stays in node/adapter code; the geometry kernel stays
  ROS-free.
- TCIG-7 may version `ContainerOpeningEstimate` and edit its adapters/consumers,
  but it must persist exactly the TCIG-1 `schema_version`, normalized descriptor,
  and hash. It must not edit `container_geometry.py` or
  `scene_tf_config_utils.py`.
- Existing ROS 1 files not installed by CMake are reference code. Do not expand
  them as production implementation; either port required behavior to ROS 2 or
  document the reference-only status.
- A sensed geometry update increments geometry version and invalidates maps,
  atlases, and candidates built for a different hash.

### Gate G7

- Atlas build skips hull-invalid cells without invoking IK.
- Reachability rates exclude inactive cells and are resolution-stable within
  one clipped boundary-cell tolerance.
- `reachable_volume_ratio` uses clipped active-hull reachable volume divided by
  the exact hull volume and is accepted here, not in G5.
- A contact point inside the hull with a payload crossing the slanted face is
  rejected.
- Floor-view coverage denominator equals the exact active floor-cell area/count.
- Cuboid and seven-face geometry round-trip through the runtime interface.
- Planning tests and message build pass.

Failure blocks claims about reachable capacity and interior coverage.

## Integration - TCIG-INTEGRATION

The integration owner rebases/merges exact passing revisions, resolves contract
drift, and runs all gates on one revision. It does not weaken a focused gate to
make integration pass.

### Integration acceptance

1. `G1` through `G7` pass on exact revisions and again after integration.
2. Complete ROS-free package tests pass for description, perception, packing,
   and planning.
3. `colcon test` passes for affected ROS 2 packages and generated interfaces.
4. Headless closed-loop packing reaches a legitimate terminal result with:
   zero selected hull-invalid boxes, zero post-selection `outside_hull`
   invariant violations, no geometry-version mismatch, and no false
   `BIN_FULL` caused by candidate retention.
5. Suite output reports the checked-in geometry hash, usable volume
   `4.22433625 m^3`, and floor area `2.28715 m^2` before presentation rounding.
6. A cuboid-config regression preserves prior valid behavior.
7. A malformed or unsupported hull fails closed before mapping/planning.
8. Static audit shows online algorithm modules import no Gazebo/eval truth and
   every geometry source has a hardware provider (`calibrated` or `sensed`).
9. Simulation started by an agent is stopped with `scripts/stop_sim.sh`; no
   residual ROS/Gazebo process remains.

Evidence belongs under:

```text
docs/status/evidence/true_container_inner_geometry/<revision>/<gate>/
```

Each subtask owner records commands, exact revision, results, and evidence
pointers in its role note and closes its own failures before reporting pass.

## Dispatch Rules

1. Cursor reviews this plan in one mailbox thread and either records
   `cursor_consensus: reached` or lists concrete amendments.
2. Reviews incorporates amendments and obtains the workflow-required consensus
   from a distinct Codex agent.
3. The approved plan is committed or otherwise given one reproducible revision.
4. Only then are runnable `subtask` rows created for ready tasks. Do not create
   parallel runnable rows for one concrete Cursor session/model; PF-R7 retains
   priority unless another Cursor session is explicitly registered.
5. A dependent task may be listed but cannot be claimed before all dependency
   threads contain a passing Result event.
6. TCIG-INTEGRATION is dispatched only after TCIG-1 through TCIG-7 pass.

## Cursor Technical Consensus

Reached 2026-09-04 by `cursor/grok-4.6`. Distinct Codex consensus also reached
2026-09-04 by `codex-tcig-consensus/gpt-5`. The commit containing this approved
text is the reproducible plan revision used for dispatch.

1. The seven-face half-space/YZ-clipping kernel is sufficient. No generic mesh
   boolean.
2. Exclusive file boundaries for TCIG-4 through TCIG-7 are free of hidden
   writes to TCIG-1/TCIG-2 files when the exclusive lists are followed.
   `waypoint_generator.py` already exists; TCIG-4 updates it. Swept-box
   containment is a TCIG-1 primitive called by TCIG-4.
3. The plain descriptor/hash is enough for calibrated YAML and a future sensed
   update if TCIG-1 is the only serializer. `inner_size` alone is not.
4. No remaining underspecification that should fork online vs offline
   predicates: both call TCIG-1 oriented-box containment. G5 does not accept
   `reachable_volume_ratio`.
5. Cursor accepts the proposed ownership and serial `cursor/grok-4.6` dispatch
   order, with PF-R7 priority, subject to exact approved base revisions at
   dispatch time.
