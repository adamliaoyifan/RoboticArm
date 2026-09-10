# Independent ready execution wave

Date: 2026-09-10

Status: approved execution overlay after distinct-Codex consensus. It narrows
four already approved subtasks to the current codebase; it does not supersede
their parent architecture or authorize any dependent/integration task.

Execution base: `0d24f18f4fd5262ca9206bf1f8613210af49d7dc`.

Consensus:
`docs/agents/discuss/2026-09-10_1518_independent-ready-wave-consensus.md`.

## Objective

Execute only work whose declared prerequisites already have passing Results
and whose primary files do not overlap the active PF-R10 generation-3/D555
simulation repair. Keep every owner responsible for implementation, focused
and package tests, failure repair, evidence, commit, role note, and thread
closure.

## Resolved prerequisites

- TCIG-1 passed at `7af40220c9e86feb7f56908d9fcde389aa48dd9d`.
- SIM-R1-1 passed after closure repair at
  `713cfbb6715073d81c2fc6cf22f7ec35ffc6a123`.

TCIG-4 passed and was reaccepted at
`10a93e898d618bc30144189d5e344992706ddf04`, but that commit is on its isolated
branch and is not an ancestor of this wave's base. No task in this wave depends
on TCIG-4 or may assume its code is present. A later TCIG integration owner
must merge the accepted TCIG-4 revision explicitly.

No task in this wave depends on PF-R10, PF-R7, DSIM, a physical calibration
board, a hand-eye result, or another task in this wave. Owners use isolated
worktrees and do not edit active PF-R10 files.

For mailbox scheduling, the accepted TCIG-1 and SIM-R1-1 results are frozen
source inputs already contained in the execution base, not unfinished work.
Replacement runnable generations therefore encode `depends_on: none`. This is
only a lifecycle-metadata normalization for legacy generationless prerequisite
threads; it does not relax either accepted architecture contract or permit an
owner to replace the pinned prerequisite implementations.

## Task table

| ID | Owner | Complexity | Primary scope | Passing outcome |
|---|---|---|---|---|
| TCIG-5 | `eng/cursor-grok-b/grok-4.6` | low | exact metric helper plus bounded metric-entry migration | Online/eval entry points use exact G5 denominators; replay volume is corrected and TCIG-6-owned floor/reachability fields are explicitly non-authoritative |
| TCIG-2 | `eng/codex-tcig-map-eng/gpt-5.6-sol` | high | weighted cargo-map core, ROS 2 mapper adapter, map tests | Fractional hull voxels and geometry-bearing surface contract pass G2 |
| TCIG-7 | `eng/codex-tcig-atlas-eng/gpt-5.6-sol` | high | atlas active mask/identity, runtime opening geometry, scorer/tests | Atlas and runtime queries are hull-aware and fail closed on identity mismatch |
| SIM-R1-5 | `eng/codex-sim-r1-eng/gpt-5.6-sol` | high | ROS 2 bringup migration, pure production state machine adapter, operator Start | Production launch is motionless before explicit correlated Start and all failures are observable |

## TCIG-5 -- exact metric denominators

### Allowed changes

- Add `src/luggage_packing/luggage_packing/geometry_metrics.py` as the only
  metric adapter from a normalized TCIG-1 descriptor to exact usable volume
  and floor area.
- Migrate:
  - `src/luggage_gazebo/scripts/pack_eval_driver.py`;
  - `src/luggage_bringup/scripts/active_loading_bag_harness.py`;
  - `src/luggage_bringup/scripts/multi_box_gazebo_matrix.py`;
  - `src/luggage_packing/scripts/packing_replay_eval.py`.
- Add focused tests under `src/luggage_packing/test/` and static audits for
  maintained metric entry points.

Do not edit `packing_replay.py` (TCIG-6), the geometry kernel (TCIG-1), the
mapper (TCIG-2), placement logic, simulator launch, or current evidence.

### Checkpoints and acceptance

1. The helper accepts only a normalized/valid descriptor and returns exact
   numerator/denominator fields, schema version, and geometry hash. Missing,
   malformed, unsupported, non-finite, or hash-mismatched geometry fails with
   a stable reason; it never falls back to rectangular defaults.
2. Checked-in seven-face geometry reports volume `4.22433625 m^3` and floor
   area `2.28715 m^2` before presentation rounding; cuboid fallback reports
   `L*W*H` and `L*W`.
3. `volume_fraction` uses packed-box volume divided by exact hull volume.
   `floor_coverage` includes a box only when its bottom is within a documented
   deterministic tolerance of the real TCIG floor, clips its footprint to the
   exact floor polygon, and uses footprint union. If an entry point cannot
   compute union, overlapping footprints fail with stable reason
   `OVERLAPPING_FLOOR_FOOTPRINTS`; summing overlap twice is forbidden.
4. Reports contain numerator, denominator, schema version, and hash. Historical
   artifacts are not rewritten.
5. `packing_replay_eval.py` uses its available `V_placed` numerator to
   recompute authoritative `volume_fraction` with the exact hull volume.
   Existing `floor_coverage`, `V_container`, `overall_fill_rate`,
   `reachable_volume_ratio`, and `reachable_fill_rate` fields originating in
   TCIG-6-owned `packing_replay.py` are labelled `legacy_non_authoritative`
   and cannot satisfy a G5 floor or reachability claim. Authoritative replay
   floor coverage is deferred to TCIG-6; `packing_replay.py` remains
   unmodified. The other three migrated entry points still produce
   authoritative G5 floor metrics from their box records.
6. A static scan shows no active hard-coded `4.344`, `1.49*1.97`, or equivalent
   rectangular usable-volume/floor denominator in the four migrated entry
   points.

Required verification:

```bash
python3 -m pytest src/luggage_description/test -q
python3 -m pytest src/luggage_packing/test -q
python3 -m pytest src/luggage_gazebo/test -q
python3 -m pytest src/luggage_bringup/test -q
colcon build --packages-select luggage_description luggage_packing luggage_gazebo --symlink-install
colcon test --packages-select luggage_description luggage_packing luggage_gazebo --event-handlers console_direct+
colcon test-result --verbose
git diff --check
scripts/check_agent_contract.sh
```

`luggage_bringup` is still a catkin/COLCON_IGNORE package until SIM-R1-5, so
TCIG-5 must add ROS-independent direct tests for both edited bringup scripts;
it must not remove COLCON_IGNORE or claim a bringup `colcon` test. No Gazebo
runtime is required; fixture-level report generation must cover every migrated
entry point.

## TCIG-2 -- weighted cargo map and surface contract

### Allowed changes

- `src/luggage_perception/luggage_perception/cargo_volume_mapper.py`;
- `src/luggage_perception/scripts/cargo_volume_mapper_node.py`;
- mapper and surface-contract tests in `src/luggage_perception/test/` and
  `src/luggage_packing/test/test_mapper_surface_contract.py`;
- a new pure surface-schema validator owned by TCIG-2; and
- `src/luggage_msgs/srv/GetCargoMapStats.srv` plus focused message/adapter
  tests for explicit geometry identity and physical-volume fields.

Do not edit TCIG-1 geometry functions, placement candidate logic, exploration,
metric entry points, or PF-R10 files.

### Checkpoints and acceptance

1. Resolve one normalized TCIG-1 geometry descriptor/hash at construction.
   Replace center-only boolean activation with exact per-cell hull intersection
   weights after clipping partial edge cells to the allocation AABB.
2. FREE/UNKNOWN/OCCUPIED physical volumes are weight sums and add to exact
   usable hull volume within `1e-8` at multiple non-divisor resolutions.
   Diagnostic integer counts remain counts and are never denominators.
3. Zero-weight cells cannot be activated by ray integration or placed-box
   rasterization. Boundary voxels contribute fractional volume.
4. `surface_map_2d()` carries normalized descriptor, schema version, geometry
   hash, explicit frame/floor semantics, and column support information that
   distinguishes real floor from space above the chamfer. The new pure
   validator rejects missing/malformed schema and hash mismatch; TCIG-3 later
   owns wiring that validator into placement consumption.
5. Seven-face wedge cells contribute no floor support or usable volume;
   cuboid behavior remains compatible.
6. `GetCargoMapStats` exposes schema version, geometry hash, usable volume,
   and FREE/UNKNOWN/OCCUPIED physical volumes as typed fields. The node must
   not hide identity or new physical values inside `message`.

Required verification:

```bash
python3 -m pytest src/luggage_description/test -q
python3 -m pytest src/luggage_perception/test -q
python3 -m pytest src/luggage_packing/test/test_mapper_surface_contract.py -q
colcon build --packages-select luggage_msgs luggage_description luggage_perception luggage_packing --symlink-install
colcon test --packages-select luggage_msgs luggage_description luggage_perception luggage_packing --event-handlers console_direct+
colcon test-result --verbose
git diff --check
scripts/check_agent_contract.sh
```

## TCIG-7 -- hull-aware reachability atlas and runtime identity

### Allowed changes

- maintained ROS-free reachability/atlas and interior-view modules in
  `src/luggage_planning/luggage_planning/`;
- a new ROS-free atlas-builder kernel with an injected IK callback and a thin
  ROS 2 adapter in maintained `src/luggage_planning` code;
- `ContainerOpeningEstimate` or a versioned replacement plus its maintained
  ROS 2 adapters/consumers;
- a deterministic atlas migration utility for the four checked-in
  `s20_container_collision_aware*.npz/.yaml` artifact pairs;
- necessary `src/luggage_planning/CMakeLists.txt`, `package.xml`, setup, and
  install-manifest changes for the new ROS 2 adapter and migration tool;
- focused planning/message tests and compact generated fixture metadata.

The current rospy `scripts/reachability_atlas_builder.py` is classified and
moved unchanged under `scripts/ros1_reference/`; do not extend it. Do not edit
the TCIG-1 kernel, cargo mapper, placement solver, simulator launch, production
orchestrator, or other ROS 1 reference logic.

The current rospy `cargo_exploration_planner_node.py` is likewise
reference-only and is not a maintained ROS 2 geometry consumer. TCIG-7 adds
and tests a ROS-free message adapter plus identity validator and the new thin
ROS 2 atlas-builder adapter. Wiring the future ROS 2 `PlanNextCargoView`
consumer belongs to SIM-R1-4 and is not part of this task.

### Checkpoints and acceptance

1. Atlas allocation may remain rectangular, but hull-invalid contact/payload
   cells are masked before IK and excluded from totals and rate denominators.
   The complete nominal payload, not only its contact point, must pass TCIG-1
   oriented-box containment.
2. Atlas artifacts persist normalized descriptor, schema version, geometry
   hash, active mask, payload dimensions/yaw convention, and builder revision.
   Runtime load/query rejects missing or mismatched identity and hull-invalid
   payloads with stable reasons. Existing schema-v2 artifacts are never
   silently assigned an identity: the migration utility deterministically
   recomputes their active masks from their stored grid/profile plus the
   checked-in geometry, writes provenance and the new hash, and refuses inputs
   that lack enough provenance.
3. Reachable volume is the clipped union of reachable active cells divided by
   exact hull volume. Resolution variation stays within one clipped
   boundary-cell tolerance.
4. Floor-view coverage uses the real horizontal floor only. Debug geometry
   renders the seven-face hull rather than an AABB presented as usable space.
5. The runtime opening geometry round-trips both cuboid and seven-face
   descriptors/hashes. A sensed geometry-version change invalidates atlas
   queries, candidates, and exploration snapshots through explicit validator
   hooks. Cargo-map invalidation belongs to TCIG-2/later integration and is
   not implemented here.

Required verification:

```bash
python3 -m pytest src/luggage_description/test -q
python3 -m pytest src/luggage_planning/test -q
colcon build --packages-select luggage_msgs luggage_description luggage_planning --symlink-install
colcon test --packages-select luggage_msgs luggage_description luggage_planning --event-handlers console_direct+
colcon test-result --verbose
git diff --check
scripts/check_agent_contract.sh
```

No motion or full atlas rebuild is required for intermediate checkpoints. Final
evidence must include a deterministic small-grid atlas fixture proving
hull-invalid cells skip the IK callback entirely.

## SIM-R1-5 -- ROS 2 production state machine and operator Start

### Allowed changes

- convert `src/luggage_bringup` from catkin/COLCON_IGNORE to the repository's
  ROS 2 ament pattern;
- move the current ROS 1 orchestrator to `scripts/ros1_reference/` unchanged;
- add a thin rclpy production orchestrator and operator-control adapter around
  the accepted ROS-free contracts from SIM-R1-1;
- focused pure and node/launch tests for R5.

Do not implement exploration policy/coordinator work, cargo-map integration,
hardware calibration, simulator-specific spawn/GT behavior, or edit PF-R10,
TCIG, DSIM, motion-planning, and perception algorithms. Do not change accepted
SIM-R1-1 message/effect schemas unless a failing compatibility test proves an
unavoidable defect; report such a defect blocked rather than widening scope.

### Checkpoints and acceptance

1. The installed production executable is rclpy and thin; the old rospy file
   is reference-only and not installed. Pure transition logic remains in the
   accepted ROS-free module.
2. Launching/configuring/activating/readiness/timers cannot synthesize Start.
   For a 60-second isolated ROS graph with no Start there are zero motion
   action goals and zero vacuum commands.
3. A visible operator surface sends one explicit `start`. Duplicate Start is
   rejected/idempotent. Wrong, stale, duplicate, malformed, or wrong-session
   pickup-ready IDs produce no detection or motion effect.
4. After a correlated Start, the node emits only the declarative initial
   exploration action effect; this task may use a test action server to inspect
   effects but may not add a fake backend selectable by production launch.
5. Pre-pick failure returns to a non-moving observable state. Carry fault
   preserves vacuum and never increments placed count. Placement commit is
   idempotent and a retry cannot double-count a box. Prompt and status QoS
   match the accepted contract and every failure has a stable reason.
6. Static audit finds no production imports or calls for spawn, clear box,
   Gazebo model state, eval GT, `sim_mode`, or fake backend.

Required verification:

```bash
python3 -m pytest src/luggage_planning/test/test_sim_r1_contracts.py -q
python3 -m pytest src/luggage_bringup/test -q
colcon build --packages-select luggage_msgs luggage_planning luggage_bringup --symlink-install
colcon test --packages-select luggage_msgs luggage_planning luggage_bringup --event-handlers console_direct+
colcon test-result --verbose
git diff --check
scripts/check_agent_contract.sh
```

The owner must add `src/luggage_bringup/test/` if absent. Full exploration,
three-box simulation, and live hardware are explicitly deferred. The 60-second
no-Start check may use an isolated ROS graph with test-only action servers. If
it starts Gazebo, it must wait until PF-R10 releases the simulator lock and
must follow `scripts/stop_sim.sh` with zero residual processes; implementation
and all non-Gazebo tests remain immediately executable.

## Isolation and closeout

- The committed revision containing this plan and its consensus record is both
  the `revision` and `plan_revision` for every runnable row. Each task uses a
  separate worktree/branch from that exact clean documentation-only descendant
  of `0001c41`. Primary-workspace mailbox updates use `AGENT_COORD_ROOT`.
- No owner merges another wave task. Integration/rebase belongs to later
  higher-generation or integration work.
- Evidence paths:
  - `docs/status/evidence/true_container_inner_geometry/<revision>/g2/`;
  - `docs/status/evidence/true_container_inner_geometry/<revision>/g5/`;
  - `docs/status/evidence/true_container_inner_geometry/<revision>/g7/`;
  - `docs/status/evidence/sim_r1/<revision>/r5/`.
- A passing Result names one clean tested commit, commands/results, dirty count,
  and evidence. Any unavailable required interface or contradictory current
  architecture yields `outcome: blocked`; thresholds are not weakened.

## Deferred work

Do not dispatch SIM-R1-2/3/4/6, either integration task, TCIG-3/6, LRF-A1,
LRF-PL1, or Todo 5 B4/B5 from this overlay. Their external baselines or
same-parent dependencies are not yet ready.

## Pointers

- `docs/plans/true_container_inner_geometry.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/agents/reviews/2026-09-10_1449_undispatched-plan-rebaseline.md`
- `docs/agents/discuss/2026-09-04_2021_exp-a1-nbv-readiness-audit.md`
- `docs/architecture/container_geometry.md`
- `docs/architecture/production_orchestration.md`
- `docs/architecture/sensor_data_pipeline.md`
