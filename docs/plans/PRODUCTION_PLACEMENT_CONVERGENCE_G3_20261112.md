# 2026-11-12 -- Placement and complete-operation production convergence, G3 discussion

- status: draft
- dispatch_ready: no
- related_parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- related_subtask: ST-4-INTEGRATION
- base_revision: dbe3157b
- owner: codex/gpt-6
- scope: Follow-up design and source/install inventory; no production behavior changes in this document.

## Confirmed requirements

1. Admit all directed box headings 0/90/180/270 degrees. A valid quarter-turn
   placement counts as available space. A bounded failed search is not proof of no space.
2. ComputePlacement and complete-operation preparation share candidate generation,
   map evidence, admission rules and configuration; no second simplified planner.
3. O is the default map owner. G is an explicit alternative. Exactly one producer
   starts in the integrated launch; both expose the same consumer semantics.
4. Every consumer inherits the selected owner's resolution, grid origin, dimensions,
   hull and height datum. No consumer-side 2 cm override or silent regridding.
5. Complete-operation preparation/execution becomes the production default after
   integration acceptance. Existing legacy protocols remain explicit compatibility
   adapters, without silent fallback after a complete-operation failure.
6. Final objective is a successful whole chain using the actual complete robot,
   tool, wrist sensors, payload geometry and deployment scene. Component tests and
   the cup-sphere release fixture are not this baseline.
7. Previous decisions remain: complete cup containment; preseal grasp reselection;
   fixed grasp after seal; measured occupancy; final side contact permitted without
   penetration/pushing; release method/height and separate 5 cm policies under review.
8. Baseline order confirmed by the user: complete-model simulation first, then
   collect real ULD observations and complete the measured real baseline. Simulation
   perception/maps must still come from simulated sensor observations; Gazebo truth
   is isolated for scoring and diagnosis, not fed as production box/map geometry.

## Observed source branches, not an asserted live deployment

No matching named runtime process was observed in the bounded process inspection.
The current shell has no AMENT_PREFIX_PATH/COLCON_PREFIX_PATH/PYTHONPATH. Therefore
the following is an inventory of declared entry points, not a claim about a live
ROS graph, an operator's sourced terminal, or a deployed machine.

| Entry | Current behavior | Consequence |
|---|---|---|
| `hardware_pick.launch.py` | Placement/prior are opt-in; placement providers come from `hardware_placement_launch.placement_nodes`. Driver is invoked separately. | Launch readiness does not run a full packing cycle. |
| `hardware_placement_launch.py` | Starts G `cargo_volume_mapper`, placement planner and selector. | It does not start O, although ComputePlacement defaults to O. |
| `hardware_pick_driver.py` | Optional read-only prior; PickSession uses segment actions, typed mode uses PlanMotionV2. Optional post-pick place planning uses actual attachment, ComputePlacement and selector. | Not a pre-pick bound complete-operation client; post-pick place planning explicitly leaves execution separate. |
| `joint_operation_prepare.launch.py` | Independent plan-only root plus native endpoint/connection/certification services; requires model/config bindings. | Not included in ordinary hardware placement launch. |
| `joint_operation_prepare_node._placements` | Internal enumerate helper, then deep + shallow selection, then half-turn expansion. | Separate candidate policy from ComputePlacement; no quarter-turn footprint generation by default. |
| `motion_planner_node.py` | `enable_joint_operation=False`; opt-in constructs complete-operation action server. | Complete-operation execution is not the default route. |
| Production orchestration contract | Defines operator start, exploration, measurement, cycle and complete-plan consumer requirements. | Contract and pure state code do not establish a wired production ROS cycle. |

Actual source references are in the hashed inventory and the preceding audit:
`docs/reviews/PLACEMENT_PLANNER_DESIGN_AUDIT_20261112.md`.

## Missing module root cause

Source and CMake installation declarations exist for the new prepare executable,
Python modules and native connection service. Standard build CMakeCache records
`AMENT_CMAKE_SYMLINK_INSTALL=OFF`. Its generated install script predates the current
CMakeLists: November 10 versus November 11. It lacks the newer declarations.
Meanwhile `install/.../motion_planner_node.py` is still an older source symlink.
New module/runtime-port and prepare executable are absent in standard prefixes.

This directly establishes stale/mixed build/install state, not missing source.
It does not identify which historical colcon command last ran or which overlay
an outside terminal uses. `/tmp/joint_operation_native/install` contains some newer
native/prepare artifacts but is itself incomplete for current runtime ports.
`install-mo` has a copied, different motion executable. None is a qualified default.

The correction must establish one named production prefix and launch command,
reconfigure/build/install affected packages and dependencies there, then test from
a clean shell. Existing symlinks do not automatically install added files, updated
CMake declarations, generated messages or C++ binaries.

## Target single production path

```text
operator START -> reset selected owner -> observe ULD -> map READY
 -> observe pick box/top domain/xyz -> shared PlacementQuery
 -> directed box heading + legal grasp G + pick/place joint branch candidates
 -> endpoint validation + MoveIt connection + timed whole-chain certification
 -> bound complete plan READY, including final pick xyz/J6 and place pose
 -> execute original pick leg -> seal -> actual attachment/suffix validation
 -> carry/place -> release confirmation -> released box/current-O retreat validation
 -> retreat -> measured map verification/commit -> next box
```

`G = box_from_tcp`. For every directed target, `container_from_tcp =
container_from_box * G`. Quarter-turn changes box occupancy and the coupled TCP
target; it is not a post-hoc J6 edit. Seal fixes G for carry and place.

ComputePlacement is a view over the common placement query, not an alternative
operation planner. The complete-operation root adds legal grasp/endpoints and
connections over those same geometric candidates. Compatible service v1/v2 names
are thin adapters. Atlas-builder v3 and observation schema versions describe
different contracts and must not be conflated into a generic version cleanup.

## Shared placement query design

Input is explicit: selected-owner snapshot/view, complete coverage descriptor,
measured box dimensions/uncertainty, allowed directed headings, hull, support
rules, separately named layout/motion/release policy and search budget. Committed
boxes are provenance-backed measurements, never synthetic occupancy from planned
slots. A predicted released box remains a private phase-specific collision object,
not a public measured O/G update.

Output includes candidate ID/center/heading, support evidence, volume/hull verdicts,
policy/snapshot identity, ranking terms, and rejection/observation-needed reasons.
Both clients reference the same inputs and pure implementation. Under identical
inputs and budget their geometric candidates and reasons must agree; complete
planning may reject additional candidates for grasp or motion reasons.

Rectangular occupancy can reuse two footprint computations (0/180, 90/270), but
must retain four physical directed poses for grasp, IK, J6 winding and paths.
Even a square box is not permission to merge tool/arm heading branches. Existing
`_snap_yaws` merges modulo pi and heading expansion is gated on `payload_symmetry`;
passing a four-value list alone does not guarantee four output headings. Define
allowed directed headings explicitly rather than inventing a symmetry prior.

Do not copy old extra 5 cm neighbor gaps into the shared query. Final nominal
side contact is allowed; uncertainty, motion clearance and release uncertainty
remain separate, evidence-based rules whose values are still under review.
No claim that nominal geometric contact is automatically executable under sensor
uncertainty. Unknown support or transit volume remains observation-needed.

Do not discard all quarter-turn or deeper alternatives with an early top-N.
Retain candidate groups by footprint, directed heading, location and IK branch;
schedule them within an explicit budget and record unvisited groups. Reuse support
and volume statistics across matching footprints, then refine actual grasp targets.
No feasible trajectory found within a budget returns SEARCH_INCOMPLETE with counts,
not NO_SPACE or UNREACHABLE. Genuine proofs must state the enumerated finite domain.

## O/G owner and normalized consumer contract

The proposed integrated launch selects `occupancy_backend=observer|grid_g`, default
observer. One branch starts O and not G; the other starts G and not O. Startup
also checks external owners: duplicate/ambiguous ownership refuses READY rather
than creating a second writer. Standalone/debug entry points remain explicit.

Common consumer operations are acquire view, read surface, read bounded cells,
validate current identity, and release view. Both return owner resolution, origin,
dimensions, hull/geometry hash, clock domain, epoch, map/observation revision,
acquisition validity, occupancy state, active-domain mask and coverage. Backend
IDs/epoch formats remain tagged, not disguised as each other's confirmation flags.

O's existing config and G's common grid config are not two independently writable
production resolutions. Launch resolves the chosen deployment resolution once,
binds it into the selected owner's effective configuration and digest, then
consumers verify that identity. A resolution change requires a new epoch. Floor
surface views remain floor-relative; O volume/grid origins remain container-absolute.

Native backend services may differ underneath adapters. Today O ReadCells already
supports begin/end plus cursor. O ReadSurface is full-domain paged. G GetCargoVolumeV2
returns the entire volume, without ROI or cursor. An adapter can provide semantic
parity by slicing a pinned G snapshot, but that does not save its initial full
transport cost. A real G ROI service is a separate implementation if measurement
justifies it; do not claim identical wire I/O already exists.

## Regional reads: proposal for discussion

Pages are transport chunks, not spatial neighborhoods. Use explicit half-open
cell bounds `[begin,end)` at the owner's original resolution, then paginate that
region starting at its own cursor zero. Changing region changes cursor meaning.
Never jump to an arbitrary cursor and infer unreturned cells are free.

Proposed first design:

1. Acquire one immutable view/lease and global hull/grid metadata. Read the full
   2D support projection for candidate discovery, rather than only a hand-picked
   shallow region. At 1 cm this is 29,353 columns, versus 4,344,244 volume cells.
2. Fetch exact 3D regions needed for candidate support, payload occupancy and
   endpoint/approach/retreat checks. Include full geometry and the explicit
   uncertainty/clearance policy, aligned outward to source cells. Cache by immutable
   snapshot identity plus cell block. Merge overlapping bounds without resampling.
3. For MoveIt connections, candidate box regions alone are insufficient: full arm,
   tool, sensors and payload sweep may leave them. Until a bounded-scene/lazy-query
   integration is behaviorally validated, load the full leased volume for final
   native whole-chain certification. This gives an honest initial implementation
   that saves cheap rejection work; not a claim of regional-only path planning.
4. Later regional-only connection planning must represent every unqueried region
   as unknown/unavailable, and enlarge coverage when exploration needs it. Padding
   only around goal/start does not establish a corridor or complete search. Native
   collision sampling must prove the geometry it consulted lies inside the admitted
   coverage, including between-state swept checks, or trigger expansion/refusal.
5. A full fallback may be chosen within the same declared deadline. If it cannot
   complete, return coverage/budget exhaustion with unread bounds and counts.
   Expired leases or a new owner revision require reacquisition and revalidation;
   cached old pages cannot be spliced into a newer snapshot.

At 65,536 cells/page the 1 cm full volume needs 67 pages. Existing ComputePlacement
rejects after 64 pages; fix count/progress/identity validation, not merely increase
64 to another magic constant. Size/page count limits follow accepted grid geometry,
MAX_VOLUME_CELLS and the request deadline, with explicit diagnostics.

Open design choices: whether regional-only native planning belongs in the first
baseline; total latency budget and how much may be spent extending deep coverage;
owner lease capacity/TTL under 1 cm full-scene construction. No arbitrary ROI radius,
page count or guessed performance improvement is selected here.

## Atlas and successful-route reuse within the unified path

Use a valid receipt-bound atlas for actual TCP targets implied by candidate G.
Consume contact/transit seeds, branch pairing, lift/clearance evidence and coverage
status before spending global connection effort. Manipulability and neighbor
confidence are ranking/diagnostic features, not collision certificates. Offset
grasp means box-center TCP atlas failure cannot reject every G for that box target.
Unknown cells trigger bounded online IK/refinement or observation-needed results.

Retain successful trajectories with model/config/scene identity, payload/G,
directed endpoints, exact joint branch/winding, event barriers and timing. Reuse
as a warm start or a candidate; new O/G contents, actual attachment and current
start require current native validation. Do not replay an old certified trajectory
under a new scene without this check, or use successful planned box poses as O.

## Ordered implementation and review checkpoints

This is a discussion successor, not an in-place rewrite or completion of claimed
G2. Executable units need a higher-generation thread/plan commit and exact commands
before dispatch. The table specifies acceptance intent, not invented test commands.

| Order | Bounded unit | Required observations before moving on |
|---|---|---|
| P0 | Pin production entry/prefix and build/install parity | Clean-shell imports and ROS package/executable lookup resolve to the pinned prefix. Installed new Python/native/messages exist. Changed source/install identities recorded; no reliance on a temporary overlay. |
| P1 | One O/G launch selector and common snapshot port | For O and G separately: exactly one producer, both ordinary placement and prepare consume the selected owner, original resolution/identity retained. Duplicate/mismatch/stale/unknown-backend cases refuse READY. |
| P2 | Shared admission + explicit four headings | Identical frozen inputs yield identical candidate geometry/reasons at both entry points. Non-square quarter-turn-only fixture succeeds; all four directed poses reach endpoint stage. Square-box tool branches remain distinct. Unknown support, overlap and hull violations reject; nominal side contact adds no unapproved process gap. |
| P3 | Bounded region access and full-volume certification fallback | Full vs tiled data are cell-identical for same snapshot. Boundary clipping, unknown/inactive semantics, stale mixed pages, stalled cursors and 67-page 1 cm case are tested. Outside queried bounds never becomes free. |
| P4 | Default bound-operation client and execution wiring | Final pick xyz/J6/place/trajectory issued together before seal. No legacy fallback. Actual attachment/current O changes can refuse suffix; failed release prevents retreat. Startup still waits for operator authorization. |
| P5 | Complete-model simulation baseline | Pinned deploy_update full URDF/SRDF, sensors/tool/payload; installed default entry. Simulated RGB-D/LiDAR/TF feed selected O/G and box perception. Pick/carry/place/release/retreat certified and simulated execution completed, with post-release observation. Truth is scoring-only. Preserve inputs and stage timings; no cup-only model, planned OCC or injected catalog dimensions. |
| P6 | Real ULD data and real baseline | Collect current ULD raw observations/TF and owner snapshots, verify alignment/error evidence and replay same installed default pipeline; distinguish plan-only, replay and actual hardware execution. Complete the agreed real whole-cycle acceptance; simulation cannot close this unit. |
| P7 | Optimize measured bottleneck | Atlas branch scheduling, shared queries and route warm-start cache evaluated on the same workload; report time/expansions and packing capacity separately. No success-rate claim from component test counts. |

Deterministic P0-P4 behavior matrices must pass every named positive and negative
case once; this is implementation-contract coverage, not a per-box reliability
estimate. P5 first successful full-model simulated cycle is one feasibility witness. It cannot
establish production reliability or capacity. A later reliability campaign needs
agreed per-unit target, workload, sample sizes, aggregation and observation-error
criteria; those are not specified or silently inferred here.

Before P5 scoring, repeatable startup/wiring rehearsals are unscored and bounded
by wall clock. Freeze source/config/model/map hashes for scored cases. A failure
preserves boundary inputs, map coverage/identity, candidate decisions, native
contacts with link/object/stage/sample, trajectories, logs and timings before
teardown. No scored-case artifact is edited to make the same run pass.

## Remaining inputs and review decisions

- Current bags contain pickup observations only. The user confirmed simulation
  first, then collection of real ULD data. Existing data does not qualify a real
  release/multibox baseline; the simulation baseline must be labeled separately.
- Regional read phase 1 retains full-volume final certification as proposed above.
  Regional-only MoveIt planning is not assumed ready.
- Release height/method, uncertainty and motion clearance values need evidence and
  the separate clearance review. They do not revert to a universal 5 cm rule.
- Full-model plan-only, simulated full execution and real hardware execution are
  separately labeled outcomes. Hardware contact accuracy, released-box association
  and abnormal-contact detection remain unqualified by current pickup-only data.

## Pointers

- `docs/reviews/PLACEMENT_PLANNER_DESIGN_AUDIT_20261112.md`
- `docs/plans/PLACEMENT_CLEARANCE_REDESIGN_DISCUSSION_20261112.md`
- `docs/agents/discuss/2026-11-11_2057_joint-operation-full-integration-g2.md`
- `docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-INVENTORY/run_01/manifest.json`
- `docs/architecture/cargo_grid.md`
- `docs/architecture/production_orchestration.md`
