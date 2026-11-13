# Planning routes and module ownership

The Python algorithms live under `luggage_planning/`, grouped by responsibility.
These folders are not four interchangeable execution modes: prior and atlas are
inputs to planning; whole-operation and segmented planning have separate entries.

```text
luggage_planning/
├── joint_operation/    complete pick/carry/place preparation and bound execution
├── placement_prior/    read-only candidate screening and snapshot evidence
├── routing/            segmented route production and connection strategies
├── atlas/              sampling, qualified proposals, graph and seed helpers
└── *.py                shared attachment, geometry, observation and motion ports
```

| Responsibility | ROS / launch entry | Python entry | Main modules |
|---|---|---|---|
| Whole operation | `joint_operation_prepare.launch.py`, `/joint_operation/prepare` | [joint_operation/prepare_node.py](luggage_planning/joint_operation/prepare_node.py) | `preparation`, `candidates`, `planner`, `aperture`, `certification`, `revalidation` |
| Bound execution | `motion_planner_node.py`, prepared-operation consumer | [joint_operation/runtime.py](luggage_planning/joint_operation/runtime.py) | `runtime_ports`, `execution`, `attachment`, `actual_contact` |
| Placement prior | `place_prior_node.py`, `/placement_prior/prepare` | [placement_prior/worker.py](luggage_planning/placement_prior/worker.py) | `core`, `snapshot`, `pick`, `local_paths`, `ranking`, `adoption` |
| Segmented route | `trajectory_selector_node.py`, `/trajectory_selector/select_place_path_v2` | [routing/factory.py](luggage_planning/routing/factory.py) | `producer`, `direct_first`, `lattice`, `corridor_supergraph` |
| Atlas | builder tools and planning consumers; no independent execution service | [atlas/proposals.py](luggage_planning/atlas/proposals.py), [atlas/runtime.py](luggage_planning/atlas/runtime.py) | `model`, `wavefront`, `graph`, `seed_chain`, `frame_contracts` |

## Whole-operation route

```text
joint_operation_prepare_node.py
  → joint_operation.prepare_node
  → preparation / candidates
  → planner → aperture / ik / connection
  → certification / issuance
  → retained complete plan
motion_planner_node.py → joint_operation.runtime → execution / runtime_ports
```

PrepareJointOperationV2 accepts optional `placement_task_mode` (`floor`/`stack`),
exclusive with explicit slots. The owner enumerates through the same shared
observed placement kernel after retaining the measured contact acquisition. This
binds candidate-grid parity, box dimensions, support height and native planning
to one geometry; clients do not regenerate dimensions after selecting slots.
Regenerate ROS interfaces when updating this field.

The same prepare node exposes the read-only
`/joint_operation/prepare_pickup_view` O recovery prerequisite (`view_owner`,
`view_context`, `view_motion`). It accepts only an owner-retained unresolved
contact from `SEARCH_INCOMPLETE`, uses the exact acquisition and latest O/full
scene, and re-admits the original placement before native view planning. Camera
sampling, optical range, IK and time budgets are explicit request inputs. It
returns an independently retained `PickupViewPlanV2`, not a six-leg operation or
execution permission. Controller dispatch, settling, new acquisition and unique
same-box association remain separate orchestration work. Ordinary successful
preparation does not query raw camera evidence or execute an observation.

Preparation keeps the existing position priority and groups already-admitted
headings with exactly the same XYZ. At each target it tries the primary measured
contact for both directed tool headings before requesting alternate centres.
`max_contact_proposals_per_direction` (1..4, default2) controls that search; the
simulation option is `operation_max_contact_proposals_per_direction`. Alternatives
spread across the measured feasible support rather than shifting the placement.
Each heading shares the existing `max_center_checks` geometric budget, and native
candidate/branch caps and the global deadline remain unchanged. These are work
bounds, not error tolerances or reachability boundaries. All contacts require
the full observed footprint and frozen quality; prepare never moves the robot.

The aperture generator belongs to this route and retains its ordered waypoints
inside the carry leg. ComputePlacement provides geometric slots. Actual attachment
geometry, current occupancy, configured ULD and native complete-model collision
checks decide whether a candidate has a valid complete trajectory.
The opening cross-section bounds the conservative payload, not an orthographic
projection of every fixed camera/head part. Those parts may remain outside the
ULD; full-model IK, edge checks and whole-chain certification still check them.
The outside/plane/inside witnesses and ordered carry connection remain required.

The standalone native connector derives OMPL's segment fraction from its own
joint-check step and the loaded group's extent. Search and returned-path checks
therefore use compatible resolution even when no external OMPL YAML is supplied.
Opening IK retains both endpoint seeds before its bounded atlas proposals;
remaining atlas seeds are ranked by raw joint travel, preserving winding.
Native IK selects the seed-nearest legal full-turn equivalent before checking
and retaining a new solution. This never clamps a joint or rewrites an issued
path. Whole-chain refusal traces retain the certification reason and each
timed leg's knot count and duration.
Align and carry retain the checked joint polyline and stop at its corners.
Only numerical collinear redundancy is removed; all three opening waypoints
remain explicit. Rest-to-rest quintic segments use the configured velocity and
acceleration limits without globally stretching the path for one short corner.
Local Cartesian contact, lift, release and retreat keep their existing timing.
The simulation profile exposes per-edge connection time and global/per-candidate
branch limits. Its diagnostic search uses 5s per edge and 8 total branches,
at most 2 per coupled candidate, within the existing 300s preparation watchdog.
These are search budgets, not reachability decisions or a production latency SLA.
Runtime phase diffs replace the owned world payload directly and send remove or
detach operations only for objects actually present in the captured live scene.
Owned box comparison composes object and primitive poses, so MoveIt's equivalent
pose storage and quaternion sign cannot appear as an external geometry change.
Layout re-admission checks the original bound slot; the existing FK accuracy
check and exact FK-derived release obstacle still govern native suffix collision
certification. Runtime uses that same release geometry for the retreat scene.
Within one action, the user-confirmed static-environment premise permits retained
O evidence while the wrist sensors look away from ULD. The age allowance ends
with the action budget, preserves the original timestamps, and is recorded in
`stationary_map_scope.json`. Epoch/config/validity and changed-content checks
remain active. Preparation still requires its configured fresh observation;
post-release adoption still requires an actual acquisition after release/settle.
Preparation pins immutable lease pages without demanding a newer counter stay
unchanged during transfer. Its final source checks compare fresh complete O
classification/support content using the same comparison as execution; a
log-odds-only revision cannot force refusal, while changed content still enters
the existing full-chain latest-map certification path.
Frozen contact evidence must pass the original age limit when opening a new
action. Only that admitted action retains it through its bounded deadline,
without renewing its timestamp; perception-owner revocation still applies.
Execution boundary archives use the same lossless gzip/raw-hash convention as
prepare, and their unchanged storage cap counts compressed bytes.
Prepare evidence stores lossless gzip CDR files; `boundaries.jsonl` records raw
and stored sizes/hashes. Decompress before deserialization; the budget counts
stored bytes and still refuses when full.

## Selecting the route

`luggage_gazebo` selects the route with `planning_pipeline`: `joint_operation`
uses the complete-operation flow; `legacy_segments` uses the segmented flow.
The grouped [joint-operation profile](../luggage_gazebo/config/joint_operation.profile.yaml)
selects `joint_operation`. The bare simulation launch still defaults to
`legacy_segments`; this directory migration does not change that default.
`frame_runtime` is a separate compatibility setting, not the route selector.
The explicit profile currently integrates RGB-D only (`operation_o_sources=rgbd`)
after measured simulated LiDAR floor returns generated a false support cell.
The full LiDAR model and point outputs remain available; `rgbd_lidar` explicitly
restores both measured inputs after source qualification. No O cells are erased
and all native collision/unknown-space gates remain unchanged.
See [production orchestration](../../docs/architecture/production_orchestration.md)
for authorization and map-owner rules.

## Shared interfaces and installation

Modules used across routes stay outside these folders until their ownership can
be established independently. In particular, placement-prior `contracts` also
contains existing shared geometry types; this move does not redefine that type
ownership. Native C++ backends in `src/` serve multiple routes. ROS wrappers stay
under `scripts/`; launch, config and test paths retain their existing names.

Repository imports use explicit new paths, for example:

```python
from luggage_planning.joint_operation.planner import JointOperationPlanner
from luggage_planning.routing.factory import build_production_route_producer
```

The complete old-to-new Python path map is
[python_module_layout.json](config/python_module_layout.json). Old internal
Python paths are retired; there is no duplicate implementation or import hook.
The former `placement_prior.py` name now denotes the `placement_prior/` package;
its former functions live explicitly in `placement_prior.core`.
ROS service types/names, executable names and launch arguments are preserved.
Build with `scripts/production/build.sh`: it removes only mapped obsolete Python
modules after the replacement install succeeds. The installed checker imports
every moved module, compares its source bytes and rejects old import paths.

### Successful whole-operation path reuse

`joint_operation/path_cache.py` stores bounded geometric/timed proposals only
after terminal, payload-free successful execution. `successful_path_cache_directory`
(default `~/.ros/luggage_successful_paths`; empty disables) must be the same for
prepare and execution owners. Its exact key binds robot/model, ULD/scene identity,
start joints and spatial pick/release tool poses. Body poses, map revision and dimensions
are deliberately outside the key: a hit is reconstructed with current identity
and attachment and independently passes the full native whole-chain certifier.
No cached certificate, contact permission or map evidence is reused. Failure
returns to local/connection search. Current execution checks and event barriers
still apply. The first implementation requires exact spatial keys; it does not
claim approximate endpoint repair or measured latency improvement.
