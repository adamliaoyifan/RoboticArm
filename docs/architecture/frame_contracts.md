# Frame contracts: staged v2 baseline

This document defines the new **offline boundary contract**, its checker and
migration gate. It does not claim that current ROS messages or live nodes have
adopted v2. Existing live placement height semantics remain those documented in
[placement.md](placement.md) until their producers and consumers migrate
together. Interface-by-interface adoption is tracked in
[frame_interfaces.json](frame_interfaces.json); initially every row is pending.

## Canonical representations

| Quantity | Representation |
|---|---|
| Raw images / sensor points | Truthful optical / sensor frame, original acquisition stamp |
| Detected box center | `world`, measured body-axis dimensions; box-axis orientation |
| Measured suction contact | `world`, surface-contact point; local outward +Z normal |
| Slot / container geometry | `container_link`, box center; full quaternion with geometric π symmetry |
| Cargo surface | `container_link`, z relative to container origin; separate configured floor z |
| Container carry path | `container_link`, suction-contact-frame origin, directed tool orientation |
| Executable Cartesian motion | Current planning frame `world`, explicit suction target link |
| IK target | Model-required frame, currently `elfin_base_link` for the relevant offline model |
| Controller trajectory | Named joint coordinates; no Cartesian frame substitution |

Body-axis width is along box +X and depth along box +Y. Do not infer long/short
axis ownership from a catalog. Box footprint symmetry is π; concrete tool
orientation has 2π periodicity. q and -q represent the same rotation, but yaw
0 and π tool-down poses generally do not. Joint winding and IK seeds are
separate from quaternion sign.

The 2.5D task supports a horizontal container. The transform utility supports
full SE(3), but that does not broaden the gravity/stacking contract. Use
`require_horizontal` at container-geometry admission; reject unsupported tilt.

## Plain boundary records

`luggage_description.frame_contracts` is ROS-free. These are plain validated
records, **not newly generated ROS message layouts**. A future ROS binding must
preserve these semantics and be independently tested across real serialization.

All pose/surface records require:

```text
contract_version: 2
kind: pose | surface
frame_id: explicit frame name
length_unit: m
stamp_ns: integer >= 0
time_basis: acquisition | robot_state | scene
scene_revision: nonempty calibration/scene identity
request_id: nonempty correlation identity
```

Zero is a valid simulation epoch, not a request for latest TF. Acquisition
geometry keeps its measurement stamp. Starting robot state keeps its own state
stamp. Static scene goals bind to the chosen scene snapshot. These times must
not be collapsed into a fabricated common `now()`.

`FrozenSuctionObservationV2` retains the original world acquisition header and
hash-bound request/observation/box/generation/model/domain/sample/policy identity.
`CheckFrozenSuctionContactV2.box_from_contact_frame` is a measured body-frame
SE(3), not a current world/wrist transform. Its returned
`world_from_contact_frame_at_observation` refers to the frozen acquisition. A
contact/quality positive is neither a physical-seal nor an execution permit.
Invalid or unavailable bindings return an inactive result without an old witness.

The F24 legacy bringup files named by historical isolation evidence are absent
from the cleaned workspace. That receipt cannot certify the current state.
The obligation remains pending, with pointers to the remaining ROS1 packing
reference and current install declarations; restoring legacy execution requires
fresh isolation or migration evidence. An offline geometry pass does not close
that obligation or certify any live integration.

Pose payload: `position[3]`, `quaternion_xyzw[4]`, `reference`, and
`orientation_semantics`. Named checker profiles are:

| Profile | Frame | Reference | Orientation | Time basis |
|---|---|---|---|---|
| detection | world | box_center | box_axis_pi | acquisition |
| contact | world | surface_contact | surface_normal | acquisition |
| slot | container_link | box_center | box_axis_pi | scene |
| map_tcp | container_link | suction_contact_frame | tool_quaternion | scene/state/acquisition |
| motion | world | suction_contact_frame | tool_quaternion | scene/state/acquisition |
| base_tool | elfin_base_link | suction_contact_frame | tool_quaternion | scene/state/acquisition |

`state` in the last column abbreviates the actual value `robot_state`.
Missing/unsupported frame, reference, version, scene identity, unit, time basis,
nonfinite coordinates, malformed vector and non-unit quaternion fail. Do not
coerce numeric strings or silently normalize invalid quaternions. Norm tolerance
is 1e-6. Extra provenance fields may be preserved; they do not override checks.

Surface payload: `geometry_hash`, `map_revision`, `origin_xy_container[2]`,
positive `resolution_m`, `height_datum=container_origin`, `floor_z_container`,
`ceiling_z_container`, and matching rectangular `surface_z_container`, `state`,
`confidence` arrays. The consumer supplies expected geometry and map revision.

- Unknown top: null and confidence `unknown`; not numeric zero or floor z.
- Measured free column: surface equals floor, confidence `sensor` or
  `verified_geometry`. Occupied tops lie within floor/ceiling bounds.
- Unverified planned geometry is not admitted as measured support.
- Optional `height_above_floor` is derived from surface minus floor; if present,
  every element must agree within 1e-9 m, with null at unknown cells.
- Floor-prior policy stays explicit and separate from measured free-space claims.
  This checker does not authorize exploration, pick, placement or motion.

Concrete-tool atlas metadata requires separate `position_frame` and
`orientation_frame` (both container_link), `reference=suction_contact_frame`,
`orientation_semantics=concrete_tool_pose`, `tool_down_reference=container_negative_z`,
metres/radians, container-origin height datum, 2π yaw period, unique yaw bins in
[0,2π), and matching scene/geometry/URDF/payload identities. This verifies metadata,
not that seeds are valid IK solutions. π-aggregated box feasibility is a different
future API, not an alias for a concrete tool-pose atlas.

## Transform and reference-point boundaries

### Complete operation plan binding

`PrepareJointOperationV2` is an independent plan-only boundary. Its result binds
the measured observation, current robot-state stamp, pinned map/snapshot,
container geometry, robot model, contact model, request, box and generation.
Acquisition and robot-state stamps remain distinct; epoch zero is valid only
with the corresponding validity flag set. A response claiming `PLAN_READY`
must contain all pick, carrying/place and released retreat trajectories.

`JointOperationPlanV2` contains five directed world-frame poses in metres/xyzw:
nominal box centre and suction_contact_frame at pick and release, plus the
settled box centre. The `world_from_place_*` fields describe release, including
the configured release gap; `world_from_settled_box` describes the placement
target after settling and supplies the world obstacle for retreat validation.
Full box axes are retained; release and settled orientation must agree. The
same predicted attachment must compose each tool pose into its corresponding
pick/release box pose. Attachment geometry reuses `AttachmentGeometryV2`
without re-expanding its conservative dimensions. It is a prediction until
actual attachment confirmation, not a measured-attachment declaration.

All legs carry existing timed `JointTrajectoryContractV2` records, in canonical
elfin_joint1..elfin_joint6 order. Exact winding and trajectory continuity are
preserved. Ordered barriers before first carry and first retreat require
attachment and release confirmation, respectively, with stationary endpoints.
The complete plan digest covers identity, poses, predicted geometry, event
barriers and every trajectory knot. Consumers compare against an independently
retained identity and approved digest. Post-plan changes require a newly checked
and bound plan, including any J6 change.

`CheckPreparedOperationV2` is read-only. It submits the complete plan for
comparison with the preparation owner's retained identity/digest; client fields
cannot establish their own expected identity. A positive response proves only
current owner issuance, not current observation, attachment, collision or
controller admission. Retention/restart semantics are specified in
`production_orchestration.md`.

The pure contract and ROS adapters verify binding and structure. They do not
prove FK, collision freedom, controller readiness or certificate liveness. The
whole-chain planner must certify empty/world-box, attached and released/world-box
scenes before sealing. Execution must check actual q and live admission, confirm
attachment and revalidate carrying paths at the barrier. Schema availability
does not imply that the service or execution path has been adopted at runtime.

The private preparation certifier independently checks endpoint FK against the
captured world-rooted robot model, the explicit deployment release gap along
container up, and the actual frozen payload geometry/ACM in each phase scene.
It sends original timed knots and controller-interpolated samples to
`ValidateRouteV2`, with no support-contact exemption. A positive receipt binds
the draft digest, phase scene digests, model and captured native evidence.
This is discrete sampled validation under the existing controller model; it
does not establish continuous collision freedom or confirm an actual grasp.

`PlanSnapshotJointPathV2` is the internal joint-connection primitive for that
private planning path. The full scene travels as its original serialized CDR
bytes with SHA256; it must not be decoded and reserialized to reproduce a byte
identity, because serialization padding is not a canonical semantic encoding.
The node hashes and deserializes those same bytes, matches its robot model
(URDF+SRDF SHA256), then creates a fresh private MoveIt PlanningScene. It plans
joint-space connections with OMPL, no endpoint-repair adapters, and rechecks
interpolated joint states at <=0.01 rad. The configurable check cap is <=50000
states. Deadline/cap/failed search do not prove spatial unreachability.

Successful output preserves exact canonical start/goal and winding, and contains
positions only. It is geometric evidence, never an executable timed trajectory
or an operation certificate. The Python port retains one in-flight request even
after cancellation/timeout, binds response identity to consumer-held expectations,
and rejects incomplete, timed, undersampled or endpoint-shifted responses. Timing,
controller-interpolated whole-chain checking and execution liveness remain the
responsibility of operation assembly/certification and execution adapters.

The structured `AtlasProposalSet` consumer uses `query_xyz_container`,
`sample_xyz_container` and each branch's `transit_xyz_container` in
container-ABSOLUTE metres. Z is never floor-relative. `transit_lift_m` and
`lift_clearance_m` are lengths, not absolute heights; the legacy cell clearance
and branch-kind scalars describe the first stored branch only. Every transit
keeps its own recorded XYZ, which may belong to a predecessor cell. Contact and
transit are paired by their original array index before finite-value filtering.
Zero joint vectors are valid when included in the explicit solution count.

Only an explicit offline qualification matching the exact NPZ/YAML digest and
full frame/model/geometry/payload contract enables the new proposal reader. Old
deploy_update files are not qualified by matching scene hashes alone. A missing
qualification supplies no seeds or negative reachability verdict. The reader
does not issue qualification records or change legacy query/rejection behavior.
Paired consumption refines the exact requested contact and recorded transit,
checks both in the current predicted carrying scene, then requests an online
geometric connection. These paths remain untimed hints for whole-chain assembly
and certification. Missing transit provenance retains contact-only hints and
requires online fallback; deadline/cancellation exposes no partial result.

`frame_transforms.transform_pose` accepts an explicit source/target transform:
translation, unit quaternion, scene_revision and stamp_ns. An explicit null
transform stamp means static for that scene revision; a dynamic transform stamp
must match the record exactly. Both position and orientation are transformed.
The operation returns a fresh record and retains acquisition/correlation data.

`box_to_tool_target` changes the represented point using a selected grasp:

```text
T_reference_tool = T_reference_box * T_box_tool
```

The grasp transform names source `suction_contact_frame`, target `box`, and its
scene revision. Its translation includes actual lateral contact offset, not only
half box height. The caller must bind the grasp to the correct measured object
and selected directed box representative; this utility is not an object-identity
or grasp-authorization gate. `rotate_vector` rotates normals/offsets without
adding an origin translation. ST02 adds explicit 3x3 positional/vector covariance
rotation; full 6D pose covariance and uncertainty propagation remain work for
the relevant migration rows.

## Interface inventory

The JSON is the machine-readable inventory authority. IDs link required
geometry/negative cases, source pointers, current assumptions and target meaning.
Case linkage is **not** evidence that a ROS consumer has installed the check.

| IDs | Boundary |
|---|---|
| F01–F05 | scene/TF; sensor ingress; detector; suction candidates; opening estimate |
| F06–F08 | cargo surface; solver candidates; SlotSpec producer/consumers |
| F09–F11 | pick legs; retry/recovery; place and grasp-to-tool conversion |
| F12–F14 | sweep/grid; selector service; atlas/task-grid artifacts |
| F15–F18 | motion serialization; IK/compensation; collision scene; attach/detach |
| F19–F21 | commit/remove/verify; measured integration; startup/orchestration |
| F22–F24 | eval/fixture drivers; GT/replay/viz; isolated legacy integration |

`migration_state` is pending, verified, or isolated. Marking a row verified alone
does not clear the gate. It also needs `runtime_evidence`, pointing under
`docs/status/evidence/` to an object with interface_id, kind=runtime_boundary,
status=pass, command, nonempty observations, positive sample_count, and a
source_sha256 entries matching **every** row source and both shared contract/
transform modules (`CONTRACT_SOURCES` in the checker), `contract_version=2`, and
`interface_contract_sha256` matching the checker-computed row contract digest.
Inventory v1 requires all F01–F24 rows; deleting a pending row is an error.
Isolation requires
kind=isolation_check and scope=legacy. Missing or stale proof remains pending.
Proof references are not cryptographic attestations; the implementation owner
must produce them from actual behavior. Negative/positive runtime acceptance
criteria must be approved when each row is implemented.

This is a curated boundary inventory, not a claim that static source scanning
can find every dynamically constructed caller. New producers/consumers require
new rows or updates. Actual runtime package paths, message versions, profiles,
and branch coverage must be captured during integration, not inferred from a
source-tree pass.

## Run and interpret the gates

From the workspace root, with Python 3 and the existing description-package
dependencies (PyYAML and ament_index_python for the original scene-helper case).
The validators themselves use only the standard library. Source the installed
Humble environment before comparing the original helper; no ROS process starts:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/check_frame_contracts.py --mode baseline --output docs/status/evidence/frame_contracts/my-baseline
python3 scripts/check_frame_contracts.py --mode migration --output docs/status/evidence/frame_contracts/my-migration
PYTHONPATH=src/luggage_description python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py
```

Use a fresh output directory every time; overwrite is refused.

- Baseline exit 0: inventory is well-formed and all deterministic offline cases
  passed. `runtime_execution=not_evaluated` still applies.
- Migration exit 2: offline checks passed, but pending/stale interface proofs
  block claiming migration complete. This is an expected refusal today.
- Exit 1: malformed inputs, missing sources or failed comparison/check.

Each output directory has exact suite/inventory snapshots, complete case
inputs and expected/actual values, summary and a revision/source-hash manifest.
To replay, pass `--suite <run>/suite.json --inventory <run>/inventory.json` with
a fresh `--output`; compare recorded source hashes first. Source changes mean
a comparison on the new implementation, not reproduction of the old binary.

The fixed analytic cases use absolute position tolerance 1e-9 m and quaternion
sign-equivalent component distance 1e-9. They include nonzero translation and z,
90/180/non-axis-aligned rotation, full RPY mechanics versus tilt rejection,
non-square footprints, off-center TCP, changed floors and stacks, unknown cells,
q/-q, different tool branches, old contracts and stale identities. No random
sampling, IK reliability, physical success rate, or hardware certification is
inferred from this deterministic AND gate.

## Failure diagnosis

Callers add interface_id, producer, consumer and request_id to ContractError's
code/field/expected/received diagnostics. Keep the original and transformed pose
plus transform snapshot at stage boundaries. Never replace a contract failure
with generic IK failure, BIN_FULL, default pose, identity TF or empty map.

Runtime labels cannot prove numeric truth. Independent geometric oracles are
required because even a transform roundtrip can pass when both sides share the
same wrong convention. Live integration and dump windows follow
[debug-evidence](../../.cursor/rules/debug-evidence.mdc).

## ST02 v2 ROS binding substrate

The generated types `FrameIdentityV2`, `PoseContractV2`, `WaypointArrayV2`,
`SurfaceContractV2`, `JointTrajectoryContractV2`, and `BoundaryDiagnosticV2`
implement the plain-record boundaries above. `ValidatePoseV2` reserves
`/frame_contracts/v2/validate_pose`; ST02 installs its type only, not a server.
All existing v1 types/endpoints retain their previous semantics. No automatic
v1 conversion, producer switch or runtime inventory completion is implied.

`luggage_description.ros_frame_bindings` is a dedicated ROS boundary layer;
`frame_contracts`, `frame_sequence_contracts`, `frame_transforms`, and
`frame_diagnostics` remain ROS-free. Perception and planning expose explicit
profile adapters in their respective `frame_message_adapters` modules. Adapters
must run before any publish, plan/execute, vacuum, or occupancy side effect.
A successful conversion returns fresh objects; an invalid payload raises
`BoundaryContractError` with the original `ContractError.code` and a structured
`.diagnostic`. Unsupported extra record fields are rejected rather than silently
losing provenance. Callers must extend the versioned schema before adding fields.

Every caller supplies an independent `BoundaryContext`: interface, producer,
consumer, expected request and scene, frame/reference/time basis, and (for maps
or paths) expected hull and map revision. Expected identities must come from
the consumer's admitted request/scene/map, never be copied from untrusted input
as a way to bypass comparison. Diagnostics retain declared and expected values,
including malformed or absent stamps, for both rejection and acceptance.

- `stamp_valid=false` means absent; `true` with `stamp_ns=0` is a valid epoch.
  Nanoseconds are uint64 and are never converted through float seconds.
- Cartesian poses retain all quaternion components and explicit reference and
  orientation semantics. Paths are nonempty ordered poses, each with its own
  stamp/time basis. All share the admitted frame, scene and request; each pose
  is validated against the chosen profile's permitted time bases. The path's
  own envelope is validated against the caller's expected time basis.
- Surfaces carry row-major dimensions and a `valid` bit per cell. Invalid cells
  use wire z=0 only as a canonical encoding, decode to `None`, and must retain
  `state=unknown, confidence=unknown`. Inconsistent validity, nonfinite payloads
  or an unflagged derived-height array fail. No adapter treats wire zero as
  measured floor support. Optional relative heights retain explicit presence.
- Joint-space records retain unique joint names/order and full position,
  optional velocity/acceleration/effort arrays and strictly increasing duration.
  Angles use radians; point arrays must match joint count. There is no Cartesian
  frame/reference/length-unit field. The nested standard JointTrajectory header
  must remain empty/zero: its timestamp authority is the v2 envelope.
- `rotate_covariance3` transports a finite symmetric row-major 3x3 positional or
  vector covariance as R C R^T. It is not a 6D pose covariance conversion or an
  uncertainty estimator. Normals/vectors rotate without translation. Full SE(3)
  math and the separate horizontal-container admission restriction remain intact.

The executable gate is
`src/luggage_planning/test/test_frame_migration_messages.py`, run against an
isolated installed Humble build with actual generated-message serialization.
Set `FRAME_MIGRATION_CASE_LOG` to preserve per-case inputs/outputs/diagnostics.
Live F01--F24 adoption remains pending until each producer/consumer chain earns
its later runtime proof; shared serialization is not that proof.

## ST09 tooling primitives (runtime adoption remains gated)

`runtime_profile` resolves grouped launch YAML with CLI precedence, rejects
unknown keys and ambiguous duplicate leaf names, and validates complete v2
identity records. Sim and hardware launch profiles share that resolver.
`runtime_readiness.ReadinessGate` is a ROS-free consumer admission primitive:
missing peers/endpoint graphs disable admission; restart, changed identity,
expired lease, duplicate provider or mixed type revoke readiness. A v1 request
cannot pass this primitive. Heartbeat metadata and readiness are excluded from
immutable fingerprints. Except for the explicitly renewable planning lease described below, a heartbeat arriving after a lease gap revokes admission
even if no request was made during the gap. Explicit `readmit` increments the
admission generation and requires fresh reports; old request tokens remain
invalid. This operation does not authorize a cycle or release carried payload.

`runtime_manifest` defines `luggage.runtime_manifest/2`: exact registered types,
provider/client ownership, process configuration and artifact pins. Application
artifacts must resolve under the isolated install; explicitly labelled and pinned
system artifacts resolve under the configured ROS root. Standard interfaces retain
their exact standard types. `RuntimeIdentityV2` reports local loaded provenance;
its steady-clock publisher runs at 4 Hz regardless of readiness or simulation
clock progress. It does not itself collect process reports, wrap
production entrypoints, or prove runtime adoption. F21 remains pending until
those integrations have real graph evidence.

`luggage_description.frame_dump` (with a compatibility import at
`luggage_gazebo.frame_dump`) defines `luggage.frame_dump/2`. It joins pose/surface
identities to independently supplied consumer expectations, stores body X/Y/Z
sizes and a world transform snapshot, preserves exact acquisition nanoseconds
and unknown cells, and distinguishes `measured` from `gt_reference`. Writes are
exclusive. The only legacy surface adapter, `surface_floor_relative_v1`, takes
an explicit `container_link` frame, `floor_relative` datum, floor, ceiling and
consumer identity; conflicting declarations fail. GT rasterization now explicitly converts base-frame inputs to the mapper's
container frame and transforms free voxels from container to world. Existing
GT writers and live visualizers are not yet switched to this envelope, so F23 remains pending.

When a selector receives a probe callback it requires positive IK and fraction
evidence. Grid seeds do not replace the requested probe. Eval probe service
loss, timeout or empty paths reject candidates; an explicitly disabled probe
retains the separately configured occupancy-only behavior.

`integration_acceptance` validates joined two-box evidence and GT-labelled
metrics separately from measured map acceptance. Its event schema is provisional
until measured production events and capture windows are frozen after rehearsal.
The ST09 verification runner reports missing graph/rehearsal obligations as
`not_evaluated`; that result blocks release and is not a runtime certification.


## Diagnostic attached payload envelope (2026-10-22)

Nominal measured box dimensions and T_tool_box remain the target-synthesis
contract, including eccentric grasps. Planning may bind a separate immutable
collision_envelope to AttachmentRecord. Its digest participates in attachment
identity and hence route/certificate/admission identity. It is never perception
geometry or an online ground-truth input.

The diagnostic profile in luggage_planning/config/payload_envelope.yaml uses
suction_contact_frame origin/local Z as the pivot/axis for bounded post-attach
yaw; known orientation stays in the nominal transform. Roll/pitch are exact;
box Z must be parallel/antiparallel to contact Z. Unsupported tilt is refused.
Size margins are outward on each box-axis face; translation half-ranges are
in tool axes after yaw. Enumerate corner sinusoid extrema over the continuous
interval, including interior stationary angles; preserve the resulting offset.
This profile states assumed bounds, not calibrated statistical coverage.

Slot closed-hull containment, corridor footprints, graph payload identity,
carrying collision scenes, released obstacles and certification must consume
this same envelope. Nominal grasp/placement targets remain unchanged. In
particular, no floor/contact exemption is implied by a larger envelope:
positive vertical margins can make a nominal floor-contact slot infeasible.
Existing suction touch links are unchanged; link2 is not exempted.
Configuration changes require a selector restart/new frozen attachment and
new certification. Out-of-bound observed slip retains the existing invalidation
and stop/reconfirmation behaviour; this change adds no slip sensor or claim
that unobserved motion is detected.


G2 diagnostic exception (user-directed 2026-10-22): tool-Z translation bound
is now5mm (XY100mm and per-face50mm unchanged). Explicit
`diagnostic_skip_slot_envelope_check=true` bypasses only the placement
closed-hull prefilter. Defaultfalse retains it. Record this diagnostic mode in
candidate rows/manifests; full-arm trajectory collision certification and
actual post-place verification remain authoritative. The exception does not
claim hull containment and does not authorize link2 collision.

G3 diagnostic profile (user-directed 2026-10-22) replaces the fixed per-face
size margins and XY100mm translation range. Width/depth grow by10 percent total
and symmetrically. Height grows by10 percent only on the face away from the
suction origin; the5mm Z attachment error is applied on that same face. For
the supported tool-down, box-vertical carrying contract this face extends in
world `-Z`. The contact-facing height face is unchanged. XY attachment error is
zero in this diagnostic profile. Continuous yaw+/-5deg remains, so its exact
union may exceed1.10 times nominal width/depth. An attachment for which the
near/far height faces cannot be distinguished from the suction origin is
refused instead of choosing a direction implicitly.


## Attachment-owned conservative geometry (G4, supersedes G1-G3 ownership)

VacuumCommandV2 freezes an AttachmentGeometry from admitted measured dimensions,
actual tool/box relation and the single attachment_geometry.yaml policy before
performing the attachment effect. Its response carries measured dimensions and
an explicit AttachmentGeometryV2: conservative dimensions, tool-relative centre
and orientation, policy provenance and digests. Nominal fields never become
inflated fields. Missing or mismatched geometry is a named refusal at executable
selection admission. No online physics/reference size enters this construction.

The scene attach and route consumers use that frozen output. Selector/provider
must not load tolerance configuration or rebuild envelopes. Capacity generation
receives conservative dimensions; its conservative-centre targets are converted
back to nominal box targets by an Attachment adapter. Inputs and shared message
objects must not be mutated; each converted target is a fresh output. Envelope
+Z is represented parallel to nominal box +Z (equivalent cuboid axis signs), so
an upright packing target cannot flip the nominal box. All geometry stays frozen
until a new measured attachment is admitted. Existing nominal map verification
and contact targets retain their measurement semantics.

The old late slot-envelope hull filter and its diagnostic bypass are removed;
capacity is handled upstream. Collision, edge and certification checks remain.
Bounds remain diagnostic: 10% total size growth, height only away from contact,
5mm away-position uncertainty, +/-5deg suction-pivot yaw, zero XY translation
uncertainty, exact roll/pitch under the vertical box/contact-normal contract.

### Action-scoped readiness (2026-10-23)

The immutable wire identity still describes the full graph. Certified unloaded
named-pose execution admits only selector, motion planner, vacuum, scene, cargo
map and driver peers; placement and waypoint production are not dependencies of
this action. Independent gates observe from startup, preserving relevant lease
gaps, incarnation changes, endpoint ownership and token generation. The executor
still verifies the full certificate, exact leg/load phase, fresh detached vacuum,
scene and map immediately before submission. Empty/forged certificates do not
authorize a smaller dependency set. Placement and next-box requests retain the
full-graph check. Only placement-planner lease expiry is renewable for a new
request: a late valid heartbeat advances its lease epoch and permanently voids
any token issued in the prior epoch. Missing/stale heartbeats cannot admit a
new request; restart/identity/provenance mismatch still latch closed.

Return planning freezes the current post-commit occupancy snapshot as a value,
records its digest/revision, and uses it throughout building and certification.
The map already changes only via explicit mutation services; return performs no
placement computation or integration. Physical occupancy events and revision
changes remain visible to execution revalidation and can revoke the route.
The next placement request reads the latest map rather than reusing return's
snapshot. A frozen planning input never licenses ignoring a new obstacle.

The pack evaluator commits a verify-anchored placed-box record before requesting
return planning. The return snapshot must therefore include the just-released
obstacle, not only boxes from prior completed cycles. Suite success is counted
after HOME; return failure never rolls back a physically released obstacle.
The suite reuses this commit receipt instead of submitting/appending it twice.

Kinematic release confirmation is read-after-write: require both a pose stamp
and receive timestamp later than the set-pose response, then confirm XY within
the existing2s budget. A recent pre-command cache value is not confirmation.
Persistent XY error still refuses at5mm, Z remains observational; rejected
candidates and exact command/read boundaries are evidence, not planning input.

The verify-anchored occupancy commit uses an independent `occupancy_commit`
gate on cargo mapper and its registered driver at both client and server.
Placement is not a dependency of committing a physically verified obstacle.
This does not bypass slot/frame/geometry/revision/verify validation inside the
mapper transaction; next placement retains full-graph readiness.
