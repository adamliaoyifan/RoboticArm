# ST08 generation 2 -- execution, scene and attachment

Parent `FRAME-MIGRATION-20260929`; dispatched owner `eng/cursor/grok-4.7`;
dependencies `ST03,ST07` (ST06 is inherited through ST07).
Updated 2026-09-30 after independent acceptance of ST07.

This is an updated generation-2 plan. It supersedes the
old draft's ST06 snapshot and unbound ST07 assumptions, not a claimed
implementation. The existing generation-1 thread remains preserved in history.
This plan does not claim ST08 or authorize hardware motion.

## Bound baseline and remaining release work

- Accepted product base: `8debda0c49438c7d8688e39fe5684dcf7fea546f`.
- Accepted ST06 ancestor: `074daf34c4df1c42bf2e3a7f58711e367ba72375`.
- Earlier repair ancestor: `dff51dc05aca25a4dbe17535b82124aaaa9e4ad6`.
- Acceptance: `docs/agents/reviews/2026-09-30_1726_st07-accepted.md` and
  `docs/agents/discuss/2026-09-30_1159_frame-stage-07-g2.md`.
  The later reviewer acceptance binds the repaired commit; do not select the
  older `904a6de` merely because the historical implementation Result names it.
- Exact-base audit: planning 678 passing, 171 subtests; contracts 15 passing;
  combined 693 passing, 228 subtests. Baseline checker 50/50; migration exit 2.
- Exact pending set: `F01,F15,F16,F17,F18,F21,F22,F23,F24`.
  After ST08: **`F01,F21,F22,F23,F24`**, migration exit 2.
- Proposed worktree `/tmp/elfin-frame-migration-st08`, branch
  `integration/frame-migration-st08-20260930`. Create it only at execution.
  Code must descend from the accepted product base plus the committed plan.
- The plan commit is assigned after this review is committed. The new dispatch
  records that exact SHA as `plan_revision`; it is not `c8ce801`.
  Do not release a dispatch whose plan remains uncommitted/ignored.

Before release, record the baseline packing result and exact failed-node IDs
at this base/environment. Prior packing/perception failures in older reports
are historical, not automatic waivers. Refresh owner/dependency snapshots and
create generation 2 through the lifecycle helpers; do not rewrite a claimed
requirement or edit the old queue row into a new generation.

## Scope and observations that change the old draft

CPU/Humble module verification only: no Gazebo, RViz, live MoveIt server,
hardware I/O, scored simulation, master merge, or shared product-worktree edits.
ST09 owns production profile switching and orchestration; INTEGRATION owns
actual motion-cycle validation. Keep v1 endpoints and wire layouts intact.

Source observations at the accepted base:

1. `SelectPlacePathV2` returns old `MotionSegment[] carry_segments` plus
   `PoseContractV2[] waypoint_poses`; `BuildMotionSequenceV2` returns old
   `MotionSegment[] segments` plus a typed `contact_pose`. An endpoint named
   v2 is not yet a typed motion producer. ST08 must connect these producers
   to its new consumer; a hand-written test-only segment is insufficient.
2. `_prepare_occupancy` converts centers to world but copies map quaternions.
   Test non-square boxes at 90 degrees and atan2(4,3), not just yaw 0 or pi.
3. `scene_manager_node._handle_add_placed` treats bare slot poses as world.
   Preserve this v1 behavior and add a typed v2 path instead of silently
   reinterpreting an unlabelled v1 request.
4. `_run_joint_path` reads positions, densifies knots, then constructs a new
   trajectory. It does not preserve supplied time/velocity/acceleration/effort.
   A generic namespace wrapper would silently lose the new joint contract.
5. `_run_pose_target` submits both the first IK solution and an alternate atlas
   queue. A verifier on the first solution alone leaves an execution bypass.
6. `BoundaryContext` has no target-link member; target-link expectation must be
   explicit in a new execution context, not read from a nonexistent attribute.
7. `/luggage/current_box` holds measured dimensions and identity, not a measured
   box pose. `_pull_gt_box` supplies privileged pose for sim physics. The old
   proposal to reuse an existing measured full pose in the controller is false.

Normative references: `docs/architecture/frame_contracts.md`,
`docs/architecture/privilege_boundary.md`, `docs/architecture/placement.md`,
`docs/architecture/perception_architecture.md`. Put pure math/admission in
focused ROS-free modules; ROS message conversion belongs in adapters. Avoid
adding all new behavior to the existing large executor/planner nodes.

## F15 -- typed motion producers, adapters and consumers

### Wire contract and producer closure

Create `MotionSegmentV2.msg`, `PlanMotionV2.action` and
`ProbeMotionSegmentV2.srv`; register them in `luggage_msgs/CMakeLists.txt`.

`MotionSegmentV2` has a nonspatial envelope: version, request_id, scene_revision,
stamp_valid, stamp_ns, time_basis, name, segment_type and target_link. Do not
invent a Cartesian frame on the joint envelope. Payloads are typed
`PoseContractV2 target_pose`, `PoseContractV2[] waypoints`, and
`JointTrajectoryContractV2 joint_trajectory`, discriminated by segment_type.
Add `bool policy_valid`, four boolean policy fields (`keep_tool_down`,
`keep_camera_down`, `lock_wrist`, `allow_ompl_fallback`) and finite
`required_cartesian_fraction` in [0,1]. The old draft incorrectly called all
five fields booleans. `policy_valid=false` rejects omitted policy on real ROS
messages, whose boolean fields otherwise silently default to false.

- Cartesian types are `pose_target` and `cartesian`. Active poses have motion
  profile: world, suction_contact_frame, tool_quaternion, metres. Request,
  scene and time basis agree with their envelope; stamps are present, not
  rewritten to now. Inactive joint payload must be the default empty message.
- `joint_path` has robot_state time basis, the existing joint contract, and no
  active Cartesian target/waypoints. Inactive default pose is ignored only
  because of this explicit discriminator; nondefault inactive payload rejects.
- Allowed names are exactly elfin_joint1 through elfin_joint6, once each.
  Permutations reorder every populated numeric vector, not only positions.
- target_link is suction_contact_frame. It is required even for a joint path
  because payload/collision attachment still binds to a tool; no Cartesian
  coordinates are inferred from that requirement.

PlanMotionV2 goal carries `MotionSegmentV2 segment`; result preserves the v1
result fields and adds `BoundaryDiagnosticV2 diagnostic`; feedback preserves
stage/segment_name/fraction. ProbeMotionSegmentV2 likewise replaces its request
segment type and adds a diagnostic to its existing result fields. Endpoints:
`/motion_planner/plan_motion_v2` and `/motion_planner/probe_motion_segment_v2`.
Malformed requests produce structured rejection, not callback exceptions.

Extend the two existing ST07 v2 service responses with
`MotionSegmentV2[] motion_segments`; retain their existing fields for their
existing tests. Update `place_service.py` to emit typed segments directly from
admitted producer records and explicit policy. Do not relabel arbitrary bare
Pose output as world merely because a caller chooses the v2 endpoint.
BuildMotionSequenceV2 also gains request_id, scene_revision and a typed
PoseContractV2 pick_pose for its pick branch. The latter agrees with the
DetectedLuggage pose/header and measured-support authorization; the place
branch keeps the typed slot and rejects a nondefault inactive pick_pose.
For Build's pick branch, use the admitted ST03 pick/candidate path and retain
its authorization and strict Cartesian-fraction policy. Do not silently fall
back from a missing selected suction candidate to the centered shortcut.

These additive v2 response fields **change ROS type support/wire identity**.
Rebuild all producer/consumer packages together and rerun ST03/ST07 suites on
that commit. Mixed old/new installed v2 types are not compatible. ST09 will
switch production profiles; this stage exercises both real v2 callbacks and
adapters with doubles. v1 service/action/message fields do not change.

### Admission and execution

Create focused `motion_frame_contracts.py` and `motion_frame_adapters.py`.
ExecutionContext wraps BoundaryContext plus expected target_link and explicit
expected planning frame. Expected scene identity comes from the loaded scene,
not from copying the incoming scene_revision. Request correlation comes from
that operation; each nested payload must agree. Canonical joints are asserted
against `motion_executor.JOINTS`.

Pure admission checks the entire record before any effect, using the existing
pose/joint decoders and diagnostic wrapper. Validate active and inactive
payloads, policy, pose/quaternion numbers, supported type/link, scene/time,
request identity, units and vector shapes. Unknown extra plain-record fields
reject. Geometry hash/map_revision are not fields of a motion leg; they remain
requirements of occupancy input, not identities manufactured on a segment.

Add execute_segment_v2/probe_segment_v2 entrances and node handlers. Admission
must precede readiness waits, occupancy injection, scene apply, seed queries,
IK, planning or execution. Catch BoundaryContractError at ROS boundaries and
return its diagnostic. Probe is plan-only with zero execution/valve calls.
Delegate only after admission, preserving cancellation, start joints, boundary
measurements, flags and fraction thresholds. Do not bypass execute_segment's
state initialization/finally cleanup by calling its inner method directly.
Reject an executor whose configured frame/link differs from the context.

Joint-path decision: v2 JointTrajectoryContractV2 is a **timed trajectory**,
not the old position-only selector-knots carrier. Preserve supplied point
order, times and optional vectors after canonical joint-name permutation.
Perform continuity, wrist and occupancy validation without overwriting its
wire trajectory. Use a focused v2 trajectory adapter/submission path; do not
route it through read_joint_knots/_robot_trajectory or implicit retiming.
Unsupported timing/policy rejects before controller submission. Keep v1 knot
retiming unchanged. A future producer needing retimed knots must explicitly
produce a valid timed v2 record, not fill arbitrary placeholder timestamps.

## F16 -- model frame, compensation and FK verification

Keep offline_ik root discovery and atlas files unchanged. The accepted URDF
has a world root and a world_base joint to elfin_base_link. Inspect actual
model transforms; the string elfin_base_link alone is not a conversion.

Use the existing strict ST07 container-transform admission where applicable:
container_link -> world, finite translation, unit quaternion, explicit static
null or nonnegative stamp. Apply require_horizontal before occupancy or
compensation effects. Frame, stamp and numeric failures must be caught before
selection/planning, including paths where there would be no winning solution.
Use transform_pose for centers **and orientations**. Verify the existing
world_to_map path against an independent inverse; do not force the forward
mapping to compensate for an unmeasured inverse error.

Lift offsets rotate as vectors (no translation). query_offset_frame supports
only container_link, world and elfin_base_link; default container_link preserves
the existing call. Convert world/base vectors using the explicit admitted
scene/model rotation before atlas_seeds_for_query. Unknown frame rejects;
never add an origin to an offset. Preserve existing payload/scene atlas identity
gates: stale identity produces no seeds and zero refinement calls.

FK guard covers **every joint goal derived from a v2 Cartesian target**:
first IK solution, queued alternate, refined seed and Cartesian fallback that
submits a joint goal. Verify before each relevant planning/controller call.
Do not use one mutable global v2-enabled flag that a concurrent v1 request can
inherit. Carry per-call admitted context, or serialize executor access with
cleanup on failure/cancel; test v1-after-v2 and interleaved-request isolation.

A verifier returns full TCP position and quaternion in world for the named
joints/tool. Missing verifier or unusable/nonfinite FK returns
FK_VERIFY_UNAVAILABLE; error >0.002 m Euclidean or >0.02 rad shortest rotation
angle returns FK_VERIFY_BUDGET. No rejected goal is submitted. Equality at the
threshold passes. An out-of-budget alternate must not bypass the first guard.
Use constructed FK outputs just below/at/above each threshold; an arbitrary
0.05-rad joint perturbation is not proof that a Cartesian budget was breached.
The admitted record remains valid; these are kinematic failures with a reason,
not frame-admission failures. v1 behavior and its existing tolerance remain.

Occupancy box repair composes container orientation with each box quaternion
and transforms its center. At yaw zero existing geometry stays identical; at
90 degrees/atan2(4,3) compare all eight world corners independently. Preserve
measured payload selection and map geometry/revision checks. No GT occupancy.

## F17 -- world collision objects and typed placed-box services

Keep measured-support authorization in pickup_collision_aabb; a finite positive
size alone is not authorization to construct a pickup box. TOP_ONLY/configured
or catalog height remains rejected. A detection uses world acquisition data;
a placed slot uses container_link scene data. After slot conversion it remains
scene-stamped: do not force it through the acquisition-only detection profile.
Use separate admitted constructors for these two sources, then one world-box
builder (box_center, box_axis_pi, body-axis WDH).

Create AddSceneBoxV2.srv: request PoseContractV2 slot, size_wdh[3], box_id;
response success/message/BoundaryDiagnosticV2. Endpoint ~/add_scene_box_v2.
Validate request, positive finite dimensions, box_id, expected node scene hash,
horizontal scene, transform direction/numbers/stamp before scene calls.
Create RemoveSceneBoxV2.srv: request box_id; response success/message;
~/remove_scene_box_v2. Empty id refuses without calling the scene client.

Typed scene builders set world explicitly. Attached object pose is expressed
in the declared tool link, not a world pose merely labelled as link-relative.
Keep v1 add/remove behavior, default attach/touch links and wire layout. Preserve
all typed fields in regression comparisons; raw CDR padding is not geometry.

## F18 -- measured attachment record and physics separation

Put admission/composition in ROS-free `attachment_frame_contracts.py`, wire
conversion in `attachment_frame_adapters.py`. Backends consume immutable
admitted records. Do not add another set of hidden frame assumptions to
vacuum_attach_utils' existing ROS helper functions.

Create VacuumCommandV2.srv at /vacuum/command_v2. Request carries enable,
request_id, scene_revision, box_id, uint64 generation, DetectedLuggage measured_box,
PoseContractV2 box_pose, and bool has_grasp/PoseContractV2 grasp. On enable,
box_pose is a measured world box-center acquisition record. It must agree with
measured_box's pose, header frame/stamp and id; dimensions and support-height
authorization come from measured_box. Request/scene/instance identity must
match the admitted operation and controller instance generation. The supplied
grasp uses the already-established ST07 box-relative transport convention.
Response carries success/message and BoundaryDiagnosticV2.

The v2 enable input is a correlated **attachment snapshot**, not an arbitrary
earlier detection reused after the robot has moved. Resolve suction_contact_frame
in world at that box acquisition stamp, with no latest-TF fallback, and record
the resolved stamp. Its selected-grasp check binds the contact to the measured
box. If the caller cannot provide this snapshot, the v2 enable path is unavailable
until ST09 supplies it; do not fabricate it from a dimensions-only topic or GT.
This computes the grasp before attachment from correlated box/tool geometry. Freeze it per request
and instance; a measurement from another generation is never reused. On disable,
use the frozen attachment and a newly measured final tool pose; ignore empty
attach-only request payloads. Missing final pose does not invent a box origin.
Current caller/profile activation remains ST09; the v2 callback itself and its
measured input contract are implemented/tested here. v1 /vacuum/command remains.

Compute measured T_box_tool using ST03 grasp_transform; its inverse locates the
box in tool axes. A supplied grasp must agree within 0.01 m Euclidean and
0.10 rad shortest rotation angle (inclusive). q/-q agree; yaw 0/pi differ.
Without supplied grasp, freeze the measured relation. Follow and detach planning
records compose this same relation with the tool's new world pose. Missing or
inconsistent identity rejects before scene/attach effects. For missing final
pose, mark last_detach_record accepted=false/MISSING_IDENTITY; do not commit a
fabricated planning pose. Existing hardware release semantics still execute
DO release in their original order; this exception is distinct from refusing
an invalid enable request. All hardware calls are spies in this task.

Keep **two provenance-separated contexts** in simulation:

- Planning attached geometry and final planning pose use measured box pose,
  measured dimensions and the frozen measured relation only.
- Contact/retention/Gazebo follow may use physics GT, pulled for the matching
  generation. Its physical relation is stored separately and never substituted
  into the planning attached object. Moving GT while holding measured input
  fixed must leave every planning-scene field unchanged.

Do not require measured and GT relations to be numerically equal. Physical and
planning objects can differ by measurement error. Within each context, its own
attach/move/detach relation must be preserved. Convert any panel-link physical
pose to/from suction_contact_frame with the declared fixed TF, not name aliasing.
Hardware attach validates the v2 record before _set_grasp; valve bit order is
unchanged. Stub remains state-only. No live driver/orchestration changes.

## Files, proof ownership and predecessor revalidation

New files: three motion interfaces; AddSceneBoxV2/RemoveSceneBoxV2;
VacuumCommandV2; the four focused contracts/adapter modules above; and
`test_frame_migration_execution.py`. Update CMake registrations and manifests
for actual imports. Extend the two ST07 v2 responses and place_service producer.
Update executor/planner, scene client/manager, pickup admission, vacuum backend
and controller at their boundaries. Extract helpers rather than impose an
arbitrary line-count/diff cap. Implementation also updates closest normative
docs if interface representation changes; it must not contradict them.

Proof source lists must cover every behavior exercised, including endpoint
wiring and producer conversion; do not optionally omit shipped source files:

| Row | Retain existing sources; add |
|---|---|
| F15 | MotionSegmentV2, PlanMotionV2, ProbeMotionSegmentV2, motion contracts/adapters, motion_planner_node, place_service and the two extended ST07 service definitions; any new producer helper |
| F16 | motion contracts/adapters and motion_executor when they own FK/compensation; retain motion_planner_node/offline_ik |
| F17 | AddSceneBoxV2, RemoveSceneBoxV2 and any new world-box builder; retain scene_manager_node, planning_scene_client, pickup_collision |
| F18 | VacuumCommandV2, attachment contracts/adapters, vacuum_controller_node; retain vacuum_backend, vacuum_attach_utils, planning_scene_client |

Source edits may invalidate previously accepted F08/F09/F11/F12/F13 proofs.
Before editing, derive the affected-row set from frame_interfaces.json; rerun
those rows' actual suites and reissue hashes on the final commit. Their states
do not change. Never remove a source to evade a stale hash. Hash all row sources
plus CONTRACT_SOURCES and regenerate interface_contract_sha256 after extending
sources. Only F15-F18 change state to verified. F01 and F21-F24 remain pending.
Do not modify ST06 atlas artifacts/runtime/query logic to fix a consumer test.
If an unexpected shared-source repair is required, record it and revalidate its
owners before claiming closure; do not silently bypass the exception.

## Acceptance matrix and observable gates

CPU/Python 3.10/ROS Humble; no motion servers or hardware. Generated real
messages, CDR encode/decode and real callback methods use bounded fake clients.
Use scoped transport doubles, not mocked admission/geometry helpers. Preserve
actual call arguments and counts for wait, scene, seed/refine, IK, planner,
controller, attach, detach and valve boundaries. No source-text assertions.

The geometry matrix is 4 yaws {0,pi/2,pi,atan2(4,3)} x 2 origins
{(0,0,0),(1.5,-0.4,0.2)} x 2 floors {0.53,0.71} x 2 support heights
{0,0.30} x 2 box-axis grasp offsets {(0,0,0.16),(0.10,-0.05,0.16)}.
WDH=(0.80,0.50,0.32); slot local yaw=0.3. Exactly 64 unique case IDs for
each of execute, collision and attach (192 required rows). Independent SciPy
or test-local matrix oracle, not a production helper called twice.

| Gate | Required observation |
|---|---|
| Execute, 64 | Actual ST07 typed producer -> wire -> real admission/callback -> captured MoveIt goal; world position/quaternion/link match oracle |
| Collision, 64 | Pickup, placed-slot and injected occupancy representations match eight body-axis world corners; non-square yaw cases catch missing rotation |
| Attach, 64 | Measured attach -> nonzero translated/rotated tool move -> detach preserves relative pose; planning and physics contexts tested separately |
| Numeric tolerance | Pure position max-component error <=1e-9 m; quaternion components after sign alignment <=1e-9; CDR path <=1e-6 for each |
| Pose semantics | q/-q equivalent; tool pi distinct; vector offsets never translated; tilted container 0.2 rad rejected |
| Joint trajectory | Canonical permutation preserves all positions/optional vectors and time_from_start exactly through captured controller request; duplicates, missing/extra names or invalid times reject |
| FK | Below/at/above each 0.002 m / 0.02 rad limit; missing/nonfinite verifier; bad alternate after a good first IK; zero submissions of rejected targets |
| Request isolation | v2 then v1, failure then success, cancel, and interleaved contexts do not leak verifier/scene/request/attachment state |
| Probe | Same admission and identities as execution; zero execution/valve calls for both success and rejection |
| Provenance | Change GT while measured snapshot fixed: planning output identical; stale box generation cannot attach; absent measured support refuses |
| v1 | Fixed real v1 callback requests match accepted base on all typed fields, exact strings and sequence order; existing yaw-only v1 expectations retained |

For each negative below, one plain-record test plus one generated-message
callback case where representable. Assert the named code and zero downstream
work; counts are observed spies, not hard-coded returned counters. Inactive
ROS payloads and omitted policy/stamp validity get actual wire tests.

| Negative | Code |
|---|---|
| Empty frame/request/scene/link or required pose | MISSING_IDENTITY |
| Wrong pose/transform frame or transform direction | FRAME_CONTRACT_MISMATCH |
| Wrong reference / orientation semantics | POSE_REFERENCE_MISMATCH / ORIENTATION_SEMANTICS_MISMATCH |
| Wrong version / units / time basis | CONTRACT_VERSION_MISMATCH / UNIT_MISMATCH / TIME_BASIS_MISMATCH |
| Stale scene / mismatched nested request | CALIBRATION_REVISION_MISMATCH / REQUEST_IDENTITY_MISMATCH |
| stamp_valid=false, negative/null-required stamp | INVALID_STAMP |
| Nonfinite position/translation, nonpositive size, fraction outside [0,1] | INVALID_NUMERIC_PAYLOAD |
| Zero/nonunit quaternion | INVALID_QUATERNION |
| Unknown segment type / wrong target link | UNKNOWN_SEGMENT_TYPE / TARGET_LINK_MISMATCH |
| Missing policy_valid or malformed boolean policy | INVALID_NUMERIC_PAYLOAD |
| Joint names / vectors / times invalid | INVALID_JOINT_NAMES / JOINT_VECTOR_SIZE_MISMATCH / INVALID_TRAJECTORY_TIME |
| Cartesian labels on plain joint record | JOINT_SPACE_LABEL_MISMATCH |
| Nondefault inactive payload | INVALID_RECORD |
| Tilted container | UNSUPPORTED_CONTAINER_TILT |
| Supplied grasp beyond measured tolerance | GRASP_BINDING_MISMATCH |
| Extra map identity on a plain motion leg | UNSUPPORTED_RECORD_FIELD |
| Missing measured support | DETECT_FULL_GEOMETRY_REQUIRED |
| Attachment instance/generation mismatch | GRASP_BINDING_MISMATCH |

Dynamic transform stamp mismatch returns TRANSFORM_TIME_MISMATCH, the existing
frame_transforms.transform_pose code. Test it before downstream calls, including
no-winner paths; do not replace a failed stamp check with latest TF. Cargo map revision/hash negatives
are inapplicable to a standalone motion message, but remain mandatory at the
occupancy-service input. Distinguish FK failure and release-without-final-pose
from invalid enable requests when evaluating side-effect counts.

Run each deterministic case once; require all required cases pass. This AND
means 100% conformance over the specified inputs, not an estimated per-unit
robot success probability. Inject position-only transform, pi-fold, dropped
joint vectors, skipped alternate FK and GT-to-planning contamination mutants
in the harness; each relevant oracle must detect its mutant. This demonstrates
that the gate distinguishes those broken behaviors. Missing row/capture is
not_evaluated and blocks completion; never treat zero samples as a pass.

## Verification procedure and revision rules

Before product edits, run baseline at the accepted SHA in an isolated clean
checkout and save collected node IDs/counts. Planning threshold: all 678 base
cases remain plus all new cases; contracts all 15; packing no new failure.
New stage case IDs must be present, not merely an increased aggregate count;
retain the exact collected base-node set and report parameterized subtests.
No skip of a required stage case. Any baseline environment failure must be
reproduced with identical node ID/cause on base and candidate, or repaired;
older counts alone cannot justify excluding a failure.

Commit product sources before acceptance. Record commit, dirty status, plan
SHA, loaded module paths, ROS type-support paths and configuration hashes.
Evidence-only commits may follow, but their recorded source hashes must match
the final committed tree. Run baseline and candidate with the same build and
Python paths. No installed overlay from the dirty primary workspace.

```bash
# From the isolated stage checkout; choose a NEW evidence run directory.
source /opt/ros/humble/setup.bash
colcon --log-base log/st08 build --build-base build/st08 \
  --install-base install/st08 --packages-up-to \
  elfin_description elfin_control luggage_description luggage_msgs \
  luggage_perception luggage_packing luggage_planning luggage_gazebo
source install/st08/local_setup.bash
export ROS_DOMAIN_ID=87
export ROS_LOG_DIR="$PWD/log/st08-ros"
export FRAME_MIGRATION_CASE_LOG="$PWD/EVIDENCE/st08-run/cases.jsonl"
mkdir -p "$PWD/EVIDENCE/st08-run"
python3 -m pytest src/luggage_planning/test/test_frame_migration_execution.py -q
python3 -m pytest src/luggage_planning/test -q
python3 -m pytest src/luggage_description/test/test_frame_contracts.py -q
python3 -m pytest src/luggage_packing/test -q
# Also run exact suites for every predecessor row invalidated by changed files.
python3 scripts/check_frame_contracts.py --mode baseline --output EVIDENCE/st08-run/baseline
python3 scripts/check_frame_contracts.py --mode migration --output EVIDENCE/st08-run/migration
```

Launch this procedure in a shell with no workspace overlay sourced; persist
stdout/stderr and each command exit independently (do not hide failures behind
the last command). Use pytest for batteries: unittest discovery misses pytest
functions/fixtures and does not reproduce the accepted count. Build dependencies
include elfin_description AND elfin_control for FK/xacro tests. For source-path
imports, enumerate this checkout's package roots explicitly and record them;
never fix a missing dependency by silently sourcing another workspace install.

Test files already present at the accepted head include
`test_frame_migration_selector.py`, `test_select_place_path_service.py`,
`test_trajectory_selector.py`, `test_occupancy_place_paths.py`,
`test_waypoint_generator.py`, `test_frame_migration_atlas.py`,
`test_joint_path_execute.py`, `test_motion_atlas_seeds.py`,
`test_motion_planner_occupancy_scope.py`, `test_vacuum.py` and
`test_vacuum_attach_utils.py`. They are included by the full planning battery.
Changed ST03 or other predecessor sources additionally require their indexed
suite(s); derive that list from inventory/proof commands before editing.

Compatibility uses full typed-message equality, exact rows_json text and list
order, as clarified in ST07. Preserve raw CDR as evidence but not as an equality
gate: padding can differ. Retain ST07's existing comparison test; for new ST08
comparisons prefer deterministic typed fields/schema-aware canonical encoding.
Four-copy stability is not proof of which bytes are padding. Inject whitespace
and reordered segments to prove the comparator does not erase real differences.

## Evidence and completion

Evidence under `docs/status/evidence/frame_migration/ST08-<run-id>/`, with fresh
attempt directories; cross-agent role notes/mailbox remain in the primary
workspace. Manifest includes source/plan revisions, environment, commands/exits,
case counts, stages, stamps, diagnostics, effect counters, geometry oracle and
actual errors, file sizes, missing artifacts, capture_complete/replay_possible.
No raw logs under docs/agents. Flush failure payloads before another attempt.

| Boundary | Trace and failure capture | Window / budget | Replay |
|---|---|---|---|
| Producer -> motion/scene/vacuum wire -> admitted record | Input/output CDR + typed JSON, request/scene/frame/link, diagnostics, all client spy arguments | One complete case; 64 MiB/run total wire data | Exact pytest node ID on recorded commit with saved inputs |
| Geometry/FK/attachment | Independent oracle, measured vs physics inputs separately, predicted/actual pose, seed and alternate index, tolerances and counts | One complete case; 16 MiB/run algorithm data | Exact pytest node ID and frozen fixture |
| Build/test/checker | Build logs, collected IDs, command exits, module/type paths, source hashes, durations and resource summary | 16 MiB/run | Recorded environment and commands |

Total cap 96 MiB/run; retain baseline, final and failed attempts (campaign cap
384 MiB, then stop new runs until retention is explicitly resolved). Do not
truncate replay-critical data to satisfy a cap. Capture-health test must write
and reload one negative bundle, join its input/output/counters, and verify its
oracle before acceptance. No real-time performance threshold is claimed; record
capture overhead/duration separately from test verdicts. Missing data blocks a
passing proof. Proof JSON itself remains below the checker's 1 MiB limit.

Completion requires: all observable gates; unchanged v1 contracts; zero new
baseline regressions; reissued affected predecessor proofs; F15-F18 verified;
exact five-row pending set; commit evidence and role note; lifecycle completion
on the newly dispatched generation. A module pass is not a live ROS-chain or
robot certification. F01 closure and ST09/INTEGRATION dispatch remain separate.

## Release checklist

1. Bind this reviewed plan to a committed revision on the accepted lineage.
2. Record missing exact-base packing baseline/waivers and affected predecessor
   proof map. Confirm the new measured v2 vacuum input and v2 producer response
   changes are included in the implementation/file acceptance scope above.
3. Create an immutable generation-2 ST08 thread with owner eng/cursor/grok-4.7,
   dependencies ST03/ST07, accepted product SHA and exact plan SHA. Supersede
   generation 1 through the lifecycle workflow, preserving its history.
4. Set dispatch_ready=yes only for that generation-2 handoff. The owner must
   still run the exact-base baseline gate above before product edits.
