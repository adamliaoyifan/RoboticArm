# Production Orchestration

This document defines the production loading-cell orchestration contract. The
same state machine and exploration contracts apply in simulation and on
hardware. Adapters may differ by backend; authorization, state progression, and
online map inputs may not.

## Startup Boundary

`WAIT_START` is the initial state. Launch completion, lifecycle activation,
service readiness, sensor readiness, timers, and ordinary `run` / `step`
commands must not authorize robot motion, vacuum changes, or detection.

The only initial authorization event is an explicit operator `start` command.
After it is accepted, the orchestrator establishes the cargo-map session before
prompting for a placed box. User ruling2026-11-12 permits a declared initially
empty ULD to skip initial container exploration, under the source/provenance
contract in real_scenario.md. An undeclared or retained-content ULD still needs
sufficient observation. A reset alone is never an empty-container declaration.
The explicit-start diagnostic driver initializes this declared empty session
through the O owner and has no mandatory portal motion. Optional diagnostic
views remain explicit; normal pickup observation is retained.

For subsequent boxes, prefer qualifying RGB-D or LiDAR observations from normal
post-release exit/return stops; add a collision-checked observation pose only
when relevant coverage is insufficient. A fixed portal pose is not a production
requirement. LiDAR integration requires its actual range/frame/time semantics,
not relabelling Cartesian laser points as optical-Z depth. Sensors share one map
owner and source resolution. Startup and execution authorization stay unchanged.

The observed three-box simulation profile uses 2cm source resolution and 2cm
lateral O obstacle expansion. It adds no extra vertical O expansion: the complete
source voxel height remains blocked, and the existing attachment envelope retains
payload height uncertainty. This explicit policy must be identical during prepare
and execution recertification. Isolated g2 replay showed vertical dilation raising
floor voxels into the carrying envelope; removing only extra vertical dilation
restored the inward local branches while retaining every occupied/unknown voxel.
The complete trajectory and actual placement still require validation. Other
profiles retain their explicitly configured collision-margin policy.

## Full-operation simulation adapter

`luggage_gazebo` selects `planning_pipeline=joint_operation` explicitly through
its grouped profile. The legacy `frame_runtime` peer graph is a separate
compatibility mechanism, not the complete-operation identity or activation flag.
The new mode composes one O owner, measured optical rays, common placement,
prepare/native services and the bound execution consumer. All model readers use
the same expanded full URDF/SRDF. No automatic startup FJT hold is issued in this
mode. A diagnostic client requires explicit `--start` before map reset, observation
motion, detection or cycle execution. Gazebo spawn output is physics-only; only
measured detector and O outputs populate online planning requests. Legacy G and
segmented selector owners must not coexist with this O composition.

For diagnostic observation repair, `--view-poses` may supply explicit JSON goals
using `diagnostic_observation_poses_v1`, bound to the exact model SHA256 and scene
revision. The runner archives this input. These goals override observation names
only; MoveGroup must still plan and execute each empty-load motion, and actual
settled RGB-D/O evidence must establish visibility. Offline IK or static-state
collision success does not establish a usable view or permit filling unknown
space. Such diagnostic goals are not calibrated real-cell poses.

## Operator Request Correlation

The orchestrator publishes each operator prompt with a generated request ID,
prompt kind, current state, and display text. Prompt messages must not contain
box geometry, pose, identity, dimensions, or simulation truth.

An operator pickup-ready event authorizes one cycle only when it carries the
exact current request ID and the expected schema version. Missing, blank,
stale, duplicate, malformed, or wrong-ID confirmations must not emit motion or
detection effects.

## Pure State Contract

The state reducer is a ROS-free contract. It may consume plain decoded events
and return declarative effects, but it must not call ROS, sleep, read YAML,
lookup TF, inspect Gazebo, or execute motion.

Effects describe work for a node layer, for example `CALL_ACTION`,
`CALL_SERVICE`, `PUBLISH_PROMPT`, and `PUBLISH_STATUS`. Effects that may move
the robot, change vacuum, or trigger detection must mark that capability
explicitly so tests can assert startup and request-correlation safety.

## Required Cycle

The required nominal progression is:

```text
WAIT_START
  -> RESET_CARGO_MAP
  -> EXPLORE_CONTAINER
  -> RETURN_PICK_OBSERVE
  -> WAIT_PICKUP_READY
  -> DETECT
  -> COMPUTE_PLACEMENT
  -> PLAN_PICK
  -> EXEC_PICK
  -> PLAN_PLACE
  -> EXEC_PLACE
  -> COMMIT_AND_VERIFY
  -> RETURN_PICK_OBSERVE
  -> WAIT_PICKUP_READY
```

Every return to `WAIT_PICKUP_READY` must happen after reaching
`pick_observe_pose`. The cargo map persists across placed boxes and is updated
only from live depth integration and explicit placement commits.

## Failure Boundaries

### Bound complete-plan consumer

The independently prepared complete-operation path uses
`JointOperationExecution` as a pure effect/state contract. It requires consumer
identity/digest admission and live start validation before issuing motion. Its
node adapter dispatches each correlated effect once. An `execute_leg` effect
consumes the original frozen timed path, without J6 edits or replanning.

Before carrying, `confirm_attachment` must produce a valid, confirmed measured
attachment from the existing attachment provider. Then `validate_attached_suffix`
checks provider freshness, actual q, actual frozen geometry, current scene,
certificate liveness and intended placement under the actual grasp. A mismatch
may pass only through that actual-geometry revalidation; refusal enters a carry
fault for bounded repair/recovery. It must not rewrite the prediction or release.

Before retreat, release confirmation must precede application of the released
box as a world obstacle and retreat validation. A failed release confirmation
retains payload responsibility. A confirmed release followed by blocked retreat
stops motion. Cancellation during controller execution waits for its terminal
result; transport errors alone cannot establish that motion has stopped.
Duplicate/stale callbacks issue no new effects.

This consumer has no ROS, global planner or vacuum dependency. Production node
adoption must implement and verify its effects against the native scene and
controller; pure consumer tests do not certify that adoption. Existing startup,
operator correlation and map ownership requirements apply to the complete-plan
path as well as the nominal cycle above.

The motion-free preparation core appends a `preparation_search_evidence_v1`
receipt to its response trace, bound to the measured top-domain digest. It
separates proposal work, complete quality refusals, incomplete quality receipts
and candidates yielded to the native planner. A yielded candidate does not prove
the native solver attempted it: shared work caps and deadlines remain effective.
Binary unproven support cells do not distinguish missing depth from off-face or
defective support. This receipt therefore leaves observation deficiency
`NOT_ESTABLISHED`; it cannot authorize an automatic observation movement or
convert finite search failure into physical unreachability. Pre-core missing
inputs retain their separate node-level reason/evidence.

`NativeOperationRevalidationPort` supplies the read-only native portion of
these validation effects. It accepts only the original canonical six-leg chain
and exact pick/pick/carry/carry/carry/retreat phases. Every current capture must
bind the same scene revision, model, geometry and map revision, with explicit
six-joint winding consistent with both the original leg start and the captured
robot state. The current world's owned payload must actually match the admitted
load phase; reconstructing private phase scenes cannot hide a missing, duplicate
or changed live attachment. Unrelated obstacles survive.

The port checks original release FK under the actual attachment against the
bound nominal box target, using the existing endpoint tolerances. The configured
release gap determines a predicted settled box pose from that grasp; this is not
an observed placement and cannot authorize an OCC commit. Only numerical pose
equivalence from the operation contract canonicalizes floating representation.
The caller must explicitly provide issued-certificate liveness, current source /
attachment freshness, whole-footprint actual-contact admission while carrying,
and current settled-box occupancy admission. A predicted grasp approval cannot
stand in for actual-contact approval after an attachment change. Each must
return literal true; absent, stale, exceptional or incomplete answers refuse.
Certificate and source liveness are checked again after native validation.

The native request retains original knots and controller-interpolated samples
of every remaining leg in full private phase scenes, including the released
world obstacle. Compatible changed geometry can pass without global replanning;
a target miss, collision, deadline or identity change yields no accepted receipt.
When capture is enabled, a native suffix refusal also records a bounded summary
of failed leg, phase-scene index, first invalid sample/TCP and contact pairs with
their matched world/attached geometry. Preserve the full request/response CDR.
The summary reports model collisions; object names and contact enums establish
neither physical contact nor cargo ownership and authorize no obstacle removal.
The receipt retains the original plan/certificate binding plus a separate native
validation digest. The port has no controller, vacuum, planner, scene-write or
map-write capability. Production adapters still need to implement the required
owner callbacks and consume its receipts before actual controller submission;
native fixture/consumer tests do not certify that adoption.

The preparation owner retains a bounded process-local issuance registry only
after the final complete response has been captured. Its read-only
`/joint_operation/check_prepared` service decodes the submitted plan against
the independently retained identity/digest. An arbitrary nonempty certificate
field is not issuance evidence. Checks do not renew retention. Restart, expiry,
capacity eviction or a successfully replaced plan makes the old plan fail
liveness. Failed preparation or response capture cannot issue a plan. The
default retention is 16 plans / 16 MiB canonical payload / 300 monotonic seconds,
with explicit node parameters; these are storage limits, not geometric or
observation thresholds. A liveness success still requires every current-source,
actual-contact, occupancy and native suffix gate before controller submission.

Production preparation first retains its exact measured acquisition through the
perception-owned frozen observation service. The initial domain is selected by
the request's exact box/generation/acquisition/frame/model key from a bounded
published-domain history, rather than whichever stream message arrived last.
This preserves an already-selected observation across map RPC latency; it does
not substitute geometry, renew timestamps or exempt freshness/quality checks.
Perception resolves that domain's digest to the exact original quality samples.
Unavailable, invalidated or conflicting handoff inputs refuse preparation.
An unavailable domain and an unavailable current joint state have distinct
refusal reasons; if both are absent the boundary records both. Input
unavailability alone does not establish an observation-coverage deficit and
must not trigger an automatic camera motion. The failure archive retains the
requested acquisition, individual availability and history clear provenance.

Candidate quality queries use the independent
`EvaluateFrozenSuctionV2` service; the existing current-frame service is unchanged.
Pre-pick candidate recovery preserves the admitted target while trying distinct
observed contact centres and native IK/opening branches. Primary directed
contacts are evaluated first; alternative quality queries are deferred until
their candidates are needed. Per-heading geometric/contact work limits and the
global native candidate/branch/deadline limits remain explicit. Already-admitted
headings at exactly one position are tried together without changing targets or
inventing rotations. An exhausted budget is incomplete search, not proof of
unreachability. No part of this search permits changing an attached grasp.
On `SEARCH_INCOMPLETE`, preparation may return `SuctionSupportProbeV2` hypotheses
for failed geometric contacts, bound to the exact domain/model and already
admitted placement. Only actually checked failed centres are ranked by missing
support; a too-small observed grid may instead carry its centre as an explicit
hypothesis. These records cannot be converted to accepted contact queries and
never authorize a grasp or observation motion. Success and other refusals return
no probes. Ordinary successful preparation makes no raw-view RPC or extra motion.

Pre-pick observation assessment separately consumes the exact acquisition-bound
camera inputs. It rasterizes the complete projected physical footprint, including
partial pixels and out-of-image regions. Missing depth inside the consumed region
or clipping by the actual image may establish a conditional information need;
mask exclusions and finite-depth quality/IK/backend failures alone do not. Bbox
provenance remains explicit. Assessment never fills the support grid. Camera
hypotheses use caller-supplied sampling, actual intrinsics/plane/camera distance,
explicit optical-Z range and configured work bounds; no production motion count
or pose defaults are implied. Predicted visibility is only a ranking heuristic.
Each view still requires full-model, payload/ULD/O-scene motion validation,
settled measurement and conditional unique same-box association before reuse.
Instance ID equality alone cannot establish association; ambiguous acquisitions
remain separate. This pure proposal layer does not implement those execution
and association gates.

Native pickup-view preparation derives the optical/tool mount from the captured
URDF's same six-joint rigid sensor branch. It uses a full private copy of the
caller-admitted O/static scene and a measured stationary pickup box, forbidding
all robot-box contact during observation. Existing attachments or conflicting
object association reject. Other obstacles, unknown-space collision objects,
meshes, padding and unrelated ACM entries survive. The owner supplies static
world admission and current-source checks; a pure Snapshot type alone does not
prove that its O/ULD contents are complete or current.

A view path requires exact-target native IK, private-scene connection, timing
that retains checked polyline corners, and independent native certification of
the controller-interpolated path. Native certification checks velocity and
acceleration extrema inside each controller interpolation interval, as well as
knots; rest at knots does not prove an interval is within limits. Linear, cubic
and quintic selection follows the installed controller's lowest-common endpoint
derivative fields. Positive request limits apply unchanged. Explicit search/time/sample/velocity limits
apply. Refused, partial, cancelled or source-revoked work returns no plan. Its
certificate binds the original source, derived scene, measured acquisition,
camera/tool target and starting joints. This is plan-only evidence: dispatch
still requires current-source/start validation and terminal-result ownership.
Later camera frames cannot substitute samples or replace the retained acquisition
identity. After certification and the final full-scene read, preparation checks
the same non-renewing owner token, live six-joint start and pinned confirmed OCC.
Unavailable, revoked, expired or changed-model evidence cannot issue PLAN_READY.

`/joint_operation/prepare_pickup_view` is the read-only O adapter for this
prerequisite. It accepts only the exact request/domain/query retained from this
owner's `SEARCH_INCOMPLETE` response, under explicit non-renewing resource limits.
It reads the exact raw acquisition, checks a localized decision-relevant gap,
re-admits the original placement on latest O content in the same epoch, and uses
the same full static-world admission and collision-margin policy as whole-plan
preparation. Explicit camera sampling, optical range, IK and time budgets are
search inputs, not calibrated uncertainty. Live joints, frozen observation,
camera evidence, static scene and O content are checked again before issuing
the independent typed view receipt. Both preparation services share one lock.
Absent optional view context cannot change an ordinary plan result. G is an
explicit unsupported recovery backend until its lifecycle adapter is qualified.
This service does not dispatch motion, imply a new observation, establish same-box
association, fill support or change the sealed grasp. Its receipt needs a future
execution-owner lookup and fresh certification; six-leg execution accepts only
its existing complete-operation type.

Issuance stores the immutable contact binding with the exact issued plan and
counts its metadata in the bounded byte budget. A duplicate publication cannot
replace or drop that binding. The read-only issuance check returns this owner
provenance, independently of the submitted plan. It still does not assert
observation liveness or physical seal; execution must check the actual perception
and attachment owners before native/controller admission. A stored binding is
not a new view, a pose correction or a reason to skip any execution gate.

The original O source is also retained independently with the issued plan:
epoch, map/observed revisions, acquisition stamp, hull/config digests, clock,
validity generation and full content digest. CheckPreparedOperationV2 returns
typed OperationMapSourceV2; duplicate issuance cannot replace or drop it and
its bytes count against retention. The default preparation root reads one O
lease for both surface and volume and rechecks a fresh lease before issuance.
Changed map content needs fresh native validation before execution;
changed epoch/config/model/hull invalidate the original source association.
Legacy explicit G has an unbound O source and cannot enter an O-only consumer.

A later O lease remains current for an original source when epoch, validity
generation, hull/config digests, clock and map_revision match, observation
revision and acquisition stamp have not gone backwards, and the owner's live
lease is valid and within the configured age limit. Repeated observations alone
do not require recertification. O currently advances map_revision on every
changed cell value, including probability changes; this conservatively triggers
recertification even when the collision classification remains unchanged.
Each page of one lease still requires its entire exact snapshot identity.
Retain the original source acquisition stamp and provenance; record the later
lease separately. Global observation activity does not renew the age of an
unobserved region or establish per-region freshness. Regional age admission
requires spatial coverage evidence and is not supplied by this version check.

The ULD static-world premise is conditional: during a normal operation only the
current cargo moves. Contact by the robot or current cargo can displace other
cargo and invalidates that premise. A collision/protective-stop indication,
execution fault, or observed displacement must revoke continuation under the
old world assumption. Request stop and retain motion ownership until a real
terminal result; preserve suction if still carrying. Do not automatically
resume the original suffix or reverse it as a recovery motion. Recovery needs
fresh observation and a separately admitted plan using the actual fixed
attachment (if still sealed). A successful geometric check or absence of an
error message does not establish absence of physical contact.

Any consumer of a cycle-frozen background map must state how that premise is
monitored, including signal ownership, timestamps, loss of signal and occluded
cargo. Robot current or torque availability cannot be assumed, nor may contact
thresholds be invented. The static-world premise alone does not authorize
discarding new O occupancy, carving out a planned cargo footprint, or ignoring
an occupied voxel. Until the monitoring and current-cargo association policy
are admitted, the current-map/native admission rules above remain in force.

The strict operation motion transport submits one original timed leg through
MoveIt ExecuteTrajectory. It retains every joint coordinate (including full
turns), timestamp, derivative and effort value. A native receipt must bind the
original identity/certificate/index and actual carry geometry; source liveness
and fresh measured raw joint coordinates are checked again immediately before
submission. SUCCEEDED alone is insufficient: MoveIt SUCCESS and a fresh measured
endpoint within the owner's explicit joint tolerance are also required.

The production owner must share one motion lock with every other motion entry
point. Pending acceptance, timeout, cancellation acknowledgement and transport
exceptions do not prove a stopped controller. The transport retains that lock
until a real rejection or terminal result is consumed, cancels late accepted
goals after a stop request, and refuses overlapping submissions. Uncertain
transport has no force-clear shortcut. This port has no vacuum, planner, scene
or map writes; fixture coverage does not certify production owner adoption.

`MotionPlannerNode` now owns one `MotionOwnership` instance for its actual
PlanMotion, PlanMotionV2 and GoToRobotPose callbacks. Busy callbacks abort before
entering their planner/controller work. Legacy MoveGroup, ExecuteTrajectory and
FJT clients explicitly track dispatch, acceptance and terminal result through
this owner. A handler returning or throwing cannot release an unresolved goal;
late accepted goals are cancelled, with ownership held until terminal status.
Unknown or nonterminal results cannot authorize a legacy fallback. Public result
futures are completed only after the owner consumes the same result, so executor
callback ordering cannot overlap consecutive submissions.
The node's action-cancel callback stops only the session bound to that exact
ServerGoalHandle. It cancels accepted inner goals, remembers the stop for late
acceptance and prevents fallback submissions; a pending cancel cannot release
the session. A terminal inner result still waits for the owning callback to
finish. An already cancelled outer goal never enters its planning/controller
body. Cancelling a stale or different goal cannot stop the current owner.

A complete-operation adapter must acquire this same owner's session across all
legs and vacuum/event barriers, and give its separate `motion_lock` to the strict
transport per leg. It must finish the session only after the strict transport
consumes terminal status and releases that lock. The session excludes legacy
requests even between legs. This process-local owner does not arbitrate unrelated
nodes or direct controller clients; the production launch/driver must route every
motion entry through the owning node. Complete-operation and driver adoption
remain required; ownership wiring alone is not a completed execution chain.

Before suction, input contact xyz is a proposal on the observed box face. The
planner may select another fully supported contact and directed attitude to
serve the target placement, but must return the final pick pose, J6 and bound
complete trajectories before motion. After suction the rigid box/tool relation
is fixed. Rotating the arm moves that fixed box and tool together; it never
selects another contact or edits the issued joint trajectories.

For the user-confirmed stationary pre-seal box, the complete-operation
`StationaryOperationAttachmentPort` qualifies the actual VacuumCommandV2
request/response against the original issued plan and perception-owner token.
It requires current nonrenewing retention plus an actual full-footprint and
surface-quality result under the returned actual G. The existing measured
attachment provider has a separate `record_stationary_attachment` entry for
this qualified source; ordinary `record_attachment` still requires zero skew.
The record acquisition stamp remains the original box stamp. Its independent
StationaryAttachmentEvidence binds the later actual TCP confirmation stamp,
both measured poses, every typed request/response field, original frozen binding, actual
footprint result, conservative geometry, box identity and generation. Neither
storage retention nor confirmation time renews the existing acquisition age.
A valid stationary seal cannot be overwritten through either provider entry;
a replay after invalidation cannot revive it. This read-only qualification
layer does not command vacuum, certify a changed grasp's remaining path or
complete production execution; those event and native admission gates remain.
Raw CDR is retained for replay; stable receipt hashes use canonical typed ROS
fields because DDS alignment padding can change for identical messages.

The vacuum boundary must read the measured TCP again after the actuator
confirms sealing; a pose read before a blocking seal wait is not confirmation
evidence. Preserve the original stationary box timestamp and the new TCP
timestamp. If sealing succeeded but this later measurement cannot qualify,
report physical success with `has_measured_attachment=false` and the explicit
`SEALED_ATTACHMENT_MEASUREMENT_UNQUALIFIED` diagnostic. Keep vacuum enabled;
the operation owner must refuse carrying and retain the unresolved payload.
Do not detach or invent a measured relation to turn this into a clean failure.

Actual full-footprint contact revalidation may use the same box's retained,
frozen pre-grasp RGB-D top observation and the measured attachment relation.
Its acquisition frame/pose remain fixed; a new wrist-camera frame must not
replace this evidence. Invalidated box identity, model or observation refuses
continuation. Re-evaluate the actual footprint against those measured samples;
the original planned-grasp quality verdict is not sufficient after an offset.

Failure handling must distinguish pre-pick, carrying, release, verification,
abort, and recovery boundaries.

Before payload attachment, failures return to a non-moving wait or explicit
retry/recovery boundary. While carrying, failures transition to a carry fault
and preserve vacuum until an explicit recovery confirms payload release.
Placement commits are idempotent and must not double-count attempts.

## Exploration Boundary

Exploration policy contracts are immutable, serializable, ROS-free plain data.
Policies may propose or score candidate views, but they must not execute
motion, mutate the map, read TF, or receive ground truth.

Exploration integration must correlate the settled view request ID, source
acquisition stamp, expected map revision, and container geometry hash. Stale,
ambiguous, repeated, or mismatched view integration fails closed.

Simulation evaluation may read Gazebo truth only offline or in test/eval scope.
The online production path must not branch on `sim_mode`, consume GT, or use a
fake production success path. Drivers that call production services
(`DetectLuggage`, `ComputePlacement`, cargo-map commit, MoveIt) are in that
online path even when they live under `luggage_gazebo`. Catalog size,
`GetCurrentBox`, and unverified planned poses are GT for this rule. The
defect catalog is [real_scenario.md](real_scenario.md).

### Explicit joint-operation simulation release

The joint-operation Gazebo composition sets `physical_release_on_detach=true`.
Its simulation vacuum backend stops follow and drains the active follow call,
then invokes `/pickup_box_spawner/release_sim_box` with physical model name and
expected generation. The spawner uses its original spawn SDF and the observed
current physical pose to recreate that same body as dynamic, restoring its full
collision mask. Geometry and visual mesh remain unchanged. It observes deletion
before one creation attempt; unresolved/duplicate requests do not issue another
destructive replacement. The separate simulation release report contains the
exact SDF and handover trace for evaluation.

This service accepts no requested placement pose, measured planning dimensions
or map writes. Its acknowledgement proves only dynamic body presence. Actual
settling and post-place sensor observations remain required; the online planner
and O receive no simulator geometry or pose. Legacy simulation routes keep their
existing release owner (`physical_release_on_detach=false`) to avoid duplicate
physical replacement. Hardware does not call this simulation actuator service.

### Bounded post-release exploration

The explicit joint-operation simulation profile enables at most three extra
post-place camera motions (`operation_post_place_views`; zero disables). They
run after the original six legs and confirmed physical release, under the same
motion session and action deadline. Reuse geometry-derived camera positions,
aim at the current missing footprint centre, and rank calibrated FOV coverage
then camera travel. Projected coverage is only a ranking proxy; occlusion and
sensor success are decided by actual subsequent O observations.

Each unloaded observation path uses the existing exact snapshot IK, snapshot
MoveIt connection, original-polyline timing and full native route validation.
Its exact timed knots and native request/response are checked before the shared
controller transport; changed valid sources require a new full check, and
unresolved motion retains ownership. These paths have separate evidence and do
not alter or reuse the six-leg certificate as their authorization. No O writes
or artificial free cells are added. After each settled view, at most three O
snapshots check actual acquisitions newer than both release and settling. The
existing complete-top-footprint/one-voxel height gate remains unchanged; bounded
exhaustion is an incomplete operation, never a successful placement observation.

### Explicit joint-operation release observation and persistent diagnostic

The explicit Gazebo joint profile passes its existing `min_support_ratio` into
`release_observation_min_ratio` (currently0.6); bare consumers default1.0. The
ratio applies only to fresh directly occupied columns within one source voxel
of the planned top. Completion does not claim full rectangular reconstruction,
instance association or support suitability; missing O cells are not filled.
The next shared placement query retains all support/tilt/known-volume hard gates.
`joint_operation_sim_driver.py --three-box --failure-hold-s 25` explicitly drives
floor/floor/stack in one O epoch, initializes empty once, finalizes each actual
simulated box without deleting it, and returns with actual O before next pickup.
All slots are filtered/ranked from exact pinned observed surface+volume; simulator
spawn dimensions/poses remain actuator/eval-only. This diagnostic flag is not a
new planner or a claim that bare launch defaults changed.

The production simulation runner forwards that workload to the same installed
driver. Three-box diagnostics retain a separate acquisition-keyed pickup input
window (RGB-D/intrinsics, semantic/instance masks, cargo cloud and available TF/
joints) before each prepare request. A missing exact-acquisition input is reported
explicitly; a later image cannot substitute for it. This capture is passive and
cannot change perception or motion admission. It covers the retained recent
window, not unrecorded older geometry extrema. An optional separate Gazebo
reference process records actual poses and contact examples by execution phase;
reference data never enter the online sensor/map/attachment interfaces.

### Joint simulation sensor density and render-resource lifecycle

The named joint profile samples actual paired depth pixels with stride2; bare
launch retains4. `operation_depth_pixel_stride` is recorded under observer_depth
in launch.json. This changes direct measured ray density, never O resolution or
unknown-state policy. Explicit joint simulation gives the spawner a run-owned
render_asset_root. Every generated/replaced visual uses a unique file URI with
content-identical copies of the original model assets and a complete file hash
manifest. Collision geometry and original texture content stay unchanged. Asset
preparation precedes deletion of a previous pickup instance. This avoids OGRE
stale mesh/material reuse after deletion; it supplies no perception geometry.
Joint controller startup now advances standard spawners only on successful exit;
one fresh standard spawner may query/recover an already-loaded controller after
a lost service reply, then a repeated failure requests named shutdown. Bare
legacy startup remains unchanged. The recovery branch has behavioral event
coverage; healthy live rehearsal alone is not failure-rate qualification.

### Configured simulation fixture geometry

`sim_world.launch.py` must bind both the pickup platform and robot pedestal
visual/collision box dimensions and bottom-centred half-height poses to the
selected `scene_tf` configuration. A stock SDF's dimensions are not an independent
source of truth. The exact same configured pedestal dimensions drive MoveIt's
static world and robot base height. Runtime supplies a private configured SDF to
Gazebo create; the template on disk remains unchanged. Scene agreement must be
checked before interpreting observed occupancy or motion refusals as robot or
ULD reachability problems. No sensor points or observed cargo geometry are
replaced by fixture configuration.

### Joint-profile simulated fixed attachment

The explicit complete-operation profile uses Fortress's native DetachableJoint
for its simulated vacuum attachment. After the existing identity-preserving carry
replacement, cargo is dynamic with gravity disabled, keeps all collision/visual
geometry and existing p04-only carry mask. The spawner loads the built-in system
onto the actual S20 model entity, naming the current cargo identity, and requires
its emitted `attached` state before reporting actuator success. The vacuum backend
then stops issuing SetPose follow commands. Physical detach must emit `detached`
before the existing dynamic release handover. The initial physical relation is
preserved; no cargo reference geometry enters perception or planning. Legacy
profiles keep SetPose following. One built-in plugin per cargo remains for the
world lifetime; this campaign has three, with no unlimited-run cleanup claim.
