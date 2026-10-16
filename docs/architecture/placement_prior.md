# Placement prior ownership and adoption

A prior is a read-only speculative hint, never an execution certificate. The
nominal orchestration cycle and operator/motion/vacuum authority are unchanged.
The prior worker owns only private copies and bounded temporary computation.
It must not call ApplyPlanningScene, modify OCC, commit cargo or issue motion.

## Snapshot boundary

OCC's v2 surface is the authoritative typed grid. Its existing v1 publication
carries acquisition and confirmation metadata absent from v2; join both by map
revision, geometry identity and matching grid data. Require measured source,
positive acquisition stamp, confirmed occupancy and no pending changes. For
committed cargo, also require the owner's require_confirmed_map gate to be on;
its default-off "confirmed" flag is insufficient evidence. Read that setting
through a read-only parameter query. Never fabricate a measured next box from
catalog dimensions. Old revisions must not replace newer accepted input.

GetPlanningScene provides a baseline world for a private copy. The prior reads
`/cargo_map/get_volume_v2` for the exact admitted surface revision. The owner
serializes this read with all map service mutations; a read never integrates,
confirms or commits data. Join scene/geometry/map identity, acquisition stamp,
confirmation, grid origin/resolution/dimensions and explicit container frame.
Require measured confirmed provenance and no pending changes. Reject mismatches.

Append unknown and occupied active voxel runs using the explicit
container-to-world transform. Merge only consecutive blocked vertical cells;
never bridge an observed-free interval. Inactive hull cells do not establish
observed free space; the static container geometry remains present. Bounds:
at most 1,000,000 volume cells and 20,000 private obstacle runs, otherwise refuse.
The 2D surface remains the support/candidate interface; its full-column unknown
classification must not erase measured 3D free intervals. The conservative
column adapter remains for offline legacy callers; the online prior requires
the exact volume and cannot silently fall back if acquisition fails.
Unknown cells remain blocked for prior screening. Retain unrelated world
obstacles; only replace the prior's own reserved column IDs. Never publish the
composed scene. Missing/full-scene identity or invalid occupied heights reject
adoption rather than falling back to an empty world. Final route checking still
uses the actual current scene and authoritative attachment.

## Grasp and tool boundary

Use T_box_tcp, shared with existing pick/place contracts. Measured suction
contact orientation is an outward surface normal, not a TCP quaternion. Derive
the grasp from the attach target produced by build_candidate_pick_segments.
Choose a validated candidate ID and retain its observation/contact-model identity;
do not shift an accepted suction patch without re-evaluation. Keep the selected
directed box axes, including the pi heading branch, across pick and place.

Rigid tool screening positions the fixed component containing suction_contact_frame
in a PRIVATE RobotState and explicitly refreshes collision-body transforms.
Check all its robot collision shapes and supplied box against the frozen world.
Do not infer clearance from TCP clearance, skip the radar/camera, or exempt a
colliding tool/world pair. The arm outside this rigid component is intentionally
not evaluated by this screening query; whole-arm IK and collision certification
remain mandatory. Critical transit/entry endpoints matter as well as final place.

The CheckPlacementToolV2 service is bounded and returns contacts plus model and
snapshot identities. It has no live scene subscription or scene-update clients.
Its successful verdict means only rigid-tool/world screening, not executable
trajectory feasibility. Native pointer/geometry caches must be updated before
const collision APIs; the historical radar collision is a required regression.

## Background and consumer boundary

One active job, one replaceable pending job, one published result. New map/input
invalidates stale work. Cancellation/deadline discards partial results. An
abandoned native service future remains tracked until completion so subsequent
jobs cannot create an unbounded service backlog. Templates and measured requests
have distinct provenance and cannot be adopted interchangeably.

Optional automatic prefetch requires explicit template dimensions and full
box-to-TCP transforms. It triggers once per confirmed snapshot, after read-only
dependencies are ready. Templates remain hypothetical. Matching geometry may
reuse generated slot proposals and their ordering, but measured contact
candidates undergo collision screening again. Changed size, model, scene or
snapshot regenerates proposals. Foreground preparation supersedes background
work; an old timeout cannot cancel a newer generation. Neither timer nor
template completion issues motion or changes map ownership.

Enable with `prefetch_enabled=true`, `prefetch_box_wdh` (three metres), and
`prefetch_grasps_flat` (one or more concatenated xyz/xyzw transforms). These
parameters describe a hypothesis, not a measured payload or approved grasp.

Before pick, bind the selected slot, candidate and grasp. Picking another candidate
requires a new matching binding; a prior for candidate A cannot follow a retry
that picked B. After attach, measured attachment and actual joints remain
canonical. Reconnect/replan as required and pass the existing complete route
certifier. On prior miss, mismatch or failure use ordinary planning; do not
relax limits or replay a cached trajectory blindly.

## Hardware place planning boundary

Successful VacuumCommandV2 enable returns `has_measured_attachment`, the
controller-admitted `measured_grasp`, and `attached_box_wdh`. Encode the reply
before the attach effect and expose it only after backend success. Disable and
failure never claim an attachment. The reply retains the actual attach TCP
stamp and scene/request identity; do not replace it with the requested prior.

`hardware_pick_driver --frame-runtime --placement-prior
--plan-place-after-pick` binds that measured reply, reads six current joints and
TCP/container TF at the joint stamp, and calls complete SelectPlacePathV2 after
pickup. A matching selected prior fixes the box target; the measured attachment
determines its TCP target. Stale maps, changed pickup candidate, missing attach
proof or changed box identity refuse this binding. Without a prior, the ordinary
ComputePlacementV2 then Select chain remains available. This option plans only;
it does not execute the place route. It requires retained vacuum and rejects
`--release`. Planning failure reports carry fault and does not release payload.
Actual selector/OCC services and a healthy typed runtime must be running; the
pick-only launch does not imply that a complete place graph exists.

## Recovered trajectory timing

Cartesian timing must bound the controller's interpolated trajectory, not only
its knot derivatives. The site retimer evaluates quintic velocity/acceleration
extrema between knots and uniformly dilates time when necessary. Derivative
fields scale by 1/k and 1/k² along with times by k, preserving the interpolated
joint curve. Recheck nanosecond wire-time rounding before returning a trajectory.
Velocity/acceleration limits remain unchanged. This correction is not a collision
or complete-route certificate; the existing route checks still apply. Report
increased motion duration separately from prior planning latency.

## Full-cycle simulation binding

Depth bootstrap may declare up to three additional scene-bound world-camera
intents. Each uses the existing certified observation route and execution
admission; only the map owner integrates settled depth acquisitions. The
response wait for an already-certified observation leg includes its trajectory
duration plus the configured response overhead. It does not alter planning or
certification budgets, trajectory timing, controller limits or collision gates.
An empty or nonmonotonic trajectory cannot supply an execution wait budget.
The V2 motion service forwards the node's configured `execute_timeout` to the
executor, as the legacy service does. A library default must not silently
override that configured execution limit.

The existing motion/OCC evaluation driver accepts `--placement-prior` only with
its typed frame runtime. Before pickup it binds one measured candidate without
changing its contact pose or rank; after pickup it preserves the selected box
slot and uses the controller-admitted actual attachment. Stale target bindings
fail instead of silently changing the desired placement. A prior miss preserves
the ordinary candidate path. This wiring requires full-cycle evidence before
claiming successful placement or performance improvement.

Runtime installs must include `st09_contracts` through the planning package's
explicit dependency. Source-root PYTHONPATH is not installation verification.
The non-scored `scripts/placement_prior/rehearse_scene.py` checks actual scene
objects, static synchronization, selector/validator discovery and a single clock
publisher; it spawns no box and is not a functional pick/place acceptance test.

Prior experiments may opt into `evidence_dir` on the sidecar. It captures exact
prepare requests/responses, baseline scene, and native collision boundaries in a
new directory with a 256 MiB limit. Exhaustion refuses adoption and must be
reported as incomplete evidence. No capture directory means capture is disabled.

Simulation must explicitly select `top_surface_mode:=dynamic` to obtain measured
suction candidates. The existing legacy detector produces no such candidates.
An initial `static_empty` map is not a measured snapshot: acquire a settled cargo
view through the map owner's normal observation workflow before expecting prior
adoption. The read-only prior must never promote static geometry to measured data.

Dynamic perception must run `SuctionPatchEvaluator` on the joined depth,
instance region and top estimate before publishing grasp candidates. A fitted
centre produced by `center_suction_evaluation` bypasses footprint validation and
must not be used as offset-grasp acceptance evidence. Zero accepted patches
remain zero; do not mint a centre fallback. Contact model thresholds remain
unchanged by this integration.

`--prior-acquisition-only` in the motion/OCC driver requires a non-scored
rehearsal and prior enabled. It returns at the detected-input hook before pickup;
it is an input capture tool, not a successful cycle. `--dump-dir` also captures
exact typed service requests/responses before client-side slot-index remapping.

## Initial measured-map prerequisite

An empty cargo semantic cloud cannot establish that the container is empty.
The existing cargo-view integration consumes cargo endpoints and drops endpoints
below floor + 0.05 m. A static-empty initial grid therefore remains insufficient
for measured prior adoption; do not change its source label or confirmation flag.
A settled full-depth container observation must be handled by perception and the
map owner, with exact acquisition/TF identity and explicit observation provenance.
Keep it separate from cargo-only clouds so equal-stamp callbacks cannot overwrite
one another. Static-wall returns may establish free ray interiors but must not be
added as cargo occupancy. Unseen/occluded cells remain unknown; another view may
be required. The prior consumes the owner's resulting snapshot only.

Evaluation drivers must stop and join both the main executor spinner and the
TransformListener's dedicated executor/thread before destroying either node.
Worker shutdown is a prerequisite to rclpy context shutdown, not interpreter
finalizer work.

The mapper now provides `/cargo_map/integrate_container_depth_view` using the
existing `IntegrateCargoView` settled-acquisition request contract. It consumes
`/luggage/perception/container_depth_points` (configurable `container_depth_topic`)
in an independent buffer. The acquisition stamp ledger is shared with cargo
integration, so the same frame cannot count twice through different endpoints.
Both view handlers serialize through the same view lock and retain existing
runtime readiness, geometry hash, expected revision, settled, freshness and
exact-stamp TF checks. No topic producer is started by the prior.

Full-depth ray processing uses the authoritative hull (including its chamfer),
permits entry only through its -X aperture, and rejects returns inconsistent
with closed faces. One voxel diagonal before a return remains unobserved;
interior returns outside that closed-face band become sensor occupancy. Near-door
obstacles remain hits even when no preceding free voxel can be established.
Static boundary returns create free-ray evidence only, never cargo hits. Missing
or rejected rays create no evidence. Surface columns remain unknown until the
existing mapper has sufficient observations; no unknown-to-free shortcut is
introduced. A zero-cargo measured depth view may initialize provenance, but that
alone does not establish enough free columns for a prior placement.

`ContainerDepthObserver` is the explicit client for that owner service. It joins
preprocessed depth images and depth CameraInfo by exact positive stamp, equal
optical frame and image dimensions; unsupported distortion is rejected. It uses
the existing six-joint MotionStabilityGate at the samples' sensor times, so stale
joint arrivals cannot refresh stability. It preserves the image stamp on optical
points and checks the owner's schema, integrated stamp and advancing map revision
before accepting a result. Call it from an orchestration worker while the ROS
executor continues processing its subscriptions and service replies.

The simulation driver opt-in `--bootstrap-depth-map` uses the existing certified
GoToRobotPoseV2 path to `portal_observe_sim`, integrates three settled depth frames,
and returns through the certified pickup-observe path before starting a box.
Failure to reach the observation pose prevents capture. It does not promise full
coverage: measured unknown/occupancy ratios are preserved. Input images, camera
calibration, joint stability samples, clouds and owner requests/replies are
captured; raw named-motion service calls are also captured when dumps are on.

For observation goals, a solved bounded revolute-joint angle may be replaced by
its nearest limit-valid full-turn equivalent relative to the actual start.
The producer records both representations; it never wraps prismatic joints or
changes the requested camera pose. MoveIt still builds the connection and the
full route still requires certification. A shorter angular displacement alone
is not evidence of collision-free motion or a shorter certified trajectory.
