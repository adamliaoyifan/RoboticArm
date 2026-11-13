# Upper-layer placement research implementation

This independent action takes a box already held by the suction panel, plans
upward insertion under the ULD ceiling, transfers the box onto declared lower
cargo, withdraws the panel, and pushes with its actual rigid plane. It does
not change ordinary pick/offset-pick selection. It is an ideal-input simulation
prototype; D555 input, irregular supports and hardware qualification remain open.

The single normative plan is [ULD upper-layer V1](../../docs/plans/ULD_UPPER_LAYER_V1.md).
Evidence and cross-agent coordination live in the primary workspace, never this
worktree. A nominal plan is not an execution success or a reliability certificate.

## Request and result

`placement_request.PlacementRequest` is an immutable value object. All poses are
row-major 4x4 transforms in `container_link`, with **absolute container Z**. The
box reference is its geometric centre, never the suction TCP. Sizes are metres;
joints are radians. Base/world/container transforms come from the active scene.
The current action inserts container +X, corresponding to base -Y in this cell;
the box's long edge must align with insertion.

| Input | Meaning and consumer |
|---|---|
| `box` | Dimensions, current geometric-centre pose, COM assumption, optional measured mass. Geometry and payload effort screen consume it. |
| `snapshot` | Occupied geometry, horizontal support rectangles/absolute heights, floor/ceiling and identity. All obstacles remain in the collision scene. A support point alone cannot describe support area. |
| `start_q` | Actual six joints at the planning boundary; the executor rechecks them. |
| `final_box_poses` | One or more permitted final box-centre transforms. The planner does not silently shift an exact goal. The caller supplies allowed small adjustments. |
| `support_source_ids` | Allowed support rectangles in this snapshot. This selects support without removing other collision objects. |
| `start_attached`, `confirmed_box_T_tool` | Existing measured grasp, preserved exactly throughout carry; no hidden regrasp. |

`request_planner.UpperPlacementPlanner.plan(request, evidence_directory, seed)`
returns `valid`, a reason, complete-plan count, `plans`, `selection` and input
revision. `valid` means at least the configured minimum of complete **nominal**
plans passed certificates and scoring. Each plan contains the final box pose,
handoff, suction/push transforms, stage paths, timed controller trajectories,
clearance witnesses and bound input/model/configuration identities. Physical
execution remains `not_evaluated` until a separate actual-controller run passes.

Each request owns private collision scenes and bounded planning workers. Input
values are copied/frozen; scenes/workers are not public mutable shared context.
Persisted results remain auditable, but a changed snapshot, attachment, start,
configuration, model or certified source invalidates their use for execution.

## Planning and execution

1. Generate alternative handoffs, grasps (unless already fixed) and CAD-derived
   panel push contacts for every permitted target. Check whole-panel fit in the
   unsupported area, geometric support coverage and COM margin.
2. Jointly choose connected IK branches and collision-free transitions. The
   already-attached chain has nine stages: inversion/approach, upward insertion,
   lower to support, release clearance, withdrawal, push reposition, contact
   acquisition, push and retreat. Cartesian contact stages and free joint-space
   routes use their appropriate planners; isolated IK endpoints are insufficient.
3. Certify interpolated paths and timed trajectories against source geometry,
   limits and configured clearance. Repair free-box repositioning against the
   box uncertainty envelope. Optional mass-based gravity screening is necessary
   actuator screening, not measured torque or full contact dynamics.
4. Reject incomplete/unsafe plans before scoring. Rank robot clearance first,
   then support within the configured collision tie band. Keep distinct complete
   schemes; target 3, minimum 1 by default. Bounded search may return fewer than 3.
5. Execute real joint-controller commands. Establish actual native lower contact
   and support dwell before native suction release. Confirm constraint removal,
   panel separation and 3s box stability before repositioning. Establish actual
   panel-plane contact before pushing. Check support, lower movement, progress,
   aligned controller feedback, final pose and final stable dwell.

The nominal box-push model is fixed-orientation straight translation along the
insertion direction. Actual motion comes from native physics after release;
there are no detached cargo pose commands. Native lower/panel contact is checked
at each 1ms physics step; the full observed scene is checked every 10ms in the
engineering profile. These checks do not establish a continuous hardware error
tube or force safety.

`upper_config.yaml`, `selection_policy.yaml` and `execution_policy.yaml` define
runtime settings. Each planning call reloads YAML. In particular
`support_com_margin_m: 0.020` changes actual candidate admission without compiling;
`support_area_ratio_min: 0.60` is geometric near-contact coverage, not pressure
area. Current geometric COM is an explicit assumption. The panel push frame is
derived from CAD rather than an invented tool centre.

The separately enabled E-EDGE1 mode permits bounded source-planned leading-bottom
edge / support-entry-top edge traversal only during detached push. It requires
independent strict native top/bottom support, 100um material-edge proximity,
50um penetration, both normal cones within 1deg, a 10mm source-event progress
band and a 1s nonresetting episode limit. All settings are external execution
policy fields. The default stays strict. No side-wall or arm collision exemption
is introduced; this experimental mode is not a force/robustness certificate.

## Running

Use ROS Humble plus NumPy, SciPy, Shapely, trimesh, python-fcl and OMPL. Planning
can run offline; the live simulation adapter pauses its owned world while offline
planning, rechecks actual held state, then resumes before sending trajectories.
This is simulation clock management, not a hardware holding-time claim.

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s research/uld_upper_layer/tests
python3 research/uld_upper_layer/request_planner.py \
  --request "$REQUEST_JSON" --scene-config "$SCENE_YAML" \
  --config "$UPPER_YAML" --execution-policy "$EXECUTION_YAML" \
  --evidence "$PRIMARY_EVIDENCE_NEW_DIRECTORY"
```

`physical_probe.py --live-request` uses the same request schema and controller
execution. Its simulation pick initializer starts at a fixture seal pose and
performs a real lift; it does not claim home-to-pick planning. Test scenes, native
plugin builds, the actual experimental profile, source hashes, failed windows
and execution reports are linked from the sole plan and primary role note.

Legacy `run.py` / `action.py` are the original geometry-only workspace study;
the request-driven public chain is `placement_request.py` -> `request_planner.py`
-> `plan_artifact.py` -> `physical_executor.py`. They must not be confused.

## Read-only placement prior

`upper_prior.UpperPlacementPrior.query(request, goals, budget) -> PriorHints` is
a deterministic hint source consumed by `UpperPlacementPlanner` when
`prior_enabled` is true in the runtime configuration (default false). It holds
no agent, no LLM and no second OCC; it cannot write the occupancy snapshot,
command motion or change vacuum, and every hint it returns is a seed or an
ordering suggestion that the normal planner re-checks end to end.

Inputs are the ordinary request objects plus the allowed final box-centre SE3
goals. The prior recomputes, from the live source model in every process, the
link6-fixed assembly projection and the necessary ceiling-fit window per goal:
a candidate is pre-dropped only when its whole assembly and box lie inside the
ceiling footprint with a confirmed attachment and the recomputed required gap
exceeds the available gap. Entrance or tilted candidates whose tool lies partly
outside the footprint are never killed by that global threshold; they keep the
exact mesh checks of the normal chain. Detached requests only receive
informational windows under archived representative attachments.

Archived artefacts are consulted read-only when `prior_index_path` names the
primary-workspace posture-prior catalogue: route templates (stage waypoints,
handoff/push families) whose snapshot/attachment identity mismatches are marked
stale, keep `seed_only` authority and never become executable certificates;
their archived handoff support and motion-only duration are re-evaluated against
the current runtime configuration (COM margin, execution budget). IK branch
seeds come from the four-direction atlas by position/rotation neighborhood
after the archived joint-XML signature matches the live URDF; seeds are initial
values only and are refined onto exact targets by consumers.

Lifecycle: the assembly projection, archive index and atlas trees are private,
read-only and cached for the process; `PriorHints` is immutable and valid for
one request only. Any snapshot, model, start, attachment or configuration
change requires a new query. Missing, unreadable or signature-incompatible
indexes, empty neighborhoods and exhausted per-goal budget slices (fair shares,
5 s total by default) degrade to plain search without crashing. Effective
switch, radius, branch, budget and index values are recorded in
`prior_query.json` per request. The prior never resamples a confirmed grasp and
never widens the caller's goal set; ordering and necessary rejection are its
only planning effects.

The checked single-case settings are `profiles/nominal_physics_planning.yaml`
and `profiles/nominal_physics_execution.yaml`. They explicitly request one
complete plan, enable source-mass screening and E-EDGE1, use slow controller
motion, and reserve 750MB for actual traces. This is an experimental profile,
not a replacement for production configuration. Default configuration still
requests up to three complete schemes and keeps strict support contact.

Case10 at source `14e6f73a` completed all nine controller stages in native
physics and the final stable dwell. Independent final error was about 1.22 mm;
maximum observed lower-body translation 1.39 mm (policy 3 mm). Thus "lower stationary"
means no commanded lower movement and drift within the declared tolerance,
not mathematically zero physical movement. One ideal-input case establishes no
success rate. Formal release calibration, contact error tubes and reliability
campaign remain open; hardware execution is not qualified by this result.
