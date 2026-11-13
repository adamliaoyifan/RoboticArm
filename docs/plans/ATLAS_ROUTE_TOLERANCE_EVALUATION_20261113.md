# Impact-based geometry tolerance and Atlas/route evaluation

- date: 2026-11-13
- author: codex/gpt-6
- status: design for review; not an executed evaluation
- dispatch_ready: no
- reason: case extractor/harness, installed freeze and concrete evaluator/model must be registered before scoring

## User direction and scope

Keep MoveIt and the current whole-operation production route. Establish complete
functional operation first, collect replayable cases, then have an independent
Claude/Cursor evaluator measure Atlas and trajectory-reuse impact. Do not modify
generic perception to match simulator geometry. Simulation truth is offline
reference only. Hardware significance requires measurements; current bags lack
independent complete-body references. No universal 2 mm ignore rule, new padding,
new mandatory observation motion, or production latency gate is authorized here.

## Geometry impact mechanism to evaluate

Maintain separate meanings for nominal measured box pose/size, actually observed
suction support, and collision geometry uncertainty. Revalidate original physical
goals/paths with the current admitted geometry before re-enumerating/replanning.
Compare geometry-induced swept-volume change with path and endpoint clearance.
Translation, rotation, attachment relation and shape extent all matter; size-only
thresholds cannot represent them. A corner at radius rho can move by up to
2*rho*sin(abs(delta_angle)/2), separately from translational/extent change.
This is an analysis bound, not an online certified uncertainty distribution.

If the original remains admissible, retain it. If affected, try another grasp
offset before suction, box orientation, IK branch, slot or route. Request a
targeted pre-pick view only when missing geometry materially affects these
choices. After suction the attachment relation stays fixed. Missing current
references do not justify fitting a percentage to simulator truth. Tight final
side contact, support and release error need their own measured assessment;
moving-path clearance does not by itself establish final-layout robustness.

## Staged evaluator handoff

Preparation is repeatable and unscored. Build the replay harness and validate
source/install/launch wiring without spending scored case seeds. Resolve concrete
owner/model and freeze a higher plan revision with exact commands and complete
case manifest before adding runnable work to OPEN.md. Existing test names alone
are not experiment acceptance criteria.

### A. Geometry sensitivity, no generic perception change

- Inputs: frozen source/native O maps, detection, grasp relation and trajectories
  from `geometry_plan_reuse/20261113/simulation_01`, plus exact diagnostic_53
  replay inputs referenced by that report. Preserve original acquisition IDs.
- Workload: original input and labelled synthetic perturbations of size, centre
  and yaw independently. Extent/translation levels 0, +/-2, +/-5, +/-10 mm;
  yaw levels 0, +/-0.5, +/-1 degree. Axis-isolated changes first. These are
  stress inputs, not asserted real error rates or newly accepted tolerances.
- Outputs per case: selected goal unchanged/changed, original path accepted or
  refused, first affected predicate/segment/contact pair, available clearance
  where actually measurable, capture/recheck/replan times. Missing distance data
  is not zero clearance or a pass.
- Deterministic acceptance: known geometry-unaffected controls retain their goal;
  constructed penetration/unknown/support-loss controls refuse. All enumerated
  controls must be scored, with exact per-case results; this is conformance, not
  a claimed real-world success probability. A non-occurring branch is
  `not_evaluated` until its constructed control supplies at least one sample.
- Real follow-up: collect measured exterior dimensions/reference and repeated
  poses across actual viewpoints before reporting absolute body error. Existing
  grab bags support repeatability analysis only. Do not turn sim error into a
  hardware deployment cutoff.

### B. Atlas contribution, independent evaluator

- Freeze one identical robot/ULD/occupancy/contact workload per paired trial.
- Variants: (1) online IK only; (2) qualified Atlas seeds with identical complete
  certification; (3) current seed/branch/opening ordering. A richer Atlas variant
  is added only after its sampled fields and exact ranking code exist. Do not
  call current status-only vs unspecified future implementation an A/B result.
- Case strata: floor/stack x shallow/deep x four directed quarter-turn headings.
  Extract one feasible candidate per stratum (16 cases), or record unavailable
  strata explicitly before scoring rather than replacing failures after scoring.
  Depth split comes from the manifest's source hull midpoint, not a tuned cutoff.
- Three fixed seeds 0/1/2 per available case, same budgets and start joints:
  48 paired trials per variant if all strata exist. Record IK attempts/branches,
  native connector calls, complete certification outcomes, preparation time,
  trajectory duration and joint travel. Run variants in alternating order.
- Report paired successes/refusals/timeouts plus median/p95/max timing and paired
  deltas. Report missing strata and sample counts. No N-consecutive-pass
  reliability gate or production latency threshold is set by this small study;
  it cannot establish fleet reliability. Native collision/unknown controls must
  all refuse under each variant, with at least one sample per refusal mechanism.

### C. Successful trajectory reuse and fallback

- On every available B case, compare cache disabled, cold empty cache, exact-end
  warm cache from a genuinely successful execution, and warm cache with a new
  obstacle intersecting the cached swept payload/robot path. Preserve current
  complete certification in every mode. Do not promote a planned-only trajectory
  as a successful execution. Cases without such an execution record are
  `not_evaluated` for warm-cache benefit.
- Use the same three seeds, initial joints, map and plan budgets as B. Record
  lookup/hit/certification/fallback times and overall preparation. Distinguish
  cache miss from cached-path refusal; report zero hits as no benefit measured.
- Acceptance: at least one real eligible hit fully recertifies, at least one
  intersected cached path refuses and enters the normal planner, and corrupted
  or identity-incompatible cache controls miss. All control verdicts must match.
  These are mechanism samples, not a success-rate estimate. No speedup assertion
  without paired elapsed times including certification/fallback cost.

## Capture and freeze requirements before dispatch

Freeze source commit, dirty input hashes/archive, effective install/model/scene/
Atlas/config hashes, original sensor/map acquisitions, exact command, machine,
case list, budgets and seeds. No modification of the scored artifact mid-run.
Use only one Gazebo stack if execution is required; most cases should replay
native planning without physics. Startup/wiring repairs are unscored rehearsals.

T1 records each capture, candidate, IK/connector, certification and reuse/fallback
boundary with IDs, outcome and time. T2 saves exact failing inputs and first
divergent output. For raw sensor-root-cause work, add bounded synchronized RGB-D,
camera info, TF, segmentation, filtered cloud, pre/post-history and attachment
inputs; the existing simulation bundle cannot substitute for these missing data.

The dispatch revision must set the raw capture window from measured stage
latency, storage cap/overhead, replay command and capture-health check. Until
those values and the harness are validated, this document is a design and must
not be represented as a runnable acceptance task. Persist failed bundles under
`docs/status/evidence/` before teardown or retry.

## G/O alignment order proposed by source audit

1. Introduce one backend-neutral snapshot/source interface with lossless backend
   identity; O uint64 and G UUID epochs remain distinct. Version the wire rather
   than forcing UUID into current O uint64 fields.
2. Use it at placement, preparation, latest-content checks, execution and
   post-release observation/adoption. Preserve native resolution and exact
   content binding; remove G's accidental surface-only production fallback.
3. Align actual ingress semantics: measured rays/origin/time/TF, static structure,
   initialization, settled/unknown policies. Review G's existing floor+5cm input
   cutoff independently; it is not spacing or collision padding.
4. Verify current main-tree dense G reads and retained-token use under concurrent
   updates. Add true source-side regional export only if measurements justify
   it; slicing full transfers is not a transport optimization.
5. Run the identical full-model floor/floor/stack functional workload under each
   owner, recording map-induced differences instead of assuming equal log odds.
   Require observed post-release cargo and current native full-chain checks.

## Pointers

- docs/agents/reviews/2026-11-13_1222_tolerant-geometry-go-review.md
- docs/status/evidence/geometry_plan_reuse/20261113/REPORT.md
- docs/plans/OCC_DUAL_BACKEND_CONSUMERS_G3_20261104.md
- docs/architecture/cargo_grid.md
