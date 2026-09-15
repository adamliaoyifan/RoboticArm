# Dynamic top-surface estimation and sealable suction-patch selection

Date: 2026-09-15

Status: approved by user for dispatch to `claude/glm-5.3`.

Parent: `DYNAMIC-SUCTION-20260915`

Base revision: `8f4d0a0dc8c449197caf146bd3a6ec6cf2fba627`

Plan revision: the exact commit containing this approved plan is recorded in
each generation-1 dispatch thread and mailbox row.

## Objective

Deliver two software-only changes to the real-cell pick path:

1. Estimate the box top from the current YOLO instance and aligned depth at
   any visible/reachable pickup position. `scene_tf` remains authoritative for
   TF, collision and safety geometry, but must not select, crop, accept or
   center the detected box surface in the hardware path.
2. Replace the single box-centre suction target with ranked, timestamped local
   suction candidates. A candidate is valid only when the configured rigid
   suction-panel footprint lies on one connected, sufficiently flat surface
   without a disqualifying height discontinuity. Vacuum failure may try a
   bounded next candidate through an explicit pre-attachment recovery path.

The outcome is fail-closed: uncertain segmentation, stale geometry, no
sealable patch, unsafe recovery or missing panel configuration never produces
an attach motion.

## Non-goals

- No suction-panel, cup, valve, pneumatic or force-sensor hardware changes.
- No use of Gazebo/fixture ground truth by an online node.
- No fixed pickup-source pose, platform height or per-trial spawn pose as a
  detector input.
- No claim that a visually flat patch guarantees a seal. DI0 remains the
  authoritative attachment result.
- No change to container placement geometry or packing policy.
- No automatic movement after a failed seal unless DI0 is confirmed low and
  the recovery retreat is planned and executed successfully.

## Existing baseline and defects

- `origin/master` already defaults `crop_to_workspace=false` and disables the
  hardware semantic workspace predicate. This invariant must be retained.
- Hardware `yolo_world` currently aliases to `BboxFillSegmenter`; it supplies
  a rectangular image ROI rather than a pixel-accurate instance mask.
- Top extraction selects a high horizontal RANSAC plane, uses PCA as a yaw
  seed, then searches a minimum-area rectangle only within +/-20 degrees of
  that seed.
- `TopSurfaceEstimate` has no local contact footprint, height-discontinuity or
  sealability result. Its confidence is dominated by inlier count.
- Pick waypoints use the detected box pose XY and one top Z; the driver tries
  one target and exits immediately on `VACUUM_SEAL_TIMEOUT`.
- The URDF exposes an approximate `suction_contact_frame` and a coarse panel
  envelope of about 0.18 x 0.18 m in the contact plane, but no authoritative
  per-cup sealing-ring geometry. This plan therefore treats one rigid
  rectangular footprint as the software safety model.

## Fixed design decisions

### A. Dynamic instance-derived top surface

The hardware pipeline shall be:

`RGB + aligned depth -> YOLO instance ROI -> depth component -> world points ->
gravity-constrained plane candidates -> connected top patch -> rectangle`.

1. Prefer a pixel instance mask when the configured backend provides one.
2. For `bbox_fill`, derive a depth component inside the box:
   - erode the box by 5 pixels before choosing seeds;
   - take the median valid depth in the central 30% of the eroded box;
   - retain the largest 8-connected component whose depth differs from the
     growing component median by at most 30 mm and whose adjacent depth jump is
     at most 20 mm;
   - reject when retained coverage is below 35% of the non-eroded bbox or fewer
     than 200 valid pixels remain at 640 x 480; scale pixel counts by image
     area for other resolutions.
3. Erode a true instance mask by 3 pixels for plane fitting. Preserve the
   original mask for boundary and coverage scoring.
4. Transform only at the acquisition stamp. Missing or stale TF fails closed;
   latest-TF fallback is forbidden.
5. Generate all gravity-consistent planes with at least 15% of retained points
   and at least 80 inliers. Plane normal must be within 8 degrees of world +Z;
   the existing 8 mm RANSAC distance is a geometry grouping tolerance only,
   never a seal threshold.
6. Score plane candidates by instance-mask support, connected image area,
   inlier fraction, residual and height. Height alone may not select a plane.
   A plane with less than 60% of the best candidate's connected support area
   cannot win merely by being higher.
7. Project the winning connected inliers into plane coordinates. Use PCA only
   as a diagnostic/seed; compute the final rectangle from the convex hull over
   the full 0-90 degree rectangular symmetry range. A PCA error greater than
   20 degrees must remain recoverable.
8. For aspect ratio below 1.15, retain `yaw_valid=false`; do not stabilize an
   unobservable yaw with `scene_tf` or a catalog prior.
9. `scene_tf` may be consulted after detection for reachability, collision and
   safety-envelope checks. Changing pickup-source XY while replaying identical
   sensor data must not change the top estimate or suction candidates.

### B. Software suction-patch model

Add a versioned hardware configuration, proposed path
`src/luggage_description/config/suction_contact_model.yaml`, containing:

- `model_version: 1`;
- `contact_frame: suction_contact_frame`;
- `footprint_type: rectangle`;
- `footprint_size_xy_m: [0.18, 0.18]` as the conservative current CAD-envelope
  default;
- `boundary_margin_m: 0.015`;
- all flatness and sampling thresholds below.

The hardware launch must require an explicit readable config and log its
absolute path plus SHA-256. Missing, malformed, non-positive or oversized
geometry fails startup. Simulation and synthetic tests may inject smaller
footprints. If a later measurement replaces 0.18 x 0.18 m, it requires a
versioned config change and rerunning this plan's gates, not a hidden constant.

Build a 5 mm-resolution height map in each top-plane coordinate system. Each
cell stores valid count, median height, robust spread, mask membership and
local normal. Candidate centers are sampled on a 10 mm grid; include the
geometric centre but never privilege it.

A candidate footprint is accepted only when every condition holds:

- at least 90% of footprint cells have valid depth;
- at least 95% of the footprint lies inside the original instance mask after
  applying the 15 mm physical boundary margin;
- one connected plane label covers at least 90% of valid footprint cells;
- plane-fit RMS residual is <= 2.5 mm;
- absolute residual P95 is <= 4.0 mm;
- robust peak-to-valley `P99(z)-P1(z)` is <= 6.0 mm;
- local-normal deviation P95 from the candidate normal is <= 5 degrees;
- no adjacent valid cells within 10 mm have a height jump > 4.0 mm;
- no two height modes separated by >= 5.0 mm each contain >= 15% of valid
  footprint cells;
- candidate normal is within 8 degrees of world +Z;
- all source data share the instance ID, generation, acquisition stamp and
  frame.

These are conservative initial software thresholds. They are intentionally
tighter than the visual RANSAC tolerance. Changing a threshold requires a new
plan generation with the same synthetic and real-cell matrices.

Accepted candidates are ordered by the following deterministic tuple:

1. lowest discontinuity flag/count;
2. highest valid coverage;
3. lowest P95 residual;
4. largest boundary clearance;
5. smallest XY distance to the estimated box centre;
6. lexicographic XY as the final reproducible tie-break.

Retain at most five accepted candidates separated by at least 50 mm centre
distance or footprint IoU <= 0.25. Publish none if every candidate fails.

### C. Interface contract

Add a ROS-free `SuctionPatchEvaluator` algorithm and plain result dataclasses.
It accepts arrays, the acquisition identity, top-plane candidates and a plain
contact-model object. It imports no ROS or TF packages and exposes state only
through `update(..., stamp, frame_id, instance_id, generation)` plus
`copy_output()`.

Add `luggage_msgs/msg/SuctionCandidate.msg` with:

- measurement `Header`;
- `instance_id` and `generation`;
- world-frame `contact_pose` whose +Z is the outward local surface normal;
- `candidate_id` and deterministic rank;
- `score`, valid coverage, mask coverage, plane coverage;
- RMS, P95 and robust peak-to-valley residuals;
- normal-deviation P95, maximum adjacent step and boundary clearance;
- footprint model version/hash.

Extend `DetectedLuggage` with accepted `SuctionCandidate[]`. Rejected
candidates and their machine-readable reason codes remain bounded diagnostics
and evidence, not an unbounded production message. `DetectLuggage` returns
`success=false`, message `DETECT_NO_SEALABLE_PATCH`, when box geometry is valid
but no patch passes. `top_surface_valid` alone no longer authorizes hardware
attach.

The selected candidate stamp/frame/instance/generation must match the
`DetectedLuggage` observation. Any mismatch returns
`SUCTION_CANDIDATE_IDENTITY_MISMATCH` before planning.

### D. Planning and retry contract

Planning evaluates accepted candidates in perception rank order, skipping any
candidate that fails TF, IK, collision or Cartesian-fraction checks. It may
attempt at most three candidates per operator request and at most one vacuum
attempt per candidate.

For a selected candidate:

- candidate XY/Z and local normal, not box-centre XY/global top Z, define the
  attach pose;
- tool orientation aligns suction-contact +Z opposite the candidate outward
  normal while preserving a collision-free yaw;
- `pre_grasp`, `approach` and `attach` are rebuilt for that candidate;
- approach and recovery motion are along the candidate normal;
- the requested candidate ID is logged in every segment and vacuum event.

On `VACUUM_SEAL_TIMEOUT` before attachment:

1. the backend completes its existing release sequence;
2. DI0 must be observed low continuously for 0.5 s within a 2.0 s deadline;
3. execute a Cartesian reverse of at least 80 mm along the attempted candidate
   normal to the recorded approach pose;
4. require successful controller settling and an unchanged box-generation ID;
5. only then may the robot move laterally and try the next candidate.

If release confirmation, recovery planning/execution, state identity or graph
health fails, stop with `SUCTION_RETRY_RECOVERY_FAILED`; do not try another
candidate. While DI0 is high or ambiguous, preserve vacuum state and enter the
existing carry/recovery fault boundary. Exhausting valid/reachable candidates
returns `SUCTION_CANDIDATES_EXHAUSTED`. Retries never trigger a new detection
while the arm occludes the original view.

## Subtasks

| ID | Owner agent/model | Depends on | Base revision | Scope | Acceptance | Required tests | Commit evidence | Dispatch |
|---|---|---|---|---|---|---|---|---|
| ST-1 | `claude/glm-5.3` | none | `8f4d0a0dc8c449197caf146bd3a6ec6cf2fba627` | Dynamic instance/depth component, multi-plane selection, full-range rectangle; remove hardware `scene_tf` detection influence | Gates A0-A5 | ROS-free unit, property, replay and launch tests | Clean passing commit plus evidence pointer | `dispatch_ready: yes` |
| ST-2 | `claude/glm-5.3` | ST-1 | same | Contact-model config, suctionability height map, discontinuity gates, ranked candidates and message adapters | Gates B0-B7 | ROS-free synthetic patch suite, message/build tests, recorded RGB-D replay | Clean passing commit plus evidence pointer | `dispatch_ready: yes`, dependency-gated |
| ST-3 | `claude/glm-5.3` | ST-2 | same | Candidate-aware waypoint generation, bounded planning selection, vacuum failure recovery and reason codes | Gates C0-C7 | waypoint, identity, vacuum state-machine, action and fault-injection tests | Clean passing commit plus evidence pointer | `dispatch_ready: yes`, dependency-gated |
| INTEGRATION | `claude/glm-5.3` | ST-1,ST-2,ST-3 | same | Clean build, synthetic matrix, recorded replay, real-cell dry-run and sealed-pick matrix | Gates D0-D8 and evidence gate E | Tested integration commit and complete evidence manifests | `colcon`, replay, no-motion and supervised real-cell runs | `dispatch_ready: yes`, dependency-gated |

The approved plan is dispatched through one generation-1 thread per row. The
scheduler may offer only ST-1 initially; ST-2, ST-3 and INTEGRATION remain
dependency-gated until their authoritative prerequisite threads close with a
passing result.

## Acceptance gates

### A. Dynamic top-surface gates

#### A0 - architecture and dependency isolation

- Plain `python3 -m pytest` imports the new geometry/evaluator algorithms with
  ROS unavailable.
- Algorithm modules contain no ROS/message/TF imports and no I/O.
- All outputs carry the original acquisition stamp and actual 3D frame.
- Buffer occupancy remains within the existing 15-frame/1.0-second contract;
  stale/cross-epoch/cross-instance fusion count is exactly zero.

#### A1 - scene independence

Replay each identical RGB/depth/mask/TF observation with pickup-source XY set
to `(0,0)`, `(-1,0)` and `(5,-3)`, and workspace half-extents set to `0.1` and
`5.0` m while hardware crop/predicate parameters remain false.

- Top validity, candidate count, candidate IDs and reason codes are identical.
- Every numeric top/candidate field differs by <= `1e-9`.
- Online code reads no spawn/GT pose and no pickup-source XY before geometry
  and candidate production.

#### A2 - synthetic dynamic-position matrix

Use three catalog sizes (carryon, standard, large), relative XY offsets
`dx,dy in {-0.30,0.00,+0.30}` m, yaw `{0,30,60,90}` degrees and noise seeds
`{11,29,47}`: 324 cases. Each case contains aligned 640 x 480 RGB-D, realistic
missing depth at 5%, Gaussian Z noise sigma 2 mm, 2% uniform outliers and a
YOLO bbox expanded 10% beyond the true projection.

- Valid top rate >= 0.99 overall and >= 0.98 in every size/offset cell.
- XY centre error P95/max <= 20/35 mm.
- top-Z error P95/max <= 10/15 mm.
- width and depth absolute error P95/max <= 30/50 mm per axis.
- for aspect ratio >= 1.15, yaw modulo 180-degree error P95/max <= 5/10
  degrees;
- for aspect ratio < 1.15, `yaw_valid=false` in 100% of cases;
- accepted platform/background-only false top count is exactly zero.

#### A3 - bbox contamination and multi-plane selection

For each size, test 12 cases: four background fractions `{10,30,50,70}%`
times three seeds, including platform, a higher 0.04 m distractor occupying
10% of ROI, and a robot-self-mask hole.

- Correct instance-connected top is selected in at least 34/36 cases.
- The two permitted non-selections fail closed; wrong-plane valid output is
  exactly zero.
- A higher plane below 60% of the best connected support area never wins.

#### A4 - rectangle recovery

For rectangular surfaces with PCA deliberately biased by `{0,15,25,40}`
degrees through partial visibility, the full-range rectangle returns yaw error
<= 5 degrees and per-axis size error <= 30 mm for at least 46/48 cases. Any
other case must return invalid/low-confidence, never a valid error beyond
10 degrees or 50 mm.

#### A5 - recorded real RGB-D position replay

Record, without robot motion, three box sizes at five visible pickup positions
(centre plus +/-0.25 m along world X/Y) and yaw `{0,45,90}` degrees: 45 cases,
one settled 3-second window per case. Aggregate the last 10 valid observations.

- at least 44/45 cases produce a valid top within 1.5 s of the first accepted
  YOLO instance;
- median/P95 temporal XY spread <= 5/12 mm and top-Z spread <= 4/8 mm;
- measured-reference XY P95 <= 30 mm and top-Z P95 <= 15 mm;
- moving `scene_tf` pickup-source values in offline replay changes no result;
- every miss has a replayable T2 bundle and stable reason code.

### B. Suction-patch gates

#### B0 - configuration

- Config schema rejects missing version/frame/footprint/thresholds, NaN,
  negative dimensions, duplicate keys and footprint dimensions above 0.30 m.
- Hardware launch prints absolute config path and SHA-256 and refuses an
  example/simulation fallback.
- Candidate records contain the same model version/hash.

#### B1 - uniformly planar surfaces

Test footprint sizes `{0.08x0.08, 0.12x0.12, 0.18x0.18}` m, tilt
`{0,3,6}` degrees, Z noise sigma `{0,1,2}` mm, yaw `{0,30,60}` and three
seeds: 243 cases.

- Candidate-present rate >= 0.99 overall and 1.00 for noise <= 1 mm.
- selected position lies inside the true eroded surface in 100% of cases;
- selected normal error P95/max <= 2/4 degrees;
- no reported metric differs from its independent reference by more than
  0.5 mm or 0.5 degree.

#### B2 - cross-plane rejection

Generate two parallel planes whose step crosses the panel footprint, with step
height `{2,4,5,6,10,20}` mm, boundary angle `{0,45,90}` degrees, boundary
offset `{-40,0,+40}` mm and three seeds: 162 cases.

- Every candidate crossing a step >= 6 mm is rejected: unsafe accept count 0.
- Every candidate with two height modes separated by >= 5 mm and each holding
  >=15% of the footprint is rejected: unsafe accept count 0.
- At the exact thresholds, comparisons follow the specification (`>` for the
  4 mm adjacent-step gate, `>=` for the bimodal gate) and are deterministic.
- Results below rejection thresholds are reported diagnostically and are not
  counted as proof of physical sealability.

#### B3 - valid local island

Use globally uneven tops with exactly one planar island that fits the full
footprint plus 15 mm margin; place the island at centre, four quadrants and
near one mask edge, with three yaws and three seeds: 45 cases.

- For the 36 interior-island cases, an accepted candidate lies wholly on that
  island in 36/36 cases.
- The nine edge cases fail closed when the physical margin is unavailable.
- No selected footprint crosses a plane label or mask boundary.

#### B4 - no valid patch

Test ridges, folds, central seams, sparse depth, two-level checkerboards and
surfaces smaller than the footprint, five seeds each: 30 cases.

- Candidate count is zero in 30/30 cases.
- Detection returns `DETECT_NO_SEALABLE_PATCH` while preserving valid box-top
  diagnostics where applicable.

#### B5 - identity and time

Inject mismatched stamp, frame, generation and instance ID one at a time, plus
a 1.1-second-old observation.

- Each case fails before planning with its exact identity/stale reason.
- Cross-stamp, cross-frame, cross-generation and cross-instance acceptance are
  each exactly zero.

#### B6 - determinism and bounded resources

- Repeating each B1-B4 input 20 times produces byte-identical candidate IDs,
  rank and reason codes; numeric differences are <= `1e-12`.
- Published accepted candidates never exceed five; retained rejected
  diagnostics never exceed 64 candidates per frame.
- On 640 x 480 input, evaluator P95 latency <= 50 ms on the designated no-GPU
  site CPU, RSS growth slope <= 2 MiB/min over 20 minutes, and no internal
  queue exceeds its configured bound.

#### B7 - visualization

The debug overlay and RViz markers display plane labels, configured footprint,
accepted rank and rejected reason without feeding any value back into online
selection. For a fixed 12-case visual fixture, all expected footprint corners,
candidate IDs and rejection colours match the golden JSON within 2 pixels.

### C. Planning, retry and vacuum gates

#### C0 - candidate use

- Built attach pose equals selected `contact_pose` within 1 mm and 0.5 degree;
  altering `box.pose` while holding candidates fixed changes no pick waypoint.
- Candidate local normal defines approach/recovery direction.
- `top_surface_valid=true` with zero candidates cannot build a pick sequence.

#### C1 - reachability selection

Given five ranked candidates with outcomes `IK fail`, `collision`, Cartesian
fraction 0.94, valid, valid, planning selects the fourth. Required Cartesian
fraction is 1.0 for approach, attach and retry retreat. All rejected reasons
and candidate IDs are preserved.

#### C2 - bounded attempts

- At most three candidates and one vacuum enable per candidate are attempted.
- No candidate ID repeats in one request.
- Candidate exhaustion returns `SUCTION_CANDIDATES_EXHAUSTED` within 180 s and
  leaves vacuum off, DI0 low and no attached planning-scene object.

#### C3 - seal success

For fake DI0 rise delays `{0.0,1.0,6.4,7.9}` s under the existing 8.0 s backend
timeout, the first candidate seals once, attaches the scene object once and
executes `pick_retreat` once. No retry recovery runs.

#### C4 - first-candidate seal failure and second success

The trace must be exactly:

```text
attach C1 -> vacuum on -> timeout -> release -> DI0 low 0.5 s
-> reverse >=80 mm -> settle -> plan C2 -> attach C2 -> vacuum on
-> seal -> scene attach -> pick_retreat
```

Event order violations, lateral motion before release confirmation, duplicate
scene attachment and vacuum-on lateral motion counts are all zero.

#### C5 - recovery failures

Inject stuck-high DI0, unknown DI0, release timeout, recovery Cartesian
fraction 0.99, controller failure, generation change and graph loss.

- Each terminates with `SUCTION_RETRY_RECOVERY_FAILED` before lateral motion.
- Stuck-high/unknown DI0 preserves the safe vacuum/recovery fault state.
- No next-candidate plan, new detection or placement commit occurs.

#### C6 - pre/carry failure boundary

Before confirmed DI0, failures remain pre-attachment and return only through
explicit retry/non-moving recovery. After confirmed DI0, any pressure loss or
motion failure enters carry fault and preserves vacuum until explicit release
recovery. Fault-injection state-transition coverage is 100% for every branch.

#### C7 - no-motion hardware dry run

With real D555/TF and motion disabled, collect ten detections at each of five
box positions. Every request prints ranked candidate poses/metrics/config hash;
no motion or vacuum command occurs. Candidate repeatability satisfies A5, and
the operator can identify the selected footprint in the recorded overlay.

### D. Integration and real-cell acceptance

#### D0 - revision and build

Test one clean committed revision with `git status --porcelain` empty. It must
descend from base revision `8f4d0a0dc8c449197caf146bd3a6ec6cf2fba627`
and the exact approved plan revision. Record submodule state, package versions,
CPU/GPU profile, config hashes and build commands.

Exact build procedure from workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to luggage_msgs luggage_perception luggage_planning
colcon test --packages-select luggage_msgs luggage_perception luggage_planning
colcon test-result --verbose
cd deployment_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select elfin_trajectory_executor
colcon test --packages-select elfin_trajectory_executor
colcon test-result --verbose
```

All selected builds/tests pass with zero failure/error and zero unexpected
skip. This site has Jazzy only; sourcing Humble or an inherited Humble overlay
invalidates the run.

#### D1 - focused ROS-free commands

The implementation must create these stable test entry points:

```bash
python3 -m pytest -q src/luggage_perception/test/test_dynamic_top_surface.py
python3 -m pytest -q src/luggage_perception/test/test_suction_patch_evaluator.py
python3 -m pytest -q src/luggage_planning/test/test_suction_candidate_planning.py
python3 -m pytest -q src/luggage_planning/test/test_suction_retry_state_machine.py
```

Each command passes all cases with no skip. Matrix reports are written by
`python3 -m luggage_perception.eval.dynamic_suction_acceptance --config
<locked-config> --out <evidence-run>` and replayed with the same command plus
`--replay <case-dir>`.

#### D2 - real-cell surface fixtures

No hardware changes are made. Use removable test objects/surface fixtures as
workloads:

- `FLAT`: one continuous flat top;
- `LOCAL_PATCH`: uneven/two-level top with one region large enough for the
  configured footprint plus margin;
- `NO_PATCH`: a height discontinuity >=10 mm crossing every possible panel
  placement.

For each class use five visible XY positions (centre and +/-0.20 m in X/Y),
three yaw values `{0,45,90}` and one run per combination: 45 attempts. Position
order is predeclared and interleaves classes; no case is selected based on an
earlier result.

#### D3 - perception result on real fixtures

- `FLAT`: accepted candidate in at least 14/15 attempts.
- `LOCAL_PATCH`: candidate wholly inside the independently marked valid patch
  in 15/15 attempts; candidate-centre error from the patch interior reference
  <=30 mm.
- `NO_PATCH`: motion authorization count exactly zero in 15/15 attempts and
  reason `DETECT_NO_SEALABLE_PATCH` or a more specific patch rejection.
- Unsafe candidate crossing the marked >=10 mm discontinuity: exactly zero.

#### D4 - supervised sealed-pick subset

After D3 passes, execute existing real-cell Gate-5 pick procedure with an
e-stop operator. Use five `FLAT` and five `LOCAL_PATCH` cases selected before
the run, covering at least three XY positions and all three yaws. `NO_PATCH`
remains no-motion.

- At least 9/10 valid-patch attempts establish DI0 within 8.0 s.
- At least 9/10 complete the configured 0.35 m `pick_retreat` while DI0 remains
  continuously high and the payload remains visibly attached.
- `LOCAL_PATCH` selected footprints cross no independently marked step.
- Any first-candidate timeout exercises the bounded recovery contract; a
  successful second candidate counts as successful but is reported separately.
- No uncontrolled contact, lateral motion during contact, duplicate hardware
  owner, stale candidate, collision, emergency stop or operator intervention
  is permitted. Any occurrence stops the gate and is a failure, not an
  excluded sample.

#### D5 - retry-specific real-cell cap

Retries are enabled only after all synthetic C gates and five single-candidate
real seals pass. At most two supervised retry trials are allowed in one run
day. A retry proceeds only from a normal `VACUUM_SEAL_TIMEOUT`; all other
failures stop. Both trials must obey release-low hold, >=80 mm reverse and
zero lateral motion before retreat. This gate checks safe sequencing, not a
required induced seal failure.

#### D6 - false authorization and state integrity

Across all A-D cases:

- online GT/fixture-pose reads: 0;
- `scene_tf`-driven detection/candidate changes: 0;
- stale/cross-identity candidates: 0;
- attach authorization with no accepted candidate: 0;
- motion authorization on `NO_PATCH`: 0;
- lateral retry motion before confirmed release: 0;
- duplicate scene attach/commit: 0.

#### D7 - latency and resources

On the locked no-GPU site profile, report end-to-end from accepted YOLO result
to ranked candidates:

- P50 <= 250 ms, P95 <= 500 ms and max <= 800 ms over at least 300 frames;
- candidate evaluator P95 <= 50 ms;
- accepted candidate output >= 2 Hz when YOLO is producing >=2 Hz;
- all buffers stay within declared bounds;
- workload-adjusted RSS slope <=2 MiB/min per touched online perception node
  over 20 minutes.

#### D8 - teardown and final decision

Only one simulator or hardware graph may own resources. Simulation work uses
the simulator lifecycle rule and `scripts/stop_sim.sh`; hardware work verifies
there is one CPS executor, one D555 owner, one scene TF owner and one Livox
owner. Teardown must leave zero task-owned launch/record/eval processes.

The unified task passes only when A0-A5, B0-B7, C0-C7 and D0-D8 pass on the
same integration revision. A failure freezes evidence and stops blind reruns.

## Evidence and dump matrix

All evidence lives under
`docs/status/evidence/dynamic_suction/<run-id>/`. T0 manifest/result is required
for every run; T1 boundary trace is required for every scored case.

| Boundary | T1 trace | Failure trigger | T2/T3 payload | Window | Format / per-case budget | Replay or inspection |
|---|---|---|---|---|---|---|
| RGB-D and TF ingest | stamp/frame/generation, dimensions, valid-depth rate, TF age/motion score | missing/stale TF, invalid depth, rate collapse | RGB, aligned depth, camera info, exact stamped TF and joint window | 1 s pre / 2 s post | MCAP + PNG/NPY, <=80 MiB | `dynamic_suction_acceptance --replay <case> --stage ingest` |
| YOLO/mask | instance ID, bbox/mask area, backend, confidence, latency | no proposal, bbox component failure, mask mismatch | RGB, raw mask, eroded mask, detections and backend stats | 1 s pre / 2 s post | PNG + JSON, <=25 MiB | same command, `--stage segmentation` |
| Depth component/cargo cloud | pixel counts, seed depth, component count, join IDs | coverage/point threshold, wrong component, identity mismatch | depth ROI, component labels, camera/world cargo points | one acquisition plus adjacent frames | NPZ + JSON, <=50 MiB | same command, `--stage cargo` |
| Plane/rectangle | candidate IDs, normals, inliers, residuals, score, chosen plane | wrong plane, geometry threshold, invalid rectangle | bounded plane candidates/inlier masks, hull, PCA eigenpairs, rectangle | failing acquisition | NPZ + JSON, <=40 MiB | same command, `--stage top` |
| Suction patch map | config hash, grid size, accepted/rejected counts/reasons, rank metrics | unsafe accept, expected candidate missing, latency | height/normal/label maps and <=64 candidate footprints; T3 only adds cell metrics | failing acquisition | compressed NPZ + JSON + PNG, <=50 MiB | same command, `--stage suction` |
| Candidate to waypoints | request/candidate IDs, TF identity, ranks, rejection reasons, waypoint poses | stale identity, IK/collision/fraction failure, wrong candidate | detected object, candidates, TF, robot/scene snapshot, all attempted waypoints | request start through selection | JSON + MCAP, <=30 MiB | `python3 -m luggage_planning.eval.suction_retry_replay --case <case>` |
| Motion/controller | candidate/segment IDs, plan result/fraction, controller result, settling | plan/execute/contact/recovery failure | MoveIt requests/responses, trajectory and planned-vs-actual joints | 1 s pre / 3 s post | MCAP + JSON, <=100 MiB | same retry replay plus trajectory inspector |
| Vacuum/retry state | candidate ID, DO0/DO1/DI0, state transitions and monotonic times | seal timeout, DI ambiguity, release/recovery/order violation | complete bounded IO/state trace and correlated controller events | 1 s pre / 10 s post | CSV/JSON + MCAP, <=20 MiB | `suction_retry_replay --case <case> --stage vacuum` |

Evidence limits and health:

- target <=200 MiB per failed case after compression, <=500 MiB per real-cell
  attempt and <=6 GiB retained for one full integration run;
- T1 overhead P95 <=5% of no-capture latency; T2 is frozen only on failure;
- every manifest lists hashes, sizes, missing artifacts,
  `capture_complete` and `replay_possible`;
- missing/unjoinable/truncated artifacts or replay disagreement makes the run
  `evidence_invalid`; instrumentation is fixed before repeating that gate;
- preserve failed bundles and final passing summaries. Passing raw frames may
  be pruned after aggregate verification; never delete a failed bundle during
  the task lineage.

## Required reports

The integration summary must contain:

- exact code and plan revisions, dirty count and config hashes;
- counts and denominators for every A-D gate;
- per-position/size/yaw/surface-class results;
- top, patch and end-to-end latency distributions;
- every rejected plane/candidate reason count;
- first- versus later-candidate seal rates;
- DI0 seal-time distribution and retention through retreat;
- all exclusions (only predeclared infrastructure/evidence invalidity is
  excludable), failures and evidence paths;
- process ownership and teardown result;
- final `pass`, `fail` or `inconclusive` with no relabelling of unsafe events.

## Risks and mitigations

- The 0.18 x 0.18 m footprint is a conservative URDF envelope, not measured
  sealing-ring geometry. Hardware launch requires a versioned config so the
  value is visible and replaceable; no smaller hidden footprint is allowed.
- Bbox depth components can merge box and similarly deep clutter. Multi-plane
  connected support and fail-closed scoring mitigate this; pixel instance
  masks remain preferred.
- A rigid footprint may reject surfaces that the real panel can tolerate.
  This is an availability cost, not an unsafe acceptance; threshold relaxation
  requires a new evidence-backed generation.
- D555 holes can remove otherwise valid patches. Multi-frame fusion is not in
  generation 1 because the box may move; a future generation may add settled,
  identity-locked fusion with explicit motion/reset evidence.
- Near-square boxes have unobservable yaw. Candidate local normal and patch
  validity remain usable, while tool yaw uses the existing safe fallback.
- Vacuum retry adds motion after a failed contact. Its release confirmation,
  reverse-only recovery, attempt cap and fault-injection gates are mandatory;
  retries stay disabled until C0-C7 pass.
