# Three-box observed full-model loading, generation 2

- owner: codex/gpt-6
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- generation: 2
- base_revision: 4909614c70b79e90fea8c725ee5e760b59be2ea2
- authorization: User active goal, 10-hour task budget; minimum changes, post-place explore is the only new product capability.
- wall_budget: 2026-11-12T15:39:15Z through 2026-11-13T01:39:15Z

## Scope

Complete three actual full-S20 Gazebo Fortress box cycles through the explicit
joint_operation route, measured detection/domain and the persistent observed O
owner. First two boxes floor-supported, third explicitly stacking on measured
support, as user confirmed. No prior/template placement, GT geometry, map painting,
unknown-as-free substitution or reset between boxes. Initial empty declaration
remains allowed exactly once. Reuse existing native planning/execution and view
generation; do not introduce another planner, framework or speculative optimization.
Only new product capability is bounded post-release explore if existing views
cannot complete the observation loop. Persistent cycle driving and the already
authorized stack workload are integration wiring, not separate planners.

User requests O inflation .02m instead of .05m. Set the diagnostic profile's shared
O/placement resolution .02m as well; retain O and exact static ULD collisions.
Do not change release height, actual grasp, model collision meshes or other margins
without measured reason and user discussion. Explore ranks a small geometry-derived
view set by currently missing target/ULD footprint visible in the measured camera
frustum, then existing MoveIt collision-checks motion. FOV overlap is a ranking
estimate; actual newly integrated observations determine success. LiDAR can improve
coverage when actual acquisition TF, source and self-return behavior are verified.

## Procedure and acceptance

Environment: production prefix, current deploy_update source, full robot/sensors,
headless Fortress, ROS_DOMAIN_ID=7, exactly one Gazebo stack/clock, O exclusive of G.
Physics spawns carryon x3 (nominal .55x.40x.25m, yaw0, no XY jitter) using the
existing grouped profile; these dimensions are spawn/eval-only. Online dimensions
must come from the measured detector. One three-box successful witness is required;
AND of three dependent functional units, not a statistical per-unit reliability
claim. No production success-rate or latency SLA is inferred.

1. Freeze owned edits at commits before live tests. Hash and archive every
effective dirty/untracked input in run/build receipts; reject changed source
between build and run. Check installed origins, full model/scene and atlas receipt.
Run production build/check_install and focused behavioral tests covering changed
parameters, observation ranking, bounded fallback, actual plan execution and
persistent-cycle map identity. All selected deterministic cases must pass once.
2. Rehearse startup/readiness without scoring: <=180s/attempt, <=15min per repair
round. Require both controllers active, six joints, sensor messages, exactly one
clock, O receipts, prepare/native/execute services. A startup failure is not a box
attempt; preserve and repair it before progressing.
3. Diagnostic first floor box: use existing driver and actual sensor O, complete
all six retained trajectory legs and actual vacuum events. Preserve first failed
boundary before any repair/retry. No success claim for planning alone.
4. Add only demonstrated missing observe/driver wiring. Freeze repairs and repeat
focused checks. For three-box witness, preserve before/after each box: exact O
lease/pages/surface, detection/domain, prepare request/trace, plan/certificate,
controller results, actual attachment/release, observed support/adoption and final
scene. Next box consumes the preceding observed map in the same epoch, without
reinitialization. Atlas must be identity-qualified and actual hint/fallback use
recorded; a mismatch cannot silently certify stale hints.
5. All three action results must be success with motion terminal and no retained
payload; subsequent pickup return must reach its measured joint endpoint. First
two selected support heights are floor-relative zero within one source voxel;
third selected bottom height is above one voxel and rests on sensor-observed
support passing the existing shared support and nonpenetration gates. Verify
actual simulated settled poses/contact support separately as eval-only evidence;
third must physically rest on earlier cargo, not hang attached or rest on floor.
Preserve executed trajectories and native validity receipts, forbidden collision
pairs and simulator contact evidence if available. Missing physical-settle or
post-place observation evidence is incomplete, not pass. Current observation
admission thresholds remain unchanged until measured inadequacy is reviewed.
6. Report all stage timings, sensor input/provenance and final native-resolution
O visualization. Capture count for each required stage must be >=1 per box;
otherwise not_evaluated. One complete three-box witness is the final functional
gate, with no silent restart/reset inside it. Tear down owned stack immediately.

Canonical commands:
`scripts/production/build.sh --evidence <fresh>/build`
`scripts/production/run.sh /usr/bin/python3 scripts/production/check_install.py --output <fresh>/install.json`
`scripts/production/run.sh ros2 launch luggage_gazebo sim_world.launch.py profile_config:=<installed-share>/config/joint_operation.profile.yaml operation_run_dir:=<fresh>/run`
`scripts/production/run.sh ros2 run luggage_gazebo joint_operation_sim_driver.py --start --run-dir <fresh>/run --evidence <fresh>/driver`
The final driver adds an explicit bounded three-cycle option using the same ports.
Exact executed options/environment are archived per run.

## Evidence and review

Root: docs/status/evidence/three_box_observed/g2/. Fresh directories only.
Per run <=2GiB and total <=16GiB; lossless compression, no failed-case deletion.
T0 revision/config/dirty/source/install hashes, commands, readiness and teardown;
T1 all stage IDs/timestamps, map/plan/grasp joins, stage timings and reason codes.

| Boundary | Trigger | Required replay payload | Window/budget | Inspection |
|---|---|---|---|---|
| Input/TF/O | missing/stale/wrong frame or coverage | actual image/K/cloud, stamped TF/joints, O receipts and source pages | latest settled frame plus <=10s bounded ring, <=512MiB | typed CDR manifest and O render |
| Placement/prepare/native | refusal or timeout | pinned O, surface, detection/domain, requests/replies/trace, q/TCP, phase scenes and collision pairs | one failed request <=512MiB | prepare trace and native replay |
| Execute/attach/release/explore | failure and successful cycle | original plan, correlated results, actual attachment, scene/map before/after, view ranking and actual evidence gain | one cycle <=512MiB | runtime/driver manifests |
| Final physical outcome | each box | simulator poses/contact support as separate reference plus measured O adoption | settled boundary <=128MiB | eval-only comparison |

Capture health requires nonempty typed boundary inputs, hashes, stage association,
capture_complete/replay_possible and explicit missing-item list. Incomplete capture
is repaired before another live attempt. Review after first full ULD snapshot,
first floor cycle, measured stack-support qualification and final three-box run.
Any deeper redesign, evidence relaxation or release-policy change requires user
discussion; ordinary wiring/error fixes proceed under the existing authorization.

## Workload and execution watchdog clarification -- 2026-11-13T04:00+08:00

The unchanged `carryon` catalog mesh's actual spawn bounds are approximately
.515886x.390038x.237463m in the captured run, rather than the nominal
.55x.40x.25m shorthand above. Use the actual recorded catalog selection and
physical model bounds for evaluation only; online geometry still comes from
perception. No GT dimensions or poses enter placement/planning.

The shared explicit profile now exposes a600s execution/observation watchdog,
matching the existing runtime maximum, after diagnostic27's retained253.56s
carry exhausted the former300s client literal once certification overhead was
included. The client records trajectory duration and budget and refuses before
motion if trajectory duration alone meets/exceeds the budget. This supersedes
only that diagnostic timeout value; no production latency SLA is established.

## Simulator sensor source qualification -- 2026-11-13T04:30+08:00

The explicit joint profile selects `server_render_engine: ogre` after isolated
same-pose known-plane measurements reproduced OGRE2 false floor occupancy
(+7.918mm target-area error) and OGRE reduced it to+0.413mm with no target-area
points above the configured inner floor. This is a simulation source repair,
not a real-LiDAR calibration or planner/O tolerance. Bare launch stays OGRE2;
resolved run metadata records the backend. Full-model RGB-D compatibility must
be demonstrated by actual measured detection and source outputs in the next run.

## Release observation review and persistent cycle wiring -- 2026-11-13T05:50+08:00

User's10h delegation supersedes the earlier wait-for-discussion step for this
measured admission review; recommendation and reason are recorded in
`docs/status/evidence/three_box_observed/g2/wiring59/decision.json`.
Diagnostic37 completed all6 original motions and3 actually checked camera views,
with actual dynamic release/settle. The action still refused at298/475 observed
columns. Eval-only physical mesh raycast proves38 demanded columns cannot match
at nominal target height;140 attainable columns remain unobserved. The old100%
rectangle gate therefore conflates missing observation with a false geometry
assumption. No reference mesh/pose is supplied to online consumers.

For the explicit joint profile only, release observation uses the existing
`min_support_ratio` as its minimum directly occupied, height-consistent column
fraction (currently0.6); unconfigured runtime defaults1.0. This is an explicit
coarse observation completion witness, not complete box reconstruction, physical
identity proof or stacking permission. Capture acquisition must still postdate
actual settle/release. Missing cells remain unchanged. No support/tilt/volume,
collision, inflation, release-height or motion certificate threshold is changed.
Future placements consume actual complete O and retain every original hard gate.

The same diagnostic37 O has0 admissible stack candidates;16 near-passing windows
have support>=0.6 but0.04m height span, exceeding the unchanged0.05 tilt ratio.
This remains an actual blocker, not reported as successful stacking. Full3box
functional gate remains AND over3 observed successful terminal empty-tool cycles,
same persistent epoch and separate physical support reference. Added driver flag
`--three-box` uses floor/floor/stack intent, exact pinned actual O,4 directions,
existing query/ranking and native complete route. Finalize preserves each physical
box before next spawn; return view retains full O but adds no phantom pickup.
Raw capture rotates per box and triggers on post-place exploration/failure;
all typed plan/map/certificate boundaries persist independently. No prior
placement, simulator geometry as online input, new planner or cache is added.
104 focused behavioral cases pass; build/install/live evidence follows.
