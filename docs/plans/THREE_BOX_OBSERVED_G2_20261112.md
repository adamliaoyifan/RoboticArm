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
