# Frame migration -- module tasks and staged verification

Parent: `FRAME-MIGRATION-20260929`. Planning baseline:
`89b7a0032b84844522d38bbb419693a5705f415a` plus the explicitly inventoried shared
worktree. Contract authority: `docs/architecture/frame_contracts.md`; interface
IDs: `docs/architecture/frame_interfaces.json`. This plan decomposes the user's
requested migration into independently reviewable module tasks and phase gates.

## Execution policy

- Concrete implementation owner for every row: `eng / codex / gpt-6`, CLI codex.
  Each owner implements, verifies, fixes and commits its own unit. No separate
  test-role handoff or child-agent delegation is required.
- Only ST00 is dispatch-ready in generation 1. The remaining rows are explicit
  queued work, not permission to run early. Before release, reviews binds that
  task to the exact integration commit containing its passing prerequisites,
  resolves any deferred acceptance items below, and publishes a higher
  generation with an exact plan revision and `dispatch_ready: yes`.
- Generation-1 queued threads use the planning baseline above. This does not
  pretend that future dependency commits already exist. Do not claim them
  against an old base or rewrite an already claimed generation.
- Coordination stays in the primary `docs/agents/` tree, including when code
  work uses an isolated worktree. Do not switch or clean the shared worktree.
- Saving WIP in a named integration branch is not certifying it or merging it
  into master. Module owners review ownership/dependency closure first. Only
  explicitly scoped files are committed; unrelated changes remain with owners.
- Existing v1 behavior and new v2 behavior must not silently share a bare-pose
  interpretation. Use versioned ROS bindings/endpoints and explicit adapters
  while migrating. A new producer and its consumers switch as a closed chain.
- Each phase is a stop gate for dependent work: preserve failure evidence,
  repair locally, rerun the affected gate, then release the next task. No
  downstream workaround, hidden identity TF, default frame or empty-map fallback.
- No hardware motion is authorized by this plan. Hardware-profile validation is
  offline/plan-only. Simulation is reserved for final integration after rehearsal.

## Tasks and dependencies

The exact immutable plan commit is recorded as `plan_revision` in every dispatch
thread. Each passing result records its implementation commit and evidence URI;
no task is marked done merely because its plan exists.

| Phase | Task | Module / interface ownership | Depends on | Generation-1 state |
|---|---|---|---|---|
| 0 | ST00 | Preserve current source identity and rerun offline baseline; all F IDs inventoried | none | ready |
| 0 | ST01 | Isolated integration source baseline, dependency closure and clean build | ST00 | queued |
| 1 | ST02 | Shared frame contracts, v2 ROS bindings/adapters, full transforms, diagnostics; F01 and shared F08/F15 substrate | ST01 | queued |
| 2 | ST03 | Sensor/observation identity, detection/contact/opening, pick and retry boundaries; F02–F05/F09/F10 | ST02 | queued |
| 2 | ST04 | Cargo map, measured integration and add/remove ledger; F06/F19/F20 | ST02 | queued |
| 2 | ST05 | Placement solver, candidates and SlotSpec producer; F07/F08 | ST03,ST04 | queued |
| 3 | ST06 | Atlas builder/query/task-grid semantics and certification; F14 and seed side of F16 | ST04,ST05 | queued |
| 3 | ST07 | Container paths, sweep, selector service and place target conversion; F11–F13 | ST05,ST06 | queued |
| 4 | ST08 | Motion message consumers, IK boundary, scene, attach/detach; F15–F18 | ST03,ST07 | queued |
| 5 | ST09 | Startup/profile gating, eval/GT/replay/viz, legacy isolation; F21–F24 | ST08 | queued |
| 6 | INTEGRATION | Actual ROS graph, complete cycle, migration gate and merge evidence | ST09 | queued |

ST03 and ST04 have independent ownership domains, but the single current owner
may run them sequentially. ST06 is never run against old slot/map semantics.
Shared F08/F15/F16 rows close only after their last listed consumer task passes;
an earlier producer-only pass does not verify the entire inventory row.

## Common deterministic verification matrix

This matrix applies to new module gates; ST00 uses the already committed 50
cases unchanged. Module-specific additions below are also required.

- Container transform yaw: 0, π/2, π, atan2(4,3).
- Translation: (0,0,0), (1.5,-0.4,0.2).
- Container floor z: 0.53, 0.71.
- Support height above floor: 0, 0.30 m.
- Non-square payload: WDH=(0.80,0.50,0.32) m.
- Grasp offset in box axes: (0,0,0.16), (0.10,-0.05,0.16) m.
- Cartesian product: 4×2×2×2×2 = 64 cases per relevant geometry boundary.
  Sensor ingress uses these transforms on a fixed stamped synthetic point set;
  it does not rerun YOLO 64 times or use catalog dimensions as live perception.
- Negatives: missing frame, wrong frame, wrong reference point, wrong contract
  version, wrong units, stale scene revision, missing observation stamp, wrong
  time basis, malformed/nonfinite pose, nonunit quaternion, stale map revision,
  wrong hull identity. Each applicable negative runs once with its exact
  documented ContractError code; inapplicable items must be named per task,
  never silently counted as covered.
- Additional orientation pair: q/-q equivalent; tool yaw 0/π distinct. A
  container roll=0.2 rad is rejected by the 2.5D admission gate while the full
  transform math itself remains capable of SE(3).

Metrics: analytic position/vector error ≤1e-9 m; unit-quaternion sign-equivalent
component error ≤1e-9 for pure Python operations, ≤1e-6 through ROS serialization.
Identity strings, nanosecond stamps, reference kinds and revision IDs match
exactly. Every invalid input must be rejected before downstream planning,
execution, vacuum or occupancy mutation; the spy/fake counter for those effects
must be zero. Missing samples => `not_evaluated`, never pass.

Every fixed case is run once. Aggregate is AND over deterministic obligations:
required per-case correctness is 100%, no flaky-case allowance or retry-to-green
selection. This checks semantic correctness, not robot success-rate reliability.
An injected wrong-frame producer and an injected position-only rotation must
make each applicable module gate fail, demonstrating that the gate detects the
defect class rather than just message spelling or A→B→A roundtrip agreement.

## ST00 -- source identity and current baseline

Scope: read-only inventory and tests, plus evidence and lifecycle notes. Do not
commit other owners' WIP, alter active messages, launch simulation or declare the
source tree clean. Environment: Python 3.10 / installed ROS 2 Humble environment,
CPU only. The original scene helper requires sourced ament; new check modules
remain ROS-free.

Commands from primary workspace (use this fresh run directory exactly once):

```bash
source /opt/ros/humble/setup.bash
python3 scripts/check_frame_contracts.py --mode baseline --output docs/status/evidence/frame_migration/ST00-20260929/baseline
python3 scripts/check_frame_contracts.py --mode migration --output docs/status/evidence/frame_migration/ST00-20260929/migration
PYTHONPATH=src/luggage_description:$PYTHONPATH python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py -v
```

Acceptance: baseline exit 0, exactly 50 cases pass; focused suite exactly 15
tests pass; migration exit 2 with exactly F01–F24 pending (24). These expected
pending results are successful diagnosis, not successful migration. Enumerate
all inventory sources absent from `git ls-files`, not just the first examples.
Archive the exact manifest source list and record its archive hash. Verify the
archive contents against the manifest SHA-256; zero missing/mismatched entries.
Record HEAD, dirty count, current branch and no-sim status. Whole-build closure
and clean-checkout usability remain explicitly unverified until ST01.

Required evidence: baseline/migration five-file bundles, unittest.log,
untracked-interface-sources.txt, sources.tar.gz, source-archive-verification.json,
and a summary. Source inventory may change before execution; if the expected
50/15 or 24 pending counts change, explain the new revision and supersede the
generation rather than silently lowering the bar.

## ST01 -- isolate the complete source baseline

Scope: map the 13 currently untracked interface dependencies and all required
tracked modifications to their feature owners/lineages; record exact dependency
closure. Produce a named `integration/frame-migration-*` worktree/branch from
the pinned baseline and scoped feature commits or clearly labelled WIP snapshot
commits. Do not merge that snapshot into master or overwrite another checkout.

Acceptance: every F01–F24 source is tracked in the integration commit; every
required build/import dependency has a recorded source/installed-package origin;
zero unexplained dirty files in the tested checkout. Existing behavior remains
unchanged by this task. Re-run ST00 numerical cases; the pending count stays 24.
Clean Humble build of the relevant installed packages must succeed:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to luggage_description luggage_msgs luggage_perception luggage_packing luggage_planning luggage_gazebo
```

Record colcon return code 0, package summary, resolved Python module paths and
package prefixes. If a feature snapshot does not build, repair only its proven
dependency/wiring issue or leave this task open; do not spend a sim run. Before
dispatch, pin the exact feature source set and build dependency profile in the
next generation so “copy whatever is dirty” is not the implementation contract.

## ST02 -- shared contract and message boundary substrate

Scope: description contracts/transforms, versioned luggage_msgs bindings and
planning/perception message adapters. Select explicit v2 endpoint/type names in
the dispatch generation; never reuse v1 bare poses with a changed interpretation.
Add a shared boundary diagnostic containing interface/request/producer/consumer,
declared/expected frame, reference point, scene/map versions and stamp basis.

Acceptance: all 64 matrix cases plus applicable negatives pass through actual
generated ROS messages and serialization, not stand-in-only objects. Pose and
waypoint arrays preserve complete orientation and identity. Joint trajectories
preserve joint names/order and are not labelled Cartesian poses. Empty frame or
version never acquires a default. All adapters return independent objects.

Verification: clean colcon build of luggage_msgs and direct dependents; implement
and run `python3 -m unittest discover -s src/luggage_planning/test -p
test_frame_migration_messages.py -v` after sourcing the isolated install.
Test log reports ≥64 positive cases and all applicable named negatives with
zero unexpected accepts and zero identity loss; emit per-case results, not just
one unittest method count. Require successful build/import of both explicitly
supported v1 and v2 bindings before enabling any v2 consumer.

## ST03 -- perception, pick and recovery

Scope: sensor preprocessing frame/stamp admission, detector/contact publishing,
opening geometry, waypoint pick entry and retry/session storage. World detection
and pick remain world; do not change camera extrinsics or reframe 2D masks.
Local aperture/normal/covariance semantics must survive the boundary. Store the
actual box-to-tool grasp transform and bind it to the observation/object.

Acceptance: 64 synthetic geometry cases; acquisition stamp retained exactly;
contact +Z and tool -Z relationship preserved; near-square invalid yaw uses the
declared fallback without inventing measured orientation. Candidate mismatch,
stale observation, missing TF, cross-object grasp and stale calibration produce
their explicit rejection with zero plan/vacuum calls. Retry uses the saved
approach target/link/snapshot and cannot silently rebuild against latest TF.
Opening aperture transformed to world agrees with independently transformed
vertices; covariance rotation must match an analytic diagonal-anisotropic case.

Implement/run `python3 -m unittest discover -s src/luggage_perception/test -p
test_frame_migration_perception.py -v` and `python3 -m unittest discover -s
src/luggage_planning/test -p test_frame_migration_pick.py -v`. Existing detection
and suction tests for touched functions also pass. Synthetic/GT references stay
in test fixtures, never in production perception inputs.

## ST04 -- cargo map and measured commit

Scope: mapper core/node, surface serialization, measured integration, add/remove
ledger and explicit v1 import adapter. V2 surface z is container-origin; unknown
tops null; no planned slot becomes verified occupancy. Preserve geometry hash,
map revision and measured provenance. Existing separate yaw subtraction must
not be applied twice during consolidation.

Acceptance: the 64-case matrix plus non-square rasterization against an analytic
oriented footprint. For raster occupancy compare to exact cell-intersection
oracle, allowing conservative boundary cells only under a declared half-open
grid rule fixed before dispatch; never compare only array sums. Free/occupied/
unknown semantics survive add→serialize→read→remove. Duplicate measured commits
are idempotent, stale revision is rejected, and removed geometry does not erase
independent sensor observations. Dual-height fields agree ≤1e-9 m. Exactly one
declared adapter converts old floor-relative dumps; wrong/missing datum fails.

Implement/run `python3 -m unittest discover -s src/luggage_perception/test -p
test_frame_migration_map.py -v`, with per-case cell mismatches=0 and explicit
provenance/revision results. Pin raster boundary ownership and preservation
behavior in the released task; no dispatch on an ambiguous grid-edge rule.

## ST05 -- placement and SlotSpec producer

Scope: placement_solver, planner node, reachability candidate adapter. Candidate
and returned slot center are container coordinates; size belongs to box axes.
Remove cross-module volume-center ambiguity through an explicit legacy decoder,
not global half-height edits. Support-relative quantities are derived locally.

Acceptance: 64 candidates have center z=floor+support+h/2, box top=center+h/2,
footprint and physical support unchanged under coordinate re-expression. Four
container yaws produce identical container packing geometry. Message output,
placed-input normalization and replan use the same center/yaw meaning. Stack,
unknown support, rejected geometry identity and blocked aperture retain existing
documented reason categories; coordinate errors are not BIN_FULL.

Implement/run `python3 -m unittest discover -s src/luggage_packing/test -p
test_frame_migration_placement.py -v`; all analytic errors within common limits,
zero inconsistent output/replan footprints. Until ST06 passes, tests may inject
a declared neutral atlas fixture; runtime may not silently disable certification.

## ST06 -- atlas and concrete tool branches

Scope: atlas builder/runtime/query and task-grid artifact metadata; target pose
construction, seed transform and cache admission. Build in one declared position
and orientation convention. Existing NPZ/YAML is not relabelled as v2.

Acceptance: 64 analytic target pose cases match container→base full transforms;
tool yaw 0/π remain distinct branches, while q/-q share a rotation. Stale scene,
URDF, payload, tool/reference or v1 semantics reject before seed use. Every
returned seed in the tested new artifact must reproduce its advertised target
under FK: position ≤0.002 m and orientation angular error ≤0.02 rad. Require at
least one sampled reachable seed per directed yaw bin tested; zero samples is
not_evaluated and cannot close the task. A failed branch cannot certify all
π-equivalent grasps unreachable. Unknown remains unknown.

Implement/run `python3 -m unittest discover -s src/luggage_planning/test -p
test_frame_migration_atlas.py -v`; preserve the artifact and every sampled seed,
FK pose, error and branch ID. Before dispatch, bind the exact builder command,
robot/payload profile, bounded grid, directed yaw bins and sampling coordinates
in the next generation using ST01's available model. This artifact/FK gate is
not an IK success-rate claim or permission for a full expensive atlas campaign.

## ST07 -- paths and selector service

Scope: occupancy sweep, path candidates, grid search, selector node/client and
place waypoint target adaptation. Slot center is converted through the chosen
grasp to TCP before path generation. Output conversion includes tool rotation,
not just xyz. The in-process and actual service variants must agree.

Acceptance: 64 target cases plus three fixed map fixtures (open straight path,
one blocked straight corridor with one open detour, fully blocked destination).
Expected outcomes respectively feasible, collision-free detour, explicit
infeasible; zero intersection with occupied/unknown-above-floor cells. Use
non-square payload at both box axes. Service vs in-process winner/waypoints agree
within common tolerances under deterministic tie-breaking. Invalid inputs yield
zero MoveIt calls; returned targets are TCP, not box centers or zero defaults.

Implement/run `python3 -m unittest discover -s src/luggage_planning/test -p
test_frame_migration_selector.py -v` and the real-generated-message selector
suite. Before dispatch, include exact fixture arrays and tie-break expectations
in the plan revision; do not infer pass from a test that merely returns a path.

## ST08 -- execution, scene and attachment

Scope: motion planner/executor entrance, frame/link constraints and compensation,
scene manager, collision adapters, attach/detach, hardware driver serialization.
Do not change valve electronics or arm controller semantics to compensate for a
coordinate error. Preserve the measured grasp/payload relative transform.

Acceptance: real message adapters plus planner/scene/controller test doubles
cover the 64 matrix targets and all negative contracts. Executed Cartesian goal
and collision-box eight corners agree with independent world geometry. Tool/link
identity and joint ordering survive. FK seed vs advertised target obeys ST06
limits. Offset compensation rotates as a vector, not a point. Attach→move→detach
preserves box/tool relation for both centered and off-center grasps. Invalid
frame, stale scene, target-link mismatch or wrong units yield zero downstream
planning/execution/attach calls. This task performs no real hardware motion.

Implement/run `python3 -m unittest discover -s src/luggage_planning/test -p
test_frame_migration_execution.py -v`; preserve exact request/response and side
effect counters. Consumer proofs close F15/F16 only after both producers and
consumers are tested on the same integration commit.

## ST09 -- tooling, startup and legacy isolation

Scope: launch/profile wiring, runtime package identity, all listed eval drivers,
GT dump, replay/viz and legacy exclusion. Keep GT as reference-only. Old dumps
are accepted only by named adapters, not auto-detected from coordinate magnitude.

Acceptance: all 64 analytic fixtures retain world geometry across serialize/
replay and marker conversion; old unknown schemas fail. Startup rejects mixed
v1/v2 endpoints or stale atlas before any motion capability is enabled. Record
actual loaded module paths and code/message versions. Legacy F24 either has
isolation proof (not installed/launched/connected in the tested profile) or is
fully migrated; file location alone is insufficient evidence.

Implement/run `python3 -m unittest discover -s src/luggage_gazebo/test -p
test_frame_migration_tools.py -v` and a no-motion ROS graph startup check from
the isolated install. This task also prepares the exact case and capture-health
runner/configs for INTEGRATION; it does not run scored simulation prematurely.

## INTEGRATION -- whole-chain gate and merge evidence

Prerequisites: every module task closed at a real commit, clean integration
checkout, production interface proofs at that revision, no missing source files,
all relevant generated-message tests passing, and a rehearsal with healthy
capture/startup. No active scene is edited during a scored run.

Required coverage: actual ROS publication/service/action paths from observation
through slot/path/execute/commit/replan, including one injected contract error
before pick and one before place. Invalid cases must yield the named contract
error with zero new motion/commit side effects; while carrying, preserve the
existing vacuum/recovery policy. Valid serialized frame/pose identities remain
consistent at every stage, within the common ROS tolerance.

Simulation smoke uses the current sim profile, carryon then standard (two boxes),
exclusive Gazebo, gui=false/use_rviz=false, ROS_DOMAIN_ID=7. The existing entry
point after ST09's wiring is:

```bash
scripts/motion_occ_place_path_run.sh --ws <pinned-integration-worktree> --out <fresh-evidence-dir> --n 2 --sequence-ids carryon,standard --place-probe on --atlas on --record-bag
```

Offline/deploy-profile pose cases must also pass at container yaw π and nonzero
translation; deploy-profile passing does not authorize robot execution. Final
`check_frame_contracts.py --mode migration` must exit 0, pending list empty,
and all proof hashes match the committed integration source.

Before dispatch, ST09/reviews must freeze the actual sim acceptance extractor,
expected cycle events, numerical measured-placement thresholds, exact config/
atlas hashes and capture-health observations. A two-box smoke is a finite
coverage check, **not** a ≥95% reliability certification. Any claimed statistical
reliability gate needs a separate sample-size/aggregation decision that can
distinguish its healthy and broken hypotheses; no consecutive-pass policy is
invented here. This unresolved live-scoring detail is why INTEGRATION is queued,
not dispatch-ready. Clean source build and deterministic interface acceptance
remain required regardless of whether a statistical campaign is later requested.

## Evidence, capture and phase release

All evidence goes under `docs/status/evidence/frame_migration/<task>-<run>/`.
Each unit records base/plan/implementation revisions, source hashes, dirty state,
command, environment/profile, workload, stage timings, sample counts, expected
and observed values, coverage and missing/not_evaluated items. Each case has a
stable ID joined through the pipeline. No artifact overwrite or blind retry.

| Boundary | T1 | Failure trigger | T2 payload and window | Format / bound | Replay |
|---|---|---|---|---|---|
| ST00 source + baseline | source hashes, case ID/result/timing | any unexpected verdict or source mismatch | exact fixture/input/expected/actual and source archive; one invocation | JSON + tar.gz; 64 MiB/run | checker with saved suite/inventory and pinned sources |
| module algorithm | case ID, contract identity, effect counts | wrong geometry/code/effect | full synthetic input/output, expected oracle, transform and traceback; one case | JSON; 16 MiB/run | named unittest case on recorded commit |
| ROS message/scene boundary | request/observation/frame/link/version/stamp | mismatch, lost field, unexpected side effect | serialized input/output, node config, TF/scene snapshot and spy trace; whole request | JSON/CDR; 64 MiB/run | actual adapter/service harness with saved request |
| atlas build/query/FK | cell, branch, hashes, query/seed status | semantics mismatch, FK error, no required samples | queried artifact slice, seed, target and FK, model identity; whole selected grid | NPZ/YAML/JSON; 256 MiB/run | frozen builder/query command from ST06 dispatch |
| simulation cycle | all named stage boundaries and contract identities | any stage/metric/capture failure | full request/response chain, bounded sensor/TF and joint/scene trace, independent GT reference | JSON/NPZ/rosbag; ≤1 GiB/run | pinned ST09 replay entry and exact scene/model |

For simulation, the pre/post ring-buffer duration and capture overhead limit
are fixed from startup/non-scored rehearsal measurements before INTEGRATION is
dispatch-ready. Required sensor data must not be silently omitted just because
the old motion-only bag does not record cameras. The selected failed case plus
its upstream/downstream boundaries is the dump unit, not all module internals.
Total retained evidence budget before a new campaign is 4 GiB; if insufficient,
stop before the next run and revise the explicit capture budget, never discard
failure evidence to continue. No deletion of existing evidence in this plan.

Startup failure before the first scored case consumes no case/seed and does not
advance any cursor. Bound rehearsals to 20 minutes per attempt session, report
startup attempt/failure counts and timestamps. Queue behind any existing sim;
stop only the stack started for this task with `scripts/stop_sim.sh`, after
required failure data is durable. Dependency release occurs only after the
owner records a passing commit and complete evidence in the authoritative
discuss thread; the mailbox is only its current projection.
