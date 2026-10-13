# ST09 generation 3: verified startup, tooling and integration handoff

- owner: eng/codex/gpt-6
- base_revision: 1795a69
- generation: 3
- status: implementation required

## Scope

Use an isolated worktree and fresh install from accepted ST08 evidence commit
1795a69. Coordination remains in the primary workspace. Preserve earlier
lifecycle events. No scored integration or hardware motion is authorized;
exclusive non-scored Gazebo rehearsal and fake-controller deployment tests are.

Resolve sim, hardware-plan-only and verification settings through one resolver,
with CLI precedence and rejection of unknown contract-critical keys. Resolve
endpoint types, scene/calibration, expanded URDF/SRDF hashes, tool, ordered
joints, hull and certified atlas. Missing identities, including URDF, reject.
Consumers keep motion, attachment and scene mutation disabled until running
peers match. Process reports include loaded modules/package prefixes/source
hashes/generated message and type-support paths/hashes/resolved settings.
Restart or identity change revokes readiness; carrying preserves vacuum/recovery.

Switch the full typed v2 chain: waypoints, selection/probe, execution, scene,
vacuum, hardware and all six F22 drivers. Preserve times, velocities, measured
attachment transforms and alternate-goal FK checks. Probe-on requires success.
Keep v1 layouts unchanged; v1 mutation cannot bypass profile admission.

Version dump envelopes explicitly: schema/contract, frame/reference/units,
acquisition/request, scene/map/hull, body dimensions, transform and provenance.
Map z is container-origin relative with explicit floor; preserve unknowns.
Keep measured and GT records separate. Legacy adapters require a named source
format and documented frame/datum; ambiguity and unknown schemas reject.
Prove F01 scene/TF publication and consumer validation, runtime F21-F23, and
F24 absence from installed executables and tested graphs. Refresh stale proofs.

## Verification

Implement scripts/frame_migration_st09_verify.sh with offline, graph, rehearsal
and all modes; explicit workspace/install/output/domain, recorded commands and
refusal to overwrite output. Each applicable adapter, dump/replay, marker and
scene/TF boundary independently runs the established 64-case matrix. Compare
world centers/eight corners/normals/grasp transforms with independent oracles:
1e-9 pure and 1e-6 serialized, exact identities/nanoseconds. Cover q/-q,
distinct tool branches, tilt rejection, changed floors, stacks, unknown cells.

Every applicable common negative plus unknown dump schema, conflicting legacy
datum, mixed types, missing peer, stale atlas, missing URDF, duplicate provider,
wrong overlay and restart must yield its documented code and zero added
planning/execution/attachment/map effects. Save expected/observed codes/counters.
Behavioral mutations (wrong frame, position-only rotation, dropped velocities/
times, legacy bypass) must fail applicable gates; source assertions do not count.

Graph: production sim/deploy nodes with fake external dependencies/controllers,
real services/actions/generated messages, no Gazebo/hardware. Three healthy
startups per profile, once per fault; 180 s startup/90 s teardown. Save graph,
process identities, gate transitions and counters. Required suites: tools unittest
(test_frame_migration_tools.py), ST08 execution, contracts, affected regressions.
Base/candidate use identical recorded environments with collected node IDs.
Packing exception requires identical failing node/reason on both, unsuppressed.
100% deterministic correctness, no retry selection. Missing samples are
not_evaluated and block release. Three startups show observed repeatability only.

## Rehearsal and frozen integration contract

Use existing persistent startup probe and predecessor-relative deadlines,
exclusive Gazebo, domain 7, GUI/RViz false; max 20 minutes/session. Flush failed
attempt evidence before teardown. Startup failures consume no case/seed/cursor.
Explicit runner rehearsal mode has a separate namespace and no scoring writes.
After readiness run one diagnostic carryon then standard cycle; measure latency
and capture coverage. Repairs require a new immutable candidate and fresh record.

Freeze executable extractor/config: join observation -> slot -> selected/probed
path -> execution -> attach/release -> measured verification/integration -> map
update -> next placement. Require return/recovery, no lost payload, successful
measured integration, >=1 sensor-occupied cell, zero floor-through candidates,
second placement using updated map. Both boxes require GT-labelled XY <=0.04 m,
absolute Z <=0.03 m, drift <=0.02 m, absolute roll/pitch <=2 degrees, center in
hull, GT sensor-footprint coverage >=0.40 with positive denominator. Production
map admission depends on measured contracts. Prepare pre-pick/pre-place faults:
zero new motion/commit and preserved carrying recovery. Offline extractor tests
reject missing events/metrics, stale revisions, threshold failures and truncation.
Freeze runner/extractor revision, resolved profile/scene/model/atlas/URDF/SRDF/
message hashes, release/drift intervals, ordering and replay command. Hash changes
invalidate handoff. Later both-box pass is finite coverage, no population claim.

## Evidence and release

Evidence: docs/status/evidence/frame_migration/ST09-<run>/. T0 includes revisions,
dirty state, environment/commands/timing/inventory/hashes/teardown; T1 every
boundary identity/result/reason/latency/effect count.

| Boundary | Failure payload | Unit |
|---|---|---|
| startup/profile | settings, process identities, graph, gates, logs | full attempt |
| tool/ROS | exact input/output/oracle, serialized request, counters | full request |
| perception/map | RGB/depth/info/cloud/TF, measured observation, integration and revision | bounded boundary window |
| planning/execution | candidates, TF/joints/scene, trajectory, controller, attachment | full requests plus bounded trace |

Live pre-window=ceil(2*maximum observed upstream latency), post-window=ceil(maximum
observed downstream completion latency); require positive samples, freeze seconds.
Injected failure must capture and replay rejection/counters. Missing boundaries
block release. Require one clock publisher, all streams, joins/checksums. Three
paired replays measure median throughput degradation <=10%. Lossless replay
inputs, <=1 GiB/live run and <=4 GiB total; stop before budget, never delete failures.

Release only with clean candidate build, complete passing evidence, F01/F21-F23
verified, F24 isolated, both checkers exit 0 without stale/pending proofs. Record
implementation commit, proof hashes, role note and lifecycle result. INTEGRATION
is a separate release against the exact candidate and frozen configuration.
