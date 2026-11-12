# Released cargo observation, provenance and collision representation

- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- base_revision: 7b697e00
- owner: codex/gpt-6
- status: implementation checkpoint

## User decisions and boundary

Keep the existing0.05m release gap. Add the missing observation association
module this round. Use conditional unique matching under the user's premise
that other cargo is stationary: frozen measured pickup evidence + actual fixed
attachment + actual release evidence + measured before/after geometry. An
occluded, contradictory or non-unique match preserves original O. A task label,
planned box pose, expected AABB, or single nearby mask does not prove identity.
No torque/current signal is available by default; this policy cannot establish
immediate detection of every physical collision or unseen cargo displacement.

## Causal input

The preserved native_cycle_02 retreat is refused at sample0 by the released
box's coarse O collision representation. Four controlled native ablations
locate this mechanism; production O defaults to0.01m versus fixture0.1m.
Evidence: ST-4-INTEGRATION/g2/release_collision_root_04. Never solve this by
rounding raw returns, reducing padding, increasing the release gap, ignoring
unknown space, or clearing a planned box-shaped region.

## Execution order

| Unit | Owner | Dependencies | Bounded result | Gate |
|---|---|---|---|---|
| ST-4-OBS-JOIN gen1 | codex/gpt-6 | none | Exact-acquisition RGB/depth/instance-mask/calibration/TF capture and point-to-pixel partition, ROS service, immutable binding | Acceptance below; no map changes |
| ST-4-OBS-MATCH gen1 | codex/gpt-6 | OBS-JOIN | Conditional unique measured registration; frozen/actual attachment/release and before/after observations bound | Must freeze measured uncertainty/observability policy before dispatch |
| ST-4-O-PROVENANCE gen1 | codex/gpt-6 | OBS-JOIN | O-owner receipt binds all contributing hits to actual integrated source observations and point partition | Exact native preprocessing/key parity; no missing/unclassified contributors |
| ST-4-OWN-REPRESENTATION gen1 | codex/gpt-6 | OBS-MATCH,O-PROVENANCE | Read-only private-scene substitution for proven exclusively-own occupied voxels; measured conservative own geometry retains existing collision padding | All other/mixed/unclassified/unknown geometry preserved; current-source binding |
| ST-4-INTEGRATION gen3 | codex/gpt-6 | prior gen2 prerequisites + units above | Same original6 timed legs and actual event barriers, current O, source-bound own representation and native suffix admission | Supersede gen2 only when dispatchable; preserve its entire matrix and deploy witness |

Only OBS-JOIN is dispatch-ready at this checkpoint. Gen2 integration remains
open and unchanged. Commit each bounded unit with its source/hash manifest;
later units require exact plan revision and acceptance before dispatch. This
table is not a claim that any later unit already exists.

## Observation acquisition contract

Producer is a focused perception adapter; geometry/math is ROS-free. Inputs
are actual colour-aligned RGB, depth, instance mask and camera intrinsics from
the existing preprocessing/segmenter topics. All four must have the exact
same positive integer acquisition stamp and optical frame, identical image
dimensions, supported lossless encodings and valid calibration. Resolve TF
at that acquisition stamp; no latest transform or task instance_id fallback.
No depth completion, temporal interpolation, mask dilation or box template.
LiDAR without equivalent exact image association remains unclassified.

Deproject valid sampled depth pixels once, preserving each source flat pixel
index. Keep background label0 and every other per-frame instance label; do
not drop inconvenient returns. This label is explicitly frame-local, never
cross-release physical identity. The exact produced point order and Float32
bytes must be the later O integration input: independently reconstructed or
nearest-neighbour clouds do not qualify. Acquisition hash binds full raw
inputs, calibration, TF, sampling stride and output partition. Retain source
images and serialized boundary request/response for replay.

The service requests an exact stamp, refuses missing/stale/evicted/malformed
or conflicting data, and returns the complete frozen measurement and partition.
It has no robot command, vacuum, map-write or scene-write port. Resource limits
are explicit caller/configured buffer sizes and age, with no geometric quality
threshold hidden in resource constants. Production wiring remains opt-in.

## Downstream obligations before replacement

1. Actual measured registration must locate visible geometry using the source
   observation; expected release pose is a search/gating prior, never an
   observed pose. Enumerate all compatible visible instances and require one
   unique candidate. Record residuals, support, fit bounds and competing
   candidates. Use observably obtained uncertainty/calibration, not an invented
   perfect dimension/pose interval or guessed tolerance tuned to the failed case.
2. O mutation authority must bind provenance to actual accepted integration,
   config/epoch/hull/clock and observed_revision. An occupied voxel may be
   substituted only when all relevant hit contributors are available and
   attributable to this same associated cargo. A matching latest hit is
   insufficient. Missing history, an unlabelled sensor, mixed returns, reset,
   conflict, stale receipt or intervening update yields no substitution.
3. Preserve other obstacles and unknowns. Bound replacement geometry using
   frozen measured shape and post-release measured pose/uncertainty; preserve
   the existing collision padding. Do not reduce geometry to just the visible
   points, which omit hidden surfaces. No O mutation or confirmed-free claim.
4. Before an original retreat controller goal, capture fresh O and validate
   the resulting full private scene through native FCL on original samples.
   Source changes invalidate the receipt. Native refusal still blocks motion.
   Displacement/fault invalidates old assumptions; preserve sealed G/vacuum
   while carrying, never auto-reattach after release or resume old legs.
5. Check wrist-view visibility before expecting success: the acquisition itself
   cannot move the wrist through an unvalidated collision. Insufficient safe
   view means representation remains coarse and the operation refuses; a
   diagnostic geometric proof is not a physical sensor observation.

## ST-4-OBS-DATA-AUDIT generation1 (dispatch-complete)

User supplied `/home/adamliao/work/robotarm_bags` on2026-11-12: raw data
exists, no measured box pose/detection error report. Add this read-only
prerequisite before OBS-MATCH. Owner codex/gpt-6, base c1ef631125cac8e698eff3c37c618a33bd9f37fb,
dependencies none. No runtime threshold, sensor playback, robot action or O
replacement is authorized by this assessment. The historical September bags
are not automatically representative of the current deployment calibration.

Workload: inventory all10 discovered metadata.yaml files once; deeply inspect
`20260916_212503_floor_box_pick/bags/pick/pick_0.mcap` once, with an optional
second indexed pass for at most3 evenly ordered exact-join source examples.
Report every selected source topic's actual header acquisition stamps, frames,
counts, duplicates, header/log lag distribution and exact RGB/depth/mask/info
join count. Report TF edges/static extrinsics, joint-difference distributions,
actual event messages and available reference/scene labels. Sampling selection
is explicit; examples do not establish worst-case coverage. Resource budget:
<=128MiB evidence, <=15min wall clock; failure stops capture with a durable
manifest. Raw MCAPs stay unchanged; record metadata hashes, MCAP size/mtime and
per-replayed-source CDR/schema hashes instead of rereading6.5GB for a full hash.

Acceptance is a finite functional AND, once per deterministic test: synthetic
MCAP counts agree with written messages; exact same-stamp/frame joining accepts;
one-nanosecond or frame mismatch does not join; duplicate/conflicting inputs
remain visible rather than silently overwritten; CDR acquisition-stamp parsing
honours endian/layout and refuses malformed records. Real-run inventory must
cover10/10 current metadata files; inspected topic counts must match metadata
for each fully read selected topic, or explicitly fail. A valid empty join is
a measured result, not a test failure. At least one decoded real image/mask
example and acquisition/source manifest are required; if no join exists,
capture one unmatched source boundary. Native MCAP reader/schema decoding only,
no Humble playback or generated-type substitution for newer recorded schemas.

All required finite cases must pass; this does not estimate physical per-box
reliability. Absolute pose/size error, false physical association rate, static
scene repeatability and current-deployment transfer are `not_evaluated` unless
independent reference/scene/motion evidence actually supports them. No P95
error inferred from acceptance targets or multiframe self-agreement. Next
measurement plan must distinguish repeatability, cross-view consistency and
independent accuracy; freeze labels, independent units, holdout and minimum
sample/power before scoring a matching policy.

Command: `PYTHONPATH=src/luggage_perception python3
src/luggage_perception/scripts/audit_cargo_bags.py --root
/home/adamliao/work/robotarm_bags --output <evidence>/run_01`; focused pytest
command and exact dirty/source hashes recorded in manifest. Installed runtime
is unaffected; no ROS graph is created. Preserve failures before correction.

| Boundary | T1 trace | Trigger | T2 payload | Window | Bound/format | Replay |
|---|---|---|---|---|---|---|
| Metadata/MCAP ingest | paths/counts/types/storage/bytes/mtime | missing/count mismatch/decode error | exact metadata/schema/offending CDR + error | single record | JSON/CDR <=16MiB | audit same immutable source or retained record |
| Acquisition join | topic/stamp/frame/lag/count/conflicts | malformed/conflicting source or zero join | contributing source headers + one complete example | exact requested acquisition | JSON/CDR/PNG <=64MiB | pure join test and native schema decoder |
| Coverage assessment | available event/reference/view evidence | unsupported accuracy/identity inference | labels/events and metric availability reasons | bounded run summary | JSON/Markdown <=16MiB | inspect preserved summary; no physical claim |

Capture health verifies all selected CDRs/schemas and output JSON/images are
decodable and source identities retained; record timing as diagnostic overhead.
Failure before scoring consumes no physical experiment. This audit has no
scored matching campaign and cannot enable substitution.

## ST-4-O-RESOLUTION generation1 (diagnostic dispatch-complete)

User requests checking whether2cm O voxels remove the10cm-fixture refusal.
Owner codex/gpt-6, base415506aa, dependencies none. Preserve original native
cycle02 source inputs and original494-sample timed retreat; no replanning,
hardware, controller, Gazebo, production config change or obstacle exemptions.
Read native_cycle_02/stack/00_observer_integrate_observation_request.cdr plus
the38 original camera requests through the captured snapshot acquisition
stamp19727797595ns. Re-integrate those exact39 CDR observations into an isolated
native O owner at0.10m and0.02m. Same geometry/filter/probability configuration,
acquisition clock/stamps, input transforms and ordering. Preserve unknowns.

Use actual ObserverSnapshotPort and build_private_snapshot to build the private
collision scenes. Keep original fine released-box geometry, robot, all remaining
world geometry and trajectory fields. Three finite diagnostic variants, once
each: rebuilt10cm with original0.05m padding; rebuilt2cm with original0.05m
padding; rebuilt2cm with zero padding as a diagnostic ablation only. Last is
not a proposed runtime change.2cm verdict is unknown before measurement; report
all494 valid or first invalid sample and actual pairs, never tune to force pass.

Functional gate is AND of:39/39 native INTEGRATED receipts per resolution;
original10cm reconstructed states identical to preserved original ReadCells;
original10cm native sample0 COLLISION/contact set reproduced; every variant
has native response result0 and complete sample-count/contacts; original leg
message fields unchanged; output geometry actually reflects owner resolution
and specified padding; complete replay capture and clean native teardown.
This gate tests one deterministic case and has no success-rate or real-wrist
coverage claim.2cm collision is a legitimate experimental result. Surface/cell
capture must be complete and same-source bound. Missing native readiness/source
or parity yields incomplete evidence, not a physical result.

Environment source /tmp/joint_operation_o_env.sh; run evidence replay.py on
isolated available domain184 with matching ROS_LOCALHOST_ONLY=1. No simulation.
Before run inspect matching native processes, never stop another worker.
Readiness bounded10s per native service; RPC10s, whole diagnostic <=15min.
Source/binary/config/parameter/trajectory hashes, revision/dirty state and exact
commands are T0. Record observed_revision and active/free/occupied/unknown
counts, private geometry bounds, first invalid sample/pairs and durations T1.
Evidence under ST-4-INTEGRATION/g2/release_resolution_01 (new suffix on retry),
<=128MiB/run, <=256MiB total; preserve every failed bundle before fixes.

| Boundary | T1 | Failure trigger | T2 | Window | Budget | Replay |
|---|---|---|---|---|---|---|
| Native O integration | source hash/stamp/status/revision | RPC/status/parity failure | exact requests/replies/config/clock | fixed39-frame history | CDR/JSON <=64MiB | same replay with preserved inputs |
| O scene conversion | states/counts/resolution/padding | mismatch/missing/invalid scene | snapshot pages + full private scene | one snapshot per resolution | CDR/JSON <=32MiB | same converter with retained snapshot |
| Native retreat validation | original leg hash/verdict/sample/pairs | refusal or unexpected error | full route request/response and native log | original494 samples | CDR/JSON <=32MiB | same native validation request |

Capture-health gate rehashes records, verifies native CDR decodes, lease release
and process exit. A diagnostic no-padding result has no production authority.

## ST-4-OBS-JOIN acceptance (dispatch-complete)

Environment: source-first Python3/ROS Humble, isolated generated interfaces
and perception install in /tmp, no Gazebo, controller, vacuum or O writes.
Exact base7b697e00 plus plan commit, record all unrelated dirty paths/count and
hash every tested changed source. No unrelated staging. Commands:

```bash
source /tmp/joint_operation_o_env.sh
python3 -m pytest -q src/luggage_perception/test/test_cargo_observation_partition.py src/luggage_perception/test/test_cargo_observation_service.py --junitxml=<run>/junit.xml
colcon build --packages-select luggage_msgs luggage_perception --build-base /tmp/cargo_observation_build --install-base /tmp/cargo_observation_install --log-base /tmp/cargo_observation_log
```

Build procedure may use an equivalent dependency-isolated package selection
when the workspace contains unrelated COLCON_IGNORE changes; record exact
command and preserve generated bindings/source parity and serialization test.

Each named deterministic case once, functional AND. This gate encodes100%
correct verdicts for these finite cases, not a stochastic reliability rate or
latency/capacity gain. Positive matrix: two visible labels plus background;
missing/NaN depth excluded while original pixel indices remain correct;
explicit stride; valid32FC1 and16UC1 encodings; non-identity acquisition TF;
lossless ROS round trip; source/installed import parity; exact-stamp bounded
buffer capture. Expected points/labels/indices must equal independent hand
deprojection, not a call back into the implementation.

Negative matrix: different stamp/frame/dimensions, unsupported encoding,
malformed/nonfinite/skew/distorted intrinsics, bad TF, zero stamp, all missing
depth, negative/out-of-range labels, bad stride, oversized input, conflicting
same-source same-stamp inputs, absent exact stamp, expired/evicted source.
Every case must refuse without an accepted observation or external effects.
Mutation case: changing caller image/mask/TF after capture must leave issued
bytes/hash unchanged. Hash sensitivity: RGB/depth/mask/calibration/TF/stride
each changes the binding. No cross-frame/task-label identity assertion.

Evidence: docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-JOIN/run_01/
(unique run suffix on repeats), <=64MiB per run, <=128MiB total this unit.
T0 includes revision/dirty/source hashes/env/commands/start/end/expected vs
actual/capture inventory/teardown. T1 acquisition IDs, shapes, point/label
counts, rejection reason and service result. T2 on any unexpected error:
freeze the single exact failing frame and request/result, all source arrays,
camera/TF/policy and traceback before correction/rerun. No cleanup without
verification. Missing inputs or a missing installed/service witness is
not_evaluated. Timing is diagnostic only; do not claim a speed improvement.

| Boundary | T1 | Trigger | T2 payload | Window | Bound/format | Replay |
|---|---|---|---|---|---|---|
| Sensor join | source IDs/stamp/frame/shapes | unexpected refusal or join/conflict | full input arrays/headers/calibration | one requested frame | compressed NPZ + JSON <=16MiB/case | pytest exact node ID with retained inputs |
| Deprojection/partition | point count/label counts/binding | wrong output or mutation | input + expected/actual partition | same frame | NPZ/JSON <=16MiB/case | same deterministic case |
| ROS service | request/reason/binding | mismatch/exception/stale acceptance | complete CDR and source arrays | same request | CDR + manifest <=16MiB/case | service behavioral test + serialized replay |

Capture health must verify all required files are present and decodable before
reporting a gate; hashing/capture overhead is reported separately and is not a
production performance score. Complete failure bundles remain durable.
