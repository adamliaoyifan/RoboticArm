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
