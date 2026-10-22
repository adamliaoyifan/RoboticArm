# Attachment-owned conservative geometry G4

- Owner: eng/codex/gpt-6
- Base: 702e3c44b5e1a74c9e47b66fa9b77b14ccde9291
- Lineage: PHASE2-20261022 / PAYLOAD-ENVELOPE / generation 4
- Authorization: user requested implementation, basic verification and closed-loop metrics; stop after two blocked closed-loop attempts and discuss preserved evidence.

## Contract

Keep measured dimensions and measured box/tool transform immutable. Attachment owns a single conservative geometry constructor and configuration. Its frozen output includes dimensions, transform/center offset, nominal input digest, policy and geometry digest. All placement-capacity, carried-search, scene and certification consumers use that output; nominal contact/target semantics remain explicit. Planning must not independently expand geometry, load tolerance policy, or recompute an envelope. Restore pre-envelope planning algorithms where possible; retain generic consumption of conservative geometry. Remove diagnostic slot-envelope bypass and late special-case filtering once upstream conservative placement is wired. No blind whole-commit revert.

Retain the agreed diagnostic bounds: total W/D growth 10%, H growth 10% away from suction plus the existing 5mm positional bound, yaw +/-5deg about the suction contact normal, zero XY positional bound. Roll/pitch are exact under the existing vertical-box/tool-down contract. Bounds are diagnostic, not calibrated coverage probabilities. Eccentric attachment must survive; absent qualified attachment is a named refusal. Before actual attachment, any preview must be explicitly identified and regenerated from the measured attachment before executable planning. A conservative center is not the nominal box center: transform candidate poses back explicitly, without mutating input dimensions.

## Implementation and verification

1. Centralize immutable Attachment geometry and policy; introduce explicit transport for frozen conservative geometry where the existing messages carry only nominal size.
2. Wire attachment output before placement candidate generation and through selector/MoveIt; retain nominal grasp targets and measured map commit. Remove planning-local policy and envelope-specific fallback/diagnostic additions.
3. Behavioural tests: nominal input unchanged; repeated consumers cannot inflate twice; centered/eccentric and both box-Z signs; continuous yaw extrema including interior critical angles; contact face unchanged; invalid/missing provenance refusal; placement fits conservative footprint and translates target centers correctly; attachment scene/search/certification geometry identity matches. Test existing attachment, placement and route regression suites. All selected tests must pass; run no sim until failures resolved.
4. Plain colcon install build for changed packages and dependents (no symlink install); verify source/install identity. Freeze implementation commit before evaluated cases, record dirty state (only untracked pydeps-frozen and evidence allowed).

## Closed-loop procedure and metrics

Adapt scripts/g3/n3_functional_run.sh to the Attachment-owned policy and transport, removing obsolete planning envelope switches. Retain inch20, seed0, A-prime scene, gain0.8, ROS domain7, 180s selector budget, prior off, 3 boxes per suite. Use exclusive headless Gazebo. Pre-scoring startup rehearsal may repair deterministic wiring; no simulated box attempt is silently discarded. Run at most TWO functional suites; after two blocked attempts preserve evidence and stop, without automatically reducing bounds or changing search algorithms. Each attempted suite is reported separately; all 3 verified/committed/map-confirmed boxes with MAX_BOXES and exit0 is a functional suite pass. This AND-over-3 gate is a smoke check, not a statistically powered reliability claim; per-box target is 100% within observed cases, population reliability not evaluated.

Report attempted/picked/packed/verified/committed boxes, completion and failure stages/reasons, wall and stage latencies, candidate count/tried/reject histogram, route builds/certified routes/carry submissions, attachment geometry identity, startup failure count, teardown status. Missing samples are not_evaluated with sample count0, never a latency of zero interpreted as success.

## Capture matrix

| Boundary | T1 | Trigger and T2 payload | Window/budget | Inspection |
|---|---|---|---|---|
| detection/attachment | request IDs, dimensions, transforms, policy/digest | refusal/failure: full detection, measured attachment, physical reference separate | failing case; 64MiB | JSON + attachment tests |
| placement/selector | IDs, geometry identity, latency, candidate counts | failure: original request/response CDR+JSON, candidates/rejections, map/hull, TF/joints/scene/ACM, installed URDF | failing request; 256MiB | saved-boundary replay |
| execution/commit | stage timeline, planned/actual joint trace, vacuum, verify/commit result | timeout/refusal: trajectory/controller output and bounded joint/plugin ring | existing capture window recorded explicitly in manifest; 512MiB | driver dump and lifecycle report |

Every run stores manifest (revision/dirty/config/seed/environment/commands/times), capture inventory, missing artifacts and replay status. Preserve the existing failure-window capture before teardown; max1GiB/run, 2GiB total, retain both attempts, no overwrites. Capture-health requires joinable requests, scene and geometry provenance; incomplete capture must be repaired before the second attempt. Record capture overhead as unmeasured unless directly measured, not a performance comparison. Release owned sim immediately after flushing artifacts; never stop another agent stack. Durable evidence under docs/status/evidence/occ_owner_b1/attachment_geometry_g4_20261022/; role notes and lifecycle only in primary docs/agents/.
