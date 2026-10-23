# Released return-observe readiness — 2026-10-23

## Scope and revisions

User-authorized goal: new placement/next-box entry requires placement planner; released unloaded return uses actual motion dependencies with collision, scene, map, vacuum and execution protection. Product `4c359ed4` implements independent startup-observed unloaded admission; `57ac1606` fixes bounded reference-evidence workload and readiness capture; `20d5206e` anchors occupancy before HOME and release confirmation after the set-pose response; `71143f9f` scopes verified occupancy commit to mapper/driver readiness. Plan `cf2dde14`, worktree `/tmp/elfin-master-pack-baseline-g3`, ROS Humble/Gazebo domain7, A-prime scene `603f9b6523e104f675cbeea77805e1ff`, seed0 inch20, prior off, select180s, unchanged attachment/envelope, XY release<=5mm and Z observation-only.

## Source-established behavior

- Driver and motion executor select unloaded dependency scope only for a complete unloaded route/certificate/leg claim. Selector named-pose admission uses the same scope. A separate gate observes relevant peers from startup, so no lease gap/restart is erased by constructing a scope late.
- Full immutable wire identity remains checked. Only placement-planner lease is renewable for a NEW request: a new lease epoch invalidates every older token. Missing/stale peer cannot admit a new placement request; incarnation/provenance changes still latch.
- Producer admission verifies actual detached vacuum freshness, exact certificate/leg/phase and map/scene before unloaded execution. A forged unloaded claim cannot authorize a carrying leg.
- Named return planning takes current post-commit map as an immutable value and records revision/digest in its request and response detail. Live changes remain visible to revalidation. Return does not compute placement or integrate a cargo view; next placement reads the current map normally.
- Release/verify trial and replay inputs are checkpointed before HOME and on exception. Driver captures the actual `_frame_runtime_handle`, bounded256-event own receive trace and stage durations; harness records an independent2048-event receive window. Capture-health gate precedes motion.
- GT reference mapper normalizes one unchanged hull per rasterization instead of reconstructing/hashing it per voxel. This changes reference-work cost, not online geometry, margins, assets, controller gains or lease limits.

## Behavioral verification

180 core tests and70 subtests pass (`offline_final_core.log`);8 mapper tests pass separately (`offline_final_mapper.log`). Combined pytest collection encounters duplicate-basename import collisions from launch_testing, so suites run in separate processes; failed collection logs are retained. Includes missing/unrelated placement expiry/restart vs relevant-peer expiry/restart/identity/type, old/new token lease epoch, NodeRuntime effect boundary, real typed client scope selection, unloaded vacuum rejection, named-route/map/scene checks, release XY gate, HOME exception durability, reference hull equivalence at every voxel center and64 scene-pose cases. Four-package build passes. Twelve installed module/script hashes match (`install_identity_03.json`).

Startup-only rehearsal `startup_01` passed without motion. Instrumented no-motion `readiness_rehearsal_01` passed3 GT evidence repetitions, each0.364–0.366s, required peers' independent receive gaps at most~0.251s. These measurements cover this rehearsal only, not a success-rate improvement. `live_01/gt_profile.txt` is an instrumented cProfile result (not timing-comparable to clean rehearsal); it identifies68400 repeated geometry reconstructions in two reference mappers.

## Frozen closed-loop attempts

| Attempt | Revision | Result | First divergence |
|---|---|---|---|
| live_01 | 4c359ed4 | n3:1 attempted,0 packed,2 not attempted;379.8s | Required scene/motion/vacuum peers expire (~2.04–2.06s) during post-verify evidence, before named-pose return execution.8 certified route legs execute; release XY=0 and verify pass. |
| live_02 | 57ac1606 | n3:2 attempted,1 packed,1 not attempted;849.7s | First HOME succeeds; second release XY11.234mm read from a pre-command sample. Raw capture was not exported: causality evidence incomplete. Also exposes pre-box HOME map, corrected before next scoring. |
| live_03 | 71143f9f | 0 attempted; startup only, no scored unit | URDF GetParameters response absent90s; clock/camera healthy. |
| live_04 | ddc35211 | **n3:3 attempted,3 packed,0 skipped;1248.6s;exit0** | **PASS**: three release/verify/HOME/commit cycles; MAX_BOXES, goto_failed_count0, clean executor shutdown and owned teardown. |

`live_01` has91 preserved files212469232 bytes, primary copy hash-verified before follow-up. Its motion replay manifest is complete; readiness replay is explicitly incomplete because the initial handle lookup omitted the scope snapshot (`live_01/readiness_failure_manifest.json`). Do not conflate these capture tiers. `trial.json` retains exact release/read evidence, verify inputs and HOME exception, repairing the prior lost-trial failure. Last prior baseline `8afbec19` stopped on placement-planner lease; live_01 exposes a different required-peer failure, so this task does not attribute every historical anomaly to placement readiness.

## Interpretation and limits

Functional gate is AND over3 measured boxes:3/3 release, verify, return and commit required; no population reliability estimate. Counts are reported per immutable attempt, without merging failures into a successful streak. Raw release90s/8MiB and plugin600s limits do not establish whole-suite physical collision coverage. Empty contact output is not proof of no collision. Required-peer loss still refuses; no critical lease was relaxed. A frozen map value does not ignore new physical occupancy events.

## Follow-up contracts and current validation

Verified occupancy is committed before HOME. Return response exposes immutable map revision/digest/scene; suite finalization consumes the existing receipt without a second commit. Occupancy add client/server gate only mapper/driver; placement planning and next-box entry retain full readiness. A failed HOME does not remove a physically released obstacle.

Release confirmation requires both pose simulation stamp and receive-monotonic time after the set-pose response. During the unchanged2s read budget, transient nonmatching samples are recorded and the reader awaits XY<=5mm; persistent mismatch still refuses, Z remains observational. The launcher now explicitly exports release raw capture location. No-motion rehearsal02 captured1702382bytes and decoded10325 poses; three evidence builds remained READY.

Startup live_03 failure and subsequent no-motion rehearsal03 controller-plugin GetParameters hang are preserved separately. A standalone domain7 cross-process GetParameters probe returned the same23248-byte URDF three times. This does not isolate the full-stack transport failure. Preflight now retries its own request after5s within the original90s bound and records every attempt; it does not bypass the plugin/controller gain gate. Rehearsal04 passed. It did not exercise the new preflight retry branch; no claim of eliminating plugin startup flakiness. Frozen live_04 follows below.

## Final measured result

Final product `ddc35211122d175dc20e48ab24cf9425882291a2` (scope/map/release fixes through `71143f9f`, startup capture-only retry `ddc35211`). Frozen clean product except pre-existing untracked `pydeps-frozen`. live_04 ran13:38:56–13:59:47; stack teardown13:59:48, no residual Gazebo/bridge processes. `live_04/driver_out/suite.json`, `ledger.jsonl`, per-case trials/replays and `acceptance_audit.json` support all3 functional units passing.

| Box seq | Verified map commit → return snapshot | Release XY | Post-command sample | HOME |
|---|---|---|---|---|
|0|2 → 2|0m|yes|9.64s, complete|
|1|3 → 3|0m|yes; rejects1 pre-command stamp|8.90s, complete|
|2|4 → 4|0m|yes; rejects1 pre-command stamp|8.88s, complete|

Every return includes a nonempty frozen snapshot digest and scene identity; admission revision matches the already accepted occupancy commit. Exactly3 committed suite ledger entries, one per box. All three verification verdicts are `ok`,8 certified route legs per box succeeded, no failed return or executor runtime failure. Offline `audit_live.py` reads durable trial/ledger only and computes the AND3 result; rerunnable with `python3 <root>/audit_live.py <root>/live_04`. This is a functional n3 result on one seed/profile, not a population success-rate estimate.

Live_04 did NOT induce placement-peer loss: scope isolation, critical-peer loss/restart refusal and full readiness on next placement are demonstrated by behavioral offline tests. Old-pose rejection was directly exercised in boxes1 and2. No change to envelope, physical assets, controller gains, routing/search budgets or XY threshold. Each release raw poses stream reaches its8MiB cap, contact streams are empty; neither establishes whole-route physical collision coverage. A later read-only controller probe ran after teardown and therefore records `not_evaluated`, not motion evidence.
