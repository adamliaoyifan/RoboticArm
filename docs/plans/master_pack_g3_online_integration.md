# BASELINE G3: online two-box integration after Cursor G2

- parent: MASTER-PACK-20261013
- subtask: BASELINE
- generation: 3
- owner: eng / claude / glm-5.3 / claude-code
- base_revision: 56b2a41ffc00d4e2292fe235863716519ea71d80
- plan_revision: exact commit containing this file, pinned in dispatch

## Goal and Boundaries

Continue Cursor's accepted offline G2 into one online two-box functional witness.
G2 is complete, not reopened. This higher generation authorizes Gazebo integration,
not hardware motion, n=3, capacity research, seed-chain, policy training, or reliability
certification. No automatic merge into master is authorized.

| Task | Owner | Prerequisite | Acceptance | Deliverable |
|---|---|---|---|---|
| BASELINE G3 integration | claude/glm-5.3 | G2 pass and reviews closure | G3-0 through G3-4 below | clean commits, tests, complete evidence, lifecycle result |

Use `scripts/agent_start.sh` before work. Create branch
`agent/eng/master-pack-baseline-g3` in `/tmp/elfin-master-pack-baseline-g3` from the
exact base above. Coordinate only in the primary workspace's docs/agents.
Do not modify Cursor's worktree, copy the dirty master overlay, or change the shared
index. The base already includes G2 runtime fixes at 9ed7f96 and its test-only closure.

## Frozen Profile

ROS Humble; Gazebo headless, no RViz; ROS_DOMAIN_ID=7; deploy_update yaw-180 scene;
bbox_fill; strict Atlas identity; graph planner; two sequential inch20 boxes;
seed=0, discrete yaw, catalog size for spawning only. Use the base's committed
scene_tf_deploy_update, atlas_runtime_deploy_update and robot_poses.baseline_update
assets. These are not the older deploy/carryon fixtures. Do not pin slots 793/794.

Route search remains 60 s per placement, final certification 2 s; no multiplication
by box count. Keep the base's per-edge validation settings, interpolation checks,
collision thresholds, touch links, ACM, fraction, singularity, attachment, support,
map confirmation, execution/handoff, and controller gates unchanged. Export resolved
runtime parameters, not just launch defaults. Retain real clocks and sensor stamps.
GT is evaluation-only; production dimensions and occupancy must come from measured
or measurement-verified inputs under real_scenario.md and privilege_boundary.md.

## Procedure and Allowed Repairs

1. Audit base/assets/install and external weight/dependency hashes. Recover the
   existing G1 runner from `docs/status/evidence/master_pack_recovery/2026-10-13_g1/run.sh`
   as a reference, not an executable to run unchanged. Reuse production entry points.
   Prepare a versioned runner with this worktree/output, explicit graph/60 s settings,
   owner-aware teardown and unique run directories. Do not source unknown master
   overlays; any needed dependencies must have declared paths and hashes.
2. Build in a clean environment (`env -u PYTHONPATH`), compare installed loaded modules
   and assets to source. Run the original B0-B3 behavioral matrix below. Historical
   partial coverage is not pass. Capture wiring must be proven before motion.
3. Rehearse startup, observation, then one non-scored inch20 through release, retreat,
   return, measurement integration and confirmed map. Allowed repairs are deterministic
   launch/driver/installation/capture and measured-map handoff defects, with nearby
   behavioral regressions. Do not optimize planner algorithms or change the frozen
   perception/physics/safety policies to obtain a pass.
4. Any planning/execution failure with a complete boundary bundle ends this generation's
   motion attempts: analyze that bundle offline, report the first unmet gate and stop.
   Missing capture is an instrumentation defect, not a root cause; repair/test capture
   before another targeted rehearsal within the resource budget. No blind reruns.
5. Once rehearsal and prerequisites pass, commit all runtime/config/test changes,
   rebuild and verify install, dirty=0, then freeze. Run exactly one scored two-box
   sequence. Do not repair the evaluated artifact or repeat scoring after a failure.
6. Flush/validate evidence, stop only owned processes, write result and role note,
   and use `scripts/agent_complete.sh --outcome pass|blocked`. A blocked result names
   the first unmet gate and exact missing decision/input, not a generic backlog.

The runner must execute installed pack_eval_driver with:

```text
--frame-runtime --frame-peer pack_eval_driver --observe-pose pickup_observe_sim
--sequence-ids inch20 --max-boxes 2 --goto-timeout 60 --seed 0
--auth-max-attempts 8 --auth-max-elapsed-sec 8
--scene-tf-config <frozen worktree scene> --out <unique score directory>
```

Use max-boxes=1 only in rehearsal. Freeze a tested stop-on-first-failure behavior
before scoring. Preserve executable `run.sh` and `inspect.sh` with actual launch,
build and test commands; run `inspect.sh <run-directory>` to compute each gate from
artifacts. A full payload request, not a manually selected winning slot, enters Select.

## Acceptance Matrix

Original B0-B6 definitions are incorporated from sections 3-5 of
`master_pack_baseline_seed_chain_recovery.md` at commit
`72e775e611b40ebb754bae872cbf59c5846f6347`; this document controls conflicts.

| Gate | Workload and minimum samples | Required observations |
|---|---|---|
| G3-0 identity/readiness | every launch; >=10 s ready window | clean frozen runtime for scoring; source/install/assets equal; graph observed; exactly one clock publisher; camera and inference each >=10 valid increasing stamps; report rates; observed pose and TF/Atlas identities valid |
| G3-1 B0 | driver constructor, CLI propagation, install identity, pose identity, catalog resolution; one positive and relevant negative each | all behavior assertions pass; wrong identity/invalid configuration cannot enter motion |
| G3-1 B1 | one complete RGBD/TF/detection input twice; missing mask, hold-only, invalid-height negatives | same association/authorization on repeats; measured positive authorizes; each negative submits zero grasps |
| G3-1 B2 | valid box, zero-voxel/outside, add failure, pending map, confirmed consumption | nonzero valid update; invalid add not successful and no success revision; failed add not committed; pending causes zero next-placement requests; confirmed revision/content consumed |
| G3-1 B3 | same real validator state with obstacle inserted then removed | production validation COLLISION with named obstacle, then valid; same robot/payload/ACM; save both scenes/responses |
| G3-2 regression | base and final affected suites once, compare exact failing IDs | all focused new tests pass; zero new failures/errors; existing failures explicitly listed, not skipped to claim green |
| G3-3 online B4/B5 | one frozen scene, two consecutive inch20 boxes | packed=2, committed=2, no verdict failures; every submitted execution leg succeeds; release/retreat/return complete; first box measured-confirmed before second placement; revision advances on each real update, occupied columns>0, max_height>0; second request includes first obstacle with matching content/frame/token |
| G3-4 B6 | all attempts and failures | complete required boundary capture; failing boundary replay_possible=true or explicitly incomplete/non-pass; successful driver rc=0; owned sim/bridge remnants=0 |

Run G2's 24 focused tests plus affected packages and ST09 B/C/I; compare against
56b2a41 by test ID. Add map/perception/driver suites if touched. Existing 14 failures
are historical information, not a substitute for collecting the current baseline.
If any planning runtime file changes, the two G2 frozen-request cold-reset checks
must pass again on that clean runtime before online scoring; budget unchanged.

Pass is the AND of all gates. The unit is one complete box cycle: both of the two
observed units must pass. This is a functional witness only, with no population
success-rate target: an independent per-box probability p gives p^2 for the pair,
which cannot distinguish 95% from 99% reliability. Positive motion and obstacle/input
negative controls distinguish a functioning path from an always-refusing or unchecked
one. Missing stages or insufficient samples are not_evaluated, never 0/0 pass.

## Resource and Stop Limits

Rehearsal/startup repair: <=90 active minutes, excluding queue/build/pure unit tests.
Isolated MoveIt validation: <=30 active minutes, never concurrent with another stack.
Scored sequence: one, <=20 minutes. Pre-first-request startup failures consume no
scored case, but count toward rehearsal time. Each startup has a 120 s readiness
deadline. At limits, preserve evidence and stop; no implicit budget extension.
Follow sim-lifecycle.mdc and owner-aware scripts/stop_sim.sh. Queue for other agents;
a separate ROS domain is not permission to run a second Gazebo or contend with it.

## Evidence and Capture Health

Root: `docs/status/evidence/master_pack_recovery/2026-10-14_g3/`. Never overwrite G1/G2.
Always record commit/dirty/plan/base, actual loaded files and hashes, runtime params,
request IDs, stage timing, frame/stamp/generation/map identity and resource/PID ownership.

| Boundary | Trigger | Required payload/window | Inspection |
|---|---|---|---|
| startup | timeout/identity mismatch | complete startup logs, resolved params, publisher counts, stamp/rate trace | manifest vs frozen assets and G3-0 |
| perception/authorization | ambiguity/rejection/timeout | synchronized RGB/depth/info/TF/detections/masks/cloud and authorization; pre=max(2 s,2*measured pipeline latency), post=1 s | same-stamp join and B1 replay |
| Select/MoveIt/certifier | refusal/timeout/invalid target | complete Select request including untried slots, submission scene/ACM/model, exact q/constraints, certification requests/responses, winning or failed trajectory; entire call plus 2 s before/1 s after | existing G2 boundary decoder/replay plus read-only inspector |
| execution/release | mismatch/abort/missing truth/detach failure | submitted trajectory endpoints, actual joint/controller streams, vacuum/attachment state, release inputs/outputs; failed leg plus 2 s before/1 s after | stage/request join; independent GT only in reference directory |
| commit/next box | add/confirm/revision/content failure | add request/response, measured view, surface/token/content, next request and collision scene; full transition plus 2 s before/1 s after | B2/B5 revision and obstacle assertions |

Enable G2's Select/portal/full-certification capture paths on the live producer,
including ST09_SELECT_REQUEST_CAPTURE_DIR and ST09_PORTAL_CAPTURE_DIR. Verify actual
typed-path writes; legacy last_boundary alone is insufficient. Dedup by exact target
and scene semantics. Before scoring, exercise triggered capture and verify files
deserialize, IDs/hash match, required frame joins have zero missing items. Report
measured capture overhead; never subtract it from planning time to pass.
JSON/bin/images/compressed window bags: <=1 GiB/run, <=8 GiB total. Keep first complete
failure per signature, do not evict it. Flush before teardown; at space limit stop
with incomplete evidence rather than delete failures. Deliver gate_matrix.json,
test JUnit/ID differences, manifests, run.sh, inspect.sh, summary.json and verdict.md.

## TODO and Handoff

- [ ] Claim G3, create isolated branch, audit runtime/assets and G2 closure.
- [ ] Complete B0-B3 and capture-health; compare regression IDs.
- [ ] Rehearse one box through confirmed occupancy under unchanged graph policy.
- [ ] Commit/install freeze; run the single two-box functional gate.
- [ ] Preserve evidence, teardown, commit owned repairs, publish first unmet gate or pass.

Read the G2 plan, verdict and reviews closure, original baseline plan, architecture
real_scenario.md, privilege_boundary.md, placement.md, frame_contracts.md and debug
evidence/sim lifecycle rules. Seed-chain remains undispatched. Any future master merge
or n=3 task is a separate decision after this result, not extra work in G3.
