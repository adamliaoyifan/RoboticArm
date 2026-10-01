# Frame migration INTEGRATION generation 2: execution plan

Parent: `FRAME-MIGRATION-20260929`. Obligations unchanged from
`docs/plans/frame_migration_stages.md` (§ INTEGRATION); this document makes
them decision-complete for dispatch against the ST09 generation-5 candidate.
Revision 2 (2026-10-01 late): rewritten to match the g5 runtime strategy that
emerged from rehearsals 32-50 — release-settle with dynamic respawn,
seeded-hop replanning inside Cartesian legs, the open-loop FJT refusal rule,
and the cargo-view set that fixes VIEW_EMPTY. Supersedes the revision-1
draft (`c52bfff`) as the binding text once reviews binds this commit.

- owner: `eng / claude / glm-5.3` (user directive 2026-10-01; supersedes the
  generation-1 projection of `eng / codex / gpt-6` on the same subtask)
- subtask: `INTEGRATION`, depends on `ST09`
- candidate_revision: the exact ST09 generation-5 passing commit accepted by
  reviews, recorded in the generation-2 dispatch row `revision` field. Denoted
  `<CANDIDATE>` below. No other revision may run the gates.
- status: proposed, awaiting reviews bind (generation 2, `dispatch_ready: yes`)

## Frozen inputs (validated before any gate runs)

All frozen values come from the ST09 generation-5 P5/P6 release evidence
(`docs/status/evidence/frame_migration/ST09-g5-*/`), produced by
`scripts/frame_migration_st09_freeze.py`:

1. Handoff manifest: scene, runner, extractors, resolved profile, model,
   atlas, URDF, SRDF, message definitions, replay commands, drift/release
   sample intervals, with content hashes. Hash mismatch invalidates the run.
2. Expected cycle-event table for the two-box sequence in **three
   categories** (the extractor enforces them differently):
   - **required**: per box — observation, slot, selected_path, probe,
     execution, attach, release, `release_settle` (settled, dynamic respawn),
     measured_verification (XY/Z within frozen thresholds), place retreat,
     place_exit, return, `return_complete`; plus per-view integrate events.
   - **conditionally allowed**: seeded-hop replans and OMPL fallbacks inside
     any Cartesian leg (present only when the excursion/wrist gate trips);
     their presence must carry the gate reason string, their absence must
     not fail an otherwise healthy leg.
   - **forbidden**: `VIEW_EMPTY`, `EXIT_FAILED_*`, `PLACE_VERIFY_*`,
     `PLACE_PATH_EXCURSION` that terminates a leg, `outside_hull`,
     unexcused MoveIt error codes.
3. Cargo-view set: the frozen observation poses used for measured
   integration (whatever g5 froze — lift, portal aim, or a multi-view set).
   Each frozen view pose MUST ship with: an IK-prevalidated solution at the
   frozen configuration (no `NO_IK_SOLUTION`/`-31` at freeze time), a
   rendered visibility pre-check showing nonzero YOLO/mask cargo points
   from that pose, and the expected `n_sensor_occupied`/coverage thresholds
   the live extractor applies.
4. Map-revision chain: the initial `map_revision`, the required increment
   after box-1 integration, and the minimum revision box-2 planning must
   consume.
5. Capture-health observations: pre/post ring-buffer windows, capture
   overhead bound, required sensor topics, depth-buffer liveness (nonempty
   at every integration view).

I0 fails closed if any of these is missing, unfrozen, stale relative to
`<CANDIDATE>`, or shipped without its pre-validation artifact.

## Gates

Every gate is a stop gate: on failure, preserve the tiered evidence bundle
(§ Failure protocol), repair, rerun only the affected gate. Aggregation is
AND over all gates; per-case correctness is 100% with no retry-to-green.

### I0 -- dispatch, environment and freeze validation (no sim)

- Generation-2 row exists with `dispatch_ready: yes`, `revision =
  <CANDIDATE>`, `plan_revision` = this plan's commit; `agent_start.sh` claim
  passes ownership, dependency (ST09 thread `status: done` + `## Result`
  `outcome: pass`) and generation checks.
- Pinned clean worktree:
  `git worktree add /tmp/elfin-integration-g2 <CANDIDATE>` then
  `git -C /tmp/elfin-integration-g2 status --short` must be empty.
- Fresh default-base build from the workspace root:
  `cd /tmp/elfin-integration-g2 && source /opt/ros/humble/setup.bash && colcon build`
  (default `--install-base install`), exit 0.
- Freeze validation: recompute every manifest hash against the worktree
  files; all match; the event table carries all three categories; every
  frozen view pose carries its IK and visibility pre-validation artifacts;
  revision-chain fields present.
- Artifacts: `INTEGRATION-g2-i0/` with build log, `git rev-parse HEAD`,
  dirty-file count (must be 0), manifest hash table.

### I1 -- offline determinism at `<CANDIDATE>`

Commands (from the worktree, after sourcing its `install/setup.bash`):

```bash
scripts/frame_migration_st09_verify.sh offline
python3 scripts/check_frame_contracts.py --mode migration
python3 -m pytest src/luggage_planning/test src/luggage_gazebo/test -q
```

- Analytic thresholds per the common matrix: position/vector error ≤ 1e-9 m
  pure Python, ≤ 1e-6 m through ROS serialization; identity strings, stamps
  and revision IDs exact.
- Migration checker: exit 0, pending list empty (this closes the standing
  `F23` pending and the two `test_frame_contracts` failures recorded at
  rehearsals 39-41 — those are g5 P6 obligations and must already be green
  on `<CANDIDATE>`; INTEGRATION re-verifies rather than inherits them).
- Standing packing exemption applies only if the suite fails on the ST09
  generation-4 base `2ea7516` and on `<CANDIDATE>` with the identical node
  and `ATLAS_IDENTITY_UNVERIFIED: urdf hash missing` reason, unsuppressed.
- Artifacts: `INTEGRATION-g2-i1/` with each command's full output.

### I2 -- deploy-profile pose cases

`scripts/frame_migration_st09_verify.sh deploy` (cases fixed by the runner:
container yaw π and nonzero translation (1.5, -0.4, 0.2)). Expected: pass at
the same tolerances as I1. Deploy-profile passing authorizes no robot
execution.

### I3 -- exclusive two-box scored smoke (core gate)

Environment: `ROS_DOMAIN_ID=7`, `gui=false`, `use_rviz=false`, exclusive
Gazebo — queue behind any running sim; startup attempt session bounded to 20
minutes; a startup failure before the first case consumes no case and starts
no cursor. No active scene is edited during the run.

```bash
scripts/motion_occ_place_path_run.sh \
  --ws /tmp/elfin-integration-g2 \
  --out docs/status/evidence/frame_migration/INTEGRATION-g2-i3 \
  --n 2 --sequence-ids carryon,standard \
  --place-probe on --atlas on --record-bag
```

Acceptance, in order:

1. **Box 1 (carryon)**: every required event observed exactly once in
   sequence; `release_settle` reports settled with drift within the frozen
   bound; measured verification passes the frozen XY/Z thresholds; the arm
   completes place_exit and returns to the observe pose (`return_complete`);
   the open-loop FJT refusal rule holds (no bare named-pose FJT dispatched
   while the cup is inside the opening).
2. **Measured integration (box 1)**: every frozen cargo view executes with
   a live depth buffer and nonzero YOLO/mask cargo points; aggregate
   integration yields at least the frozen minimum `n_sensor_occupied`;
   `map_revision` increments to the frozen box-1 value. `VIEW_EMPTY` in any
   frozen view is a failure, not a skip.
3. **Box 2 (standard)**: placement planning consumes `map_revision` ≥ the
   box-1 resulting revision (recorded in the dump, not asserted from the
   row); the full box-1 event set repeats; final map revision matches the
   frozen chain.
4. Serialized frame/pose identities stay consistent at every stage boundary
   within ROS tolerance.
5. Bag and capture-health observations match the frozen windows; per-run
   evidence ≤ 1 GiB, total retained frame-migration evidence ≤ 4 GiB.
6. Teardown: stop only this stack with `scripts/stop_sim.sh` after evidence
   is durable.

One deterministic run of the case set; no repetition unless a failure is
repaired, in which case only the affected gate reruns. A two-box smoke is a
finite coverage check, not a reliability certification.

### I4 -- injected contract faults in the live chain

Using the ST09 rehearsal fault mechanisms (same injection flags as the
generation-5 pre-pick/pre-place rehearsal), one fault before pick and one
before place, each once:

- The named `ContractError` code surfaces at the boundary that the freeze
  manifest assigns it to.
- Zero new motion, vacuum or commit effects: spy/counter evidence must show
  zero side-effect counts after rejection. The post-fault arm state must
  remain recoverable (the egress/return machinery is part of the recovery
  path and must not wedge).
- While carrying, the existing vacuum/recovery policy is preserved (no
  silent drop, no forced retry).

Artifacts: `INTEGRATION-g2-i4/` with request/response chains, spy traces and
the two expected error codes.

### I5 -- replay from the recorded run

Replay the I3 bag through the frozen replay command from the manifest
(three paired passes maximum). Median throughput degradation ≤ 10% versus
the captured live window; frame/pose identity replay matches the frozen
extractor expectations, including the view integrations (replayed clouds
must reproduce the same sensor-occupied cell counts within tolerance).

### I6 -- final proofs and both checkers against final sources

- Re-emit all runtime proofs against the final worktree sources into a fresh
  evidence dir (proof re-emit always writes a new dir; a touched row source
  makes the old row stale).
- `python3 scripts/check_frame_contracts.py --mode migration` exit 0,
  pending list empty, proof hashes match.
- `scripts/frame_migration_st09_verify.sh` passes in every mode the runner
  defines.

### I7 -- evidence bundle, lifecycle and merge evidence

- Every gate dir records: base/plan/implementation revisions, source hashes,
  dirty state, exact command, environment/profile, workload, stage timings,
  sample counts, expected and observed values, coverage and
  missing/`not_evaluated` items.
- Completion through `scripts/agent_complete.sh` with `outcome: pass`,
  `revision` = the exact integration commit (real commit, dirty state
  forbidden), tests summary, evidence pointer.
- Merge evidence: record the branch/commit state of
  `/tmp/elfin-integration-g2` and ownership/dependency closure. Merging into
  master remains a reviews/user act after the Result, not part of this plan.

## Failure protocol

Per `.cursor/rules/debug-evidence.mdc` and the master plan's dump matrix
(simulation-cycle row): the selected failed case plus its upstream/downstream
boundaries is the dump unit. Manifest identifies case, stage, first observed
failure, revision/dirty state, config/seed, artifact files, capture window,
missing artifacts, replayability. For stage-boundary mysteries the view
frames are part of the required capture: every integration view saves its
color/depth frame and YOLO/mask counts at failure time (the VIEW_EMPTY
diagnosis on 2026-10-01 required exactly this tier; do not regress it). No
teardown before the bundle is durable; no blind rerun; if required
intermediate capture is unavailable, record the run as incomplete and add
instrumentation before spending another run. A failure reproduced identically
on `2ea7516` and `<CANDIDATE>` is documented as pre-existing, not fixed in
place.

## Coordination

Coordination stays in the primary workspace `docs/agents/` tree
(`AGENT_COORD_ROOT` if run from the worktree); no second mailbox. The g5
worktree `/tmp/elfin-st09-g3` is never entered or modified by this task.
