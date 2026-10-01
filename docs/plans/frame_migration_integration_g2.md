# Frame migration INTEGRATION generation 2: execution plan

Parent: `FRAME-MIGRATION-20260929`. Obligations unchanged from
`docs/plans/frame_migration_stages.md` (§ INTEGRATION); this document makes
them decision-complete for dispatch. Binding authority for this generation is
the dispatch row's `plan_revision` pointing at the commit containing this file.

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
2. Expected cycle events for the two-box sequence and the numerical
   measured-placement thresholds applied by the live extractor.
3. Capture-health observations: pre/post ring-buffer windows, capture
   overhead bound, required sensor topics.

I0 fails closed if any of these is missing, unfrozen, or stale relative to
`<CANDIDATE>`.

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
  files; all match; expected-events and thresholds sections non-empty.
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
- Migration checker: exit 0, pending list empty, every proof hash matching
  the committed source.
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

1. Every expected cycle event from the frozen manifest is observed exactly
   once per box, in sequence (carryon then standard).
2. The live extractor applies the frozen measured-placement thresholds and
   both boxes verify within them (this closes the class ST09 g5 was chasing:
   place verification against slot rest, not TCP-vs-plan).
3. Serialized frame/pose identities stay consistent at every stage boundary
   within ROS tolerance.
4. Bag and capture-health observations match the frozen windows; per-run
   evidence ≤ 1 GiB, total retained frame-migration evidence ≤ 4 GiB.
5. Teardown: stop only this stack with `scripts/stop_sim.sh` after evidence
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
  zero side-effect counts after rejection.
- While carrying, the existing vacuum/recovery policy is preserved (no
  silent drop, no forced retry).

Artifacts: `INTEGRATION-g2-i4/` with request/response chains, spy traces and
the two expected error codes.

### I5 -- replay from the recorded run

Replay the I3 bag through the frozen replay command from the manifest
(three paired passes maximum). Median throughput degradation ≤ 10% versus
the captured live window; frame/pose identity replay matches the frozen
extractor expectations.

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
missing artifacts, replayability. No teardown before the bundle is durable;
no blind rerun; if required intermediate capture is unavailable, record the
run as incomplete and add instrumentation before spending another run. A
failure reproduced identically on `2ea7516` and `<CANDIDATE>` is documented
as pre-existing, not fixed in place.

## Coordination

Coordination stays in the primary workspace `docs/agents/` tree
(`AGENT_COORD_ROOT` if run from the worktree); no second mailbox. The g5
worktree `/tmp/elfin-st09-g3` is never entered or modified by this task.
