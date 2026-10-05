# ST09: three-box quick track (carryon, standard, carryon)

Date: 2026-10-05. Starting point: worktree `/tmp/elfin-st09-converge-i`,
branch `integration/st09-converge-i-g3-fix`, commit `82a6bc7` or later.

## Objective

In the deployment simulation, complete the sequence carryon, standard,
carryon end to end using online detected dimensions, the current occupancy
map, and online placement candidates. Every box must go through perception,
pick, placement, release, map update and home return. A route failure for one
candidate must reject that candidate and continue to the next candidate in
the same request. Boxes on the same support layer keep at least 0.05 m of
oriented-footprint boundary clearance; a stacked box is exempt from this
horizontal gap only against its explicit support parent. The track is complete
after two consecutive full passes at one frozen code/config/install candidate,
each starting from a fresh stack. Placement candidate IDs and trajectories are
not frozen production inputs.

Scope and claims:

- This track demonstrates completion and repeatability for this one
  scenario. It does not claim a general success rate.
- The generation 2 matrix failure stays on record, unchanged. Wilson
  statistics and general planner coverage remain owned by
  `plan/2026-10-05_st09-i-g3-attribution-plan.md`.
- This track replaces the matrix gate for this scenario only. Record that
  decision on the discuss thread before the first scored attempt.
- Atlas stays enabled in live runs. Atlas-off is only a diagnostic
  comparison. The matrix difference (112 vs 116) is an aggregate gap of four
  successes. It does not identify four specific atlas-caused failures.

## Hard constraints (unchanged for the whole track)

- Planning budgets, IK, geometry and collision tolerances, collision
  certification, and the 0.01 rad retreat gate.
- The physical scene, online perception, and atlas enabled.
- No prescribed slots and no injected ground truth.
- `same_layer_clearance_m=0.05` for this track. It is part of the frozen
  runtime/config hash and may not change between the two passing sessions.
- Same-layer clearance uses the distance between oriented footprints, not
  centre distance or unrotated AABBs. Equality passes; a smaller gap fails.
- A stacked box may overlap its explicit support parent's XY footprint, but
  must still pass support coverage, top-clearance, container containment and
  positive-volume non-penetration. It keeps normal clearance/collision rules
  against every non-support box.
- Within a session: no map resets between boxes, no skipped boxes.
- Across sessions: each independent session starts with an empty container
  and a newly initialized map. Boxes from different sessions are never
  combined.
- Every cycle follows the production operator boundary:
  `RETURN_PICK_OBSERVE -> WAIT_PICKUP_READY -> DETECT`. Reaching the observe
  pose publishes an `OperatorPrompt`; only a matching one-shot `PickupReady`
  may start the next cycle. Waiting for the operator has no detection timeout.
- In simulation, accepting `PickupReady` triggers `spawn_next_box`, then the
  existing generation/stamp readiness checks and detection. On hardware the
  operator places the physical box before sending `PickupReady`; no spawn is
  performed. Neither path uses RGB pixel difference as evidence of a new box.

If a blocker can only be solved by changing one of these, stop and report
the measured conflict.

## Execution order

Production wiring and capture checks, then focused tests, then
commit/build/freeze, then startup rehearsal, then a three-box attempt, then
repair of the first blocker if needed, then two consecutive full passes at
the same candidate.

## Before the first scored attempt

Record the starting revision (commit and dirty count) before any of these
checks. P1 and P5 may need code changes. Any such repair gets a focused
regression and a commit before the candidate is frozen.

- **P1. Production seed provenance (hard).** Pick one dev case that
  previously had zero goals. Run the installed production goal-generation
  path on it, not `route_matrix_eval.py`. Assert both of the following:
  - `tool_down_j1_minus_pi` was attempted, as shown by the seed source
    recorded on the attempt;
  - that seed produced a valid goal.
  A successful goal from some other seed does not pass P1. If the
  production path does not record seed provenance, add it as the repair.
- **P2. Install matches source (hard).** Rebuild the affected packages into
  the exact overlay the runner loads (`install-st09`). Record its module
  hashes and check that they match the committed source.
- **P3. Historical whole-chain failures (may end `not_replayable`).** Audit
  the T2 bundles of the two earlier `PLANNING_BUDGET_EXHAUSTED` failures for
  request, start joints, attachment transform, map snapshot and revision,
  and atlas identity. If a bundle is complete, replay it offline with atlas
  on and off. If not, record `not_replayable` with the missing fields. A
  missing historical field is not a tooling defect.
- **P4. Replay command (hard).** Provide an offline command that takes a
  captured live planning request and runs it with atlas on and off. Show it
  working on one complete fixture. Unlike P3, this may not end
  `not_replayable`.
- **P5. Capture completeness (hard).** Confirm the runner records, per box:
  the planning request, start joints, attachment transform, map
  snapshot/revision/source, candidate targets, the selected route or the
  rejection reasons, and stage timings. For execution failures it must also
  record the failed leg's trajectory, the joint and vacuum trace, and the
  release poses. Add only what is missing, and test the addition offline.
- **P6. Focused regressions (hard).** The slot64 replay must give a first
  DESCEND at tier 0 with fraction >= 0.95. Candidate isolation, DESCEND
  pairing, payload gates, retreat readiness and the three-box evidence judge
  must all pass.
- **P7. Startup readiness (hard).** Run `scripts/sim_startup_rehearsal.sh`
  until controller, perception and planning reach ready with exactly one
  `/clock` publisher, followed by a clean teardown with zero residual
  processes. Rehearsal launches are not scored attempts.
- **P9. Operator-cycle gate (hard before attempt 2).** Behaviourally exercise
  two cycles through the production `OperatorPrompt` / `PickupReady` contract.
  Require a unique request ID per prompt; reject blank, stale, duplicate and
  wrong-ID confirmations without spawning or detecting. For simulation,
  verify the accepted confirmation happens before box spawn and that detection
  consumes RGB/depth/semantic acquisitions newer than that acceptance/spawn
  boundary. Do not require a pixel change. For hardware, verify the same
  accepted confirmation starts detection without invoking a simulation spawn.
- **P10. Attempt-1 hold-track attribution (hard before attempt 2).** Treat this
  separately from P9. The current launches disable `SuitcaseViewWait`, and its
  "waiting for suitcase RGB update" line is informational. Reproduce or replay
  the generation-4 path from attempt 1 and show why all 11 authorization reads
  selected `hold_track`. Attempt 1 itself is not replayable for this cause: it
  did not preserve mask identity/pixel count or filter source/point count/cloud
  stamp. Add T1/T2 capture for the box epoch stamp; mask and instance-mask
  stamps, generation, ID and nonzero counts; depth stamp; exact join stamp;
  filter source and raw/tracked point counts; tracker generation/instance and
  pre/post source; and every `DetectionFrame` stamp/source/support gate used by
  authorization. Inspect these boundaries, not RGB pixel difference. First
  rule out an in-flight pre-epoch depth/mask join being committed after the
  tracker reset: its acquisition stamp and processing epoch must both be newer
  than / equal to the accepted Box 2 epoch boundary.
  If a post-epoch, same-generation, exact-stamp, support-valid `measure` frame
  existed within `cloud_max_age_sec` but `latest()` selected a newer hold,
  change selection to choose the newest eligible fresh measure from the
  existing window. Do not authorize a held cloud, extend freshness, or reuse a
  previous generation. Add a regression with interleaved
  `measure, hold_track, hold_track` frames and a regression where an in-flight
  pre-epoch join completes after reset and must be discarded.

  Attribution is now closed by
  `g3_p10_hold_repro_20261005`: a generation-3 join acquired at 113.652 s
  committed after the generation-4 epoch at 113.769 s, and per-frame source
  was copied from an approximately 1 Hz latest `stats_json` value rather than
  joined by acquisition stamp. The bounded repair is therefore fixed:
  - bind acquire generation/instance to every taken join and discard it at
    commit if the live epoch differs;
  - bind filter source to `DetectionFrame` by the exact acquisition stamp;
    rate-limited aggregate stats remain diagnostics only;
  - do not scan historical frames for a measure and do not change authorization
    duration, cloud age, support rules or hold-track rejection.

- **P11. Candidate fallback (hard before attempt 2).** Exercise the installed
  production selector with at least two placement candidates in one request.
  Candidate A must fail deterministically at a named route boundary (the
  primary fixture is a collision with an already placed box); candidate B must
  produce a complete certified route. The service must return B without a new
  placement request. IK, collision, Cartesian fraction, excursion, branch
  continuity and retreat/return failures are candidate-level rejections.
  Evidence records candidate ID, failing stage/reason, elapsed and remaining
  budget, and final selection. If the request fails, it records attempted and
  untried candidate counts plus every observed rejection. Repeating a known
  deterministic short-edge failure until the global budget expires is a fail.
- **P12. Same-layer spacing and stacking (hard before attempt 2).** Behavioural
  tests use oriented footprints and `same_layer_clearance_m=0.05`: 0.049 m
  same-layer gap rejects; 0.050 m passes; a rotated footprint is evaluated by
  its polygon; an upper box may overlap its explicit support parent when the
  existing support threshold, top clearance and non-penetration pass; the same
  upper box still rejects insufficient support or a clearance violation with
  a non-support box. The parameter appears in runtime identity, capture and
  freeze manifests.

## Limits

- **Wall clock:** 8 hours total, counting rehearsal, diagnosis, builds and
  tests.
- **Scored attempts:** at most 5, counting both the first pass and the
  confirmation. An attempt is consumed once box 1 begins. A failure before
  box 1 is a readiness failure: repair it through P7, and it consumes no
  attempt.
- **Same-blocker stop:** if the same demonstrated cause recurs at the same
  stage after its fix, stop and report. The same failure code alone (for
  example another `PLANNING_BUDGET_EXHAUSTED`) does not count as the same
  blocker.
- **Undiagnosable failures:** if a live failure lacks the inputs P4 needs,
  repair capture before the next attempt. Never rerun a failure that cannot
  be diagnosed.
- **Running out of attempts:** if the fifth attempt is the first full pass,
  report `confirmation pending`, not completion.

## To-do

1. Record the starting revision on the discuss thread, together with the
   decision to replace the matrix gate.
2. Close P1, P4, P5, P11 and P12, plus any capture or provenance repairs. Each repair
   gets a focused regression.
3. Run P6 and P3.
4. Commit, rebuild into `install-st09` (P2), and freeze candidate K. Write
   `candidate_manifest.json` into each attempt's evidence directory, so a
   later candidate cannot overwrite it. The manifest holds: commit, dirty
   count, deployment config (`scene_tf_deploy.yaml`,
   `atlas_runtime_deploy.yaml`), atlas identity and hash, and the installed
   module hashes.
5. Run P7: startup rehearsal with clean teardown.
6. Add the P10 boundary capture and run a non-scored two-spawn reproduction.
   Classify the first divergence before changing perception behaviour, then
   add the focused regression and smallest supported repair. This is the A1
   repair; P9 is a workflow requirement and is not evidence that the
   hold-track defect is fixed.
  P10 passes only when no pre-epoch join commits, every `DetectionFrame`
  source equals the filter source at the same stamp, same-stamp measure runs
  support fitting, and same-stamp hold remains unauthorized. Preserve the
  newly reached `PLANNING_BUDGET_EXHAUSTED` request and diagnose/replay it
  offline before spending attempt 2.

   P10 passed in `g3_p10_repair_repro_20261005`: Box 2 had 64 joins, zero
   epoch crossings, zero exact-stamp source mismatches, hold stayed support
   invalid, and a measure authorized on read 4. The subsequent planning
   failure is not replayable because `candidate_targets` were omitted. Repair
   planning capture before another sim: preserve every candidate ID, slot pose,
   synthesized tool/waypoint targets, reachability class and seed sources,
   together with the already-required start joints, attachment transform, map
   snapshot/revision and atlas identity. Validate the replay tool on a complete
   fixture, then repeat the non-scored two-box reproduction. A live planning
   failure must replay atlas-on and atlas-off before attempt 2.
7. Run P9 with a carryon-to-standard non-scored rehearsal. Return to
   `pickup_observe_sim`, publish the next prompt, wait for explicit
   `PickupReady`, spawn standard only after acceptance, and require a fresh
   matching-generation measured result. An already visible box is valid; the
   gate is command and acquisition ordering, not image difference.
   The same rehearsal also validates repaired planning capture. If planning
   fails, its request must be replayable and diagnosed before scoring. If it
   succeeds, retain one complete captured request and show atlas-on/off replay
   reaches the same stage before scoring.

   The completed capture reproduction localizes the failure to reachable slot
   64: exact IK goals exist, 74 edges were built, four goals remained open and
   the 9.317 s production search budget expired. Before changing production
   scheduling, replay this single candidate against the captured revision-2
   map, attachment and start joints in two modes:
   - exact production policy, which must reproduce `SEARCH_INCOMPLETE`;
   - diagnostic reference search with a 60 s ceiling and no production claim.
   If reference search finds a certified route, record its first complete node
   and edge sequence and repair only the ordering/deduplication that delayed
   that sequence, inside the unchanged production budget. If reference search
   finds no route, retain atlas `reachable` as local IK truth, classify the
   candidate as route-incomplete after the existing edge-class repair caps,
   and continue to the next candidate inside the unchanged total budget. Do
   not mark it atlas-unreachable and do not increase time, attempts or
   tolerances.
8. Clean-room check: no `ros2 launch luggage_gazebo` process, no residual
   bridges, no `/clock` publisher. Use `ROS_DOMAIN_ID=7`, headless, and the
   pidfile.
9. Run attempt N into a new directory:

   ```bash
   scripts/st09_i/s5_m3_run.sh \
     --out docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3/g3_three_box_run03_aN \
     --n 3 --sequence carryon,standard,carryon \
     --deadline-sec 1200 --no-record-bag
   ```

   After launch, require exactly one `/clock` publisher and readiness before
   box 1.
10. On failure:
   - Flush the evidence, run `scripts/stop_sim.sh`, and check for residual
     processes.
   - Find the first failing box and stage from its actual inputs. For a
     planning failure, run the P4 atlas on/off replay.
   - Add a regression that reproduces the failure, implement the smallest
     supported fix, and commit.
   - Return to step 4 with candidate K+1, and restart from box 1.
11. After a full pass: stop, run the clean-room check, and run the same
   candidate again from a fresh stack with an empty container and a new map.
   A failure reopens that blocker. Any code change means the next candidate
   needs two new consecutive full passes.
12. Close out: write the eng note and the thread reply with both evidence
    paths, the candidate commit and its manifest.

## Per-session acceptance (all required)

- All three boxes complete and finalize with empty failure codes, and every
  home return succeeds.
- Each cycle reaches pickup observe, emits one prompt, accepts exactly one
  matching `PickupReady`, and only then spawns (simulation) or detects
  (hardware). Prompt/confirmation ordering and request IDs are in evidence.
- Each box has its own request and box identity, and all 11 ordered boundary
  events.
- Every selected placement comes from that cycle's online solver response.
  Saved candidate IDs and plan-only trajectories are diagnostic evidence only.
- Every rejected candidate has a named stage/reason. If a selected candidate
  is not the first candidate, the same request records the earlier rejection
  and automatic fallback. No deterministic candidate consumes the whole
  request budget through repeated equivalent failures.
- Exactly two revision handoffs (1 to 2 and 2 to 3), each joined to a
  committed map revision.
- Boxes 2 and 3 plan on `release_integrated` or measured maps that pass the
  existing freshness gate.
- Every retreat starts from a fresh, verified joint state within 0.01 rad.
- Geometry: XY <= 4 cm, Z <= 3 cm, drift <= 2 cm, tilt <= 2 deg,
  coverage >= 0.40, inside the container, and no floor-through.
- Every same-support-layer oriented-footprint pair has a measured boundary
  gap >= 0.05 m. A stack records its support parent and may overlap that parent
  in XY only when support coverage, top clearance and non-penetration pass;
  other box pairs remain subject to clearance/collision checks.
- The existing three-box judge returns pass with no missing evidence.

## Completion rule

Two consecutive complete passing sessions at one frozen candidate. Each
session is independent and starts from an empty container. Partial
sequences, map resets between boxes, and boxes taken from different
sessions do not count.
