# 2026-09-10 -- PF-R10 generation 3 closed-loop placement

- role: eng
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: open
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R10
- base_revision: 0001c413147e4a2f00a01801d6a8ac16ca92ba93
- started_at: 2026-09-10T14:46:17+08:00
- completed_at: n/a

## Summary

User-directed PF-R10 generation 3. Closed-loop Gazebo placement now verifies
entity identity, XY, roll/pitch, yaw, and persistence against
`/world/airport_loading/pose/info` on the existing world service bridge,
retries a bounded number of times, and fail-closes unplaceable trials.
Stale spawner/detector fixtures are repaired. Scoring no longer lets a
failed placement drop out of the Gate-4 decision. Three consecutive
`gate4_short6` runs plus PF-G6S are not yet recorded.

## Requirement

- Bounded closed-loop placement vs `/world/airport_loading/pose/info` after
  settle; fail the trial explicitly if the requested pose cannot be
  established.
- Do not filter failed placements out of scoring or change detector
  geometry thresholds.
- Keep `--warmup-frames 30`, report untrimmed recovery, fail recovery
  longer than 1.4 s or fewer than 30 settled scored frames per trial.
- Repair the three stale-fixture failures.
- Pass C1-C3 on one clean exact commit: three consecutive stored six-trial
  Gate-4 runs, PF-G6S, C3 teardown with zero residual processes.

## Changed

- `src/luggage_gazebo/scripts/pickup_box_spawner_node.py`
- `src/luggage_gazebo/launch/sim_world.launch.py`
- `src/luggage_gazebo/package.xml`
- `src/luggage_gazebo/test/test_pf_r5a_fix1_spawn_fail_closed.py`
- `src/luggage_gazebo/test/test_pf_r10_place_verify.py`
- `src/luggage_perception/test/test_pf_r6_detector_instrumentation.py`
- `src/luggage_perception/test/test_pf_g4h_evaluator.py`
- `src/luggage_perception/luggage_perception/eval/gate4_scoring.py`
- `scripts/platform_free_height_gate4_eval.py`

## Verification

- not run: focused pytest in progress.

## Result

- open: implementation in progress; C1-C3 not yet recorded.

## Pointers

- `docs/agents/discuss/2026-09-10_1445_pf-r10-g3-closed-loop-place.md`
- `docs/agents/discuss/2026-09-07_2039_pf-r10-gate4-integration.md`
- `docs/agents/reviews/2026-09-10_1425_claude-pfr9-pfr10-sim-review.md`
- `docs/plans/pf_r8_r9_perception_acceptance.md`
