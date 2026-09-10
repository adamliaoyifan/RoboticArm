# 2026-09-10 -- Claude PF-R9/PF-R10 simulation review

- role: reviews
- agent: codex-reviews-main
- model: gpt-5
- cli: codex
- status: done

## Summary

PF-R9 generation 2 remains a passing depth-primary implementation at
`bcb54c9`: the camera payload boundary is RGB, aligned depth, and CameraInfo;
camera geometry is deprojected locally; and world transforms use acquisition
stamps and fail closed. PF-R10 is not yet accepted because the required three
consecutive Gate-4 runs are absent and Gazebo placement is intermittently
invalid. Current PF-R10 focused regression also has three stale-fixture
failures, and its eng role note violates the metadata format. The next gate is
therefore simulation placement closure plus the
unchanged PF-R10 C1-C3 run set. D555 live calibration and deployed TF changes
are deferred until the board is ready and do not gate simulation work.

## Acceptance

- Closed-loop Gazebo placement verifies entity identity, XY, roll/pitch, and
  persistence with bounded retries; an unplaceable trial fails explicitly.
- The 30-frame warmup is fixed at the reviewed 21 Hz profile, with untrimmed
  recovery evidence, recovery no longer than 1.4 seconds, and at least 30
  settled scored frames per trial.
- Three consecutive six-trial Gate-4 runs, PF-G6S, and zero-residual teardown
  pass on one clean exact commit with all raw artifacts retained.
- The spawner/detector fixture regressions pass, closed-loop placement has
  focused success and retry-exhaustion tests, and the agent contract passes.
- PF-R7 then independently audits that passing commit before a higher DSIM-1
  generation is bound to it.
- Hardware D8/RSS, hand-eye solve, and TF-tree application remain separately
  parked and cannot be used to block or waive the simulation gates.

## Risks

- Current `HEAD` includes unaccepted PF-R10 repair commits; it is not a passing
  baseline merely because PF-R9 passed at `bcb54c9`.
- The active Gazebo `/d435/points` bridge is a known DSIM-1 gap, not evidence
  that PF-R9 reintroduced transported camera geometry.
- The parked hardware thread used the wrong D455 product name; execution must
  target the installed D555 PoE.
- Current focused PF-R10 test result: 46 passed, 3 failed because direct-node
  fixtures omit newly introduced `_set_pose_cli` or scratch-buffer members.

## Pointers

- `docs/status/evidence/reviews/2026-09-10_pfr9_pfr10_sim_review/RESULT.md`
- `docs/agents/discuss/2026-09-09_1147_pf-r9-g2-payload-depth-primary.md`
- `docs/status/evidence/platform_free_height/2026-09-09_pfr9_g2_payload/RESULT.md`
- `docs/agents/discuss/2026-09-07_2039_pf-r10-gate4-integration.md`
- `docs/status/evidence/platform_free_height/2026-09-09_pfr10_gate4_integration/RESULT.md`
- `docs/plans/d555_sim_depth_pipeline_execution.md`
- `docs/agents/discuss/2026-09-09_1939_d455-hardware-validation-deferred.md`
