# 2026-09-14 -- PF-R7 generation 3 bounded conditional acceptance

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Recorded the user-directed PF-R7 acceptance amendment. Known, fully evidenced
YOLO proposal misses are now unscored and do not advance or reset the
three-slot sequence. PF-R7 continues to fail on accepted-proposal downstream
faults, false cargo, geometry errors, temporal safety defects, resource/lifecycle
failures, or incomplete evidence.

The replacement plan makes execution bounded: six eligible cases per slot,
three slots, at most 12 attempts per slot, 36 attempts/two stack resets/45
minutes per campaign, and no automatic rerun after a scored failure. Budget
exhaustion produces `inconclusive`, flushes evidence and tears down the stack.

## Acceptance

- G0-G3 deterministic/fail-closed suites pass on one clean reproducible
  descendant of the PF-R10 accepted revision.
- Three bounded slots each contain two eligible carryon, standard and large
  cases and pass the conditional G4 geometry limits.
- PF-R10 C2 buffer, lag, workload-adjusted RSS and zero-residual bars pass on
  eligible windows.
- Every exclusion has the complete proof required by the exact class boundary;
  missing evidence cannot waive or pass a case.
- All watchdog and reset paths terminate within their numeric budgets and
  preserve failure evidence before reset.

## Subtasks

| ID | Owner agent/model | Depends on | Base revision | Scope | Acceptance | Required tests | Commit evidence |
|---|---|---|---|---|---|---|---|
| PF-R7-H1 | `cursor/grok-4.6` | PF-R1..PF-R10 accepted chain | `874b4bdb38c16b771975718fe424c431c080d178` | Implement bounded attempt classifier, watchdog/reset driver and ledgers; then run one bounded PF-R7 generation-3 audit | Exact A-D gates and budgets in the plan; known detector misses unscored only with complete proof | classifier/reset tests x3, affected evaluator/PF-R10 tests, build, one bounded live campaign | clean passing commit, role note, evidence/verdict and teardown proof |

## Risks

- Broadly calling any `no_cargo_cloud` a YOLO miss would hide mask, join or
  geometry failures. The plan therefore requires proof that no accepted
  proposal existed and that the online downstream path was never entered.
- Excluding misses can starve the denominator. Fixed eligible counts and hard
  attempt/time limits yield `inconclusive`, never a vacuous pass or endless
  retry.
- POS-1 is already claimed and shares the exclusive simulator; PF-R7-H1 unit
  work may proceed without Gazebo, while live campaigns must serialize.

## Pointers

- `docs/plans/pf_r7_generation3_bounded_acceptance.md`
- `docs/agents/discuss/2026-09-07_2057_pf-r7-e2e-audit-generation2.md`
- `docs/status/evidence/platform_free_height/2026-09-14_pfr7_e2e/sha_60dafb7/`
- `docs/plans/place_only_perfect_geometry_sim.md`
- `.cursor/rules/debug-evidence.mdc`
- `.cursor/rules/sim-lifecycle.mdc`
