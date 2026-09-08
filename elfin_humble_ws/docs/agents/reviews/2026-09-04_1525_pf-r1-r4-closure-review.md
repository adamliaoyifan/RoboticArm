# 2026-09-04 - PF-R1 through PF-R4 closure review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

PF-R1 and PF-R2 meet their behavioral focused gates but still need an exact
Git output revision and owner Result event. PF-R3 and PF-R4 have passing unit
tests but do not yet meet the reviewed online contract. PF-R5 remains blocked
until all four owner subtasks close through `agent_complete.sh`.

## Findings

1. PF-R3 accepts status for acquisition N-1 as evidence for N through a 0.5 s
   default tolerance, and the node clamps the minimum tolerance to 0.1 s. A
   robot can begin moving between those frames, so this is not same-acquisition
   evidence. Buffer/join status by its stamped acquisition and reject unmatched
   status; add out-of-order and N-1 rejection tests.
2. PF-R4 derives the expected instance/generation from the last received row.
   If only stale frames arrive after a spawn, they become the selected instance
   and are scored against the new GT. The expected identity must come from the
   eval-side spawn/current-box state, never from detector output being tested.
3. PF-R4 reports size and XY coverage but omits yaw and does not gate required
   coverage. Gate 4 must fail when three sizes, required XY offsets, yaw values,
   or trial counts are absent.
4. `active_window_hz` returns total frames divided by the sum of first-to-last
   spans. For N frames there are N-1 intervals, so each window is overcounted;
   a 5 Hz fixture reports about 5.56 Hz. Use interval counts and explicit trial
   collection windows so a detector stall is not mistaken for orchestration.
5. PF-R4 negative-control mode still requires valid top/XY/dimensions, while
   PF-R2 correctly requires raw-only input to produce no valid top. The negative
   control must instead require the explicit segmentation failure and zero
   valid top/full-height outputs.
6. No PF-R1 through PF-R4 note names a Git commit containing the cumulative
   implementation. `0674f84 + working tree` cannot close a dependency.

## Acceptance

- PF-R3 only permits support fitting with matching stamped geometry evidence;
  N-1, absent, malformed, false, stale, and out-of-order status fail closed.
- PF-R4 uses eval-known expected identity, gates size/XY/yaw/trial coverage,
  computes active Hz from observed intervals inside explicit trial windows,
  and has a fail-closed raw-only negative-control verdict.
- Focused and affected package regressions remain green.
- Claude creates a cumulative Git commit and closes PF-R1 through PF-R4 with
  owner Result events that name the resolved commit.

## Pointers

- `docs/plans/platform_free_height_remediation.md`
- `src/luggage_perception/luggage_perception/platform_free_pipeline.py`
- `src/luggage_perception/luggage_perception/eval/gate4_scoring.py`
- `scripts/platform_free_height_gate4_eval.py`
- `docs/agents/eng/2026-09-04_1530_pf-r3_stamped-status-tf.md`
- `docs/agents/eng/2026-09-04_1545_pf-r4_eval-harness.md`

