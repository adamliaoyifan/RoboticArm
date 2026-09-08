# 2026-09-04 — adopt incremental checkpoint testing

- role: test
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- checkpoint: n/a
- revision: 0674f84

## Summary

Adopted the reviews workflow: focused gates run as soon as eng publishes
`kind=checkpoint` with a revision; failures reuse the same thread and mailbox
ID as `kind=regression` back to eng; full E2E still runs after every focused
gate passes. No checkpoint row for platform-free height is open yet, so no
Gate was executed.

## Commands

- not run; waiting for `kind=checkpoint` with a non-`n/a` revision.

## Evidence

- none.

## Result

- pass: protocol adopted.
- inconclusive: platform-free height Gates 0-6 not started; `Q-20260904-2`
  remains blocked until eng checkpoint handoffs.

## Pointers

- `docs/agents/reviews/2026-09-04_1204_incremental-checkpoint-testing.md`
- `docs/agents/README.md`
- `docs/agents/test/README.md`
- `docs/plans/platform_free_height_test_plan.md`

## Open

- Reviews: `platform_free_height_test_plan.md` still lists Gate 0-6 without a
  checkpoint table mapping E0-E5 to focused gates, prerequisites, and block
  scope.
