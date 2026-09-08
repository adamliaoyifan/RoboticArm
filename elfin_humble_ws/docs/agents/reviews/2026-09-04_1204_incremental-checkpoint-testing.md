# 2026-09-04 -- Incremental checkpoint testing

- role: reviews
- agent: codex-scheduler-review
- model: gpt-5
- cli: codex
- status: done

Superseded by `2026-09-04_1452_owner-closed-subtask-flow.md`. Checkpoint
handoff remains historical evidence only; it is not the current workflow.

## Summary

Reviews-to-eng plans now require testable engineering checkpoints. Eng hands
each completed checkpoint directly to test with a checkpoint id and exact
revision, allowing focused verification to run before the full implementation
is complete. Required gate failures block declared dependent work and final
acceptance; all passing focused gates are still followed by an end-to-end
regression.

## Acceptance

- Mailbox rows distinguish questions, tasks, checkpoints, and regressions.
- Checkpoint and regression rows identify both checkpoint and revision.
- Eng and test role templates define immediate checkpoint handoff and result
  feedback.
- A plan cannot treat focused checkpoint tests as a substitute for final E2E.
- Notification and contract-check scripts reject incomplete checkpoint work.

## Checkpoints

| ID | Eng deliverable | Focused test gate | Prerequisites | Blocks |
|---|---|---|---|---|
| WF1 | Machine-readable checkpoint handoff | Contract and helper smoke tests | none | scheduler implementation |
| E2E | Complete reviews-eng-test workflow | Full agent contract check | WF1 | workflow release |

## Risks

- Testing a mutable shared worktree can produce ambiguous results, so every
  checkpoint must carry an exact commit, branch, tag, or isolated revision.
- Multiple checkpoint threads must remain independent so separate test agents
  cannot overwrite one another's mailbox state.

## Verification

- `scripts/check_agent_contract.sh`: pass.
- `bash -n scripts/agent_notify.sh scripts/check_agent_contract.sh`: pass.
- Checkpoint-to-regression smoke test: preserved the mailbox ID, reversed the
  target from test to eng, and updated thread metadata.
- Incomplete runnable task without checkpoint/revision: rejected as expected.

## Pointers

- `AGENTS.md`
- `docs/agents/README.md`
- `scripts/agent_notify.sh`
- `scripts/check_agent_contract.sh`
