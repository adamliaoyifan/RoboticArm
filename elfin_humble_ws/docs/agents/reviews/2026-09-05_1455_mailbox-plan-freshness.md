# Mailbox Plan Freshness Review

- date: 2026-09-05 14:55
- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- task: mailbox-plan-freshness
- revision: working-tree
- status: done

## Summary

The mailbox lifecycle needs generation-aware freshness checks from dispatch
through completion. A proposed executable plan is ready for the required
distinct Codex consensus review; implementation has not been dispatched.

## Finding

The current mailbox and scheduler flow does not hard-guarantee latest-plan
execution after a runnable task has already been queued or claimed. The
scheduler re-reads `OPEN.md` before dispatch and skips already claimed threads,
but there is no enforced plan generation, superseded/cancel state, or
completion-time stale-plan rejection. Agents are instructed to read the thread
and `OPEN.md` before work, so appended updates can be seen by a compliant
agent, but stale queued prompts and in-progress claims are not automatically
invalidated.

## Required Hardening

Add a monotonic plan generation or approved-plan revision to runnable rows and
thread metadata; add explicit `superseded` or `cancelled` thread status; make
pollers and scheduler notify/stop owners when a claimed thread is superseded;
and make `agent_complete.sh` reject passing closure when the claimed generation
is no longer current.

## Proposed Plan

The bounded design and subtask decomposition are in
`docs/plans/mailbox_plan_freshness.md`. It defines generation authority,
plan-revision identity, claim and dependency snapshots, atomic terminal
transitions, owner stop acknowledgement, migration, and end-to-end acceptance.

The distinct Codex review reached consensus on the initial plan and on the
bounded owner-identity/runtime-registry amendment. The plan is approved;
implementation dispatch uses the exact commit containing this text.

## Consensus

- reviewer: `codex/gpt-5`
- session: `01a06a53-6433-7180-ae5b-c7963c8f1e26`
- result: reached
- approved plan SHA-256:
  `585a007cefc8c2c64a16d2ded7f0004825fdb075482f5e39af850dbe50015c6c`

## Pointers

- `docs/plans/mailbox_plan_freshness.md`
- `docs/agents/README.md`
- `scripts/agent_notify.sh`
- `scripts/agent_start.sh`
- `scripts/agent_complete.sh`
- `scripts/agent_scheduler.py`
- `scripts/agent_poll_self.py`
