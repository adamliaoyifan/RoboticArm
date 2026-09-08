# Owner Lifecycle Claim Helper

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: AGENT-FLOW-20260904
- subtask: OWNER-LIFECYCLE
- base_revision: 0674f84-wt
- started_at: 2026-09-04T15:06:00+08:00
- completed_at: 2026-09-04T15:35:10+08:00

## Summary

Implemented a machine-readable owner lifecycle for runnable agent work:

- `scripts/agent_start.sh` claims owner-assigned work after dependency checks.
- `scripts/agent_complete.sh` now requires a Claim event before closure, with
  `--started-at` reserved for migrating already-started work.
- `scripts/test_agent_lifecycle_smoke.sh` validates dependency gating, owner
  matching, owner closure, mailbox removal, and metrics visibility in an
  isolated temp repository.
- `scripts/agent_scheduler.py` adds dry-run/watch dispatch planning over
  `OPEN.md` and `docs/agents/RUNTIME.md`, with live queue dispatch limited to
  supported registered adapters and already-claimed threads skipped.
- `scripts/agent_register.sh` registers or refreshes a running CLI session for
  scheduler visibility.
- `scripts/check_agent_contract.sh` now requires the register helper,
  scheduler, and lifecycle smoke script.

## Requirement

Support the owner-closed subtask workflow where reviewers decompose and dispatch
work, and each concrete agent owns implementation, testing, repair, and closure
for its assigned subtask.

## Result

The lifecycle protocol now exposes explicit `Claim` and `Result` events for a
future listener/scheduler. The smoke test passed for the critical routing
cases: blocked dependency, wrong owner rejection, owner start, owner complete,
dependent start, scheduler planning, mailbox cleanup, and metrics parsing.

## Pointers

- `scripts/agent_start.sh`
- `scripts/agent_complete.sh`
- `scripts/test_agent_lifecycle_smoke.sh`
- `scripts/agent_register.sh`
- `scripts/agent_scheduler.py`
- `docs/agents/RUNTIME.md`
- `scripts/agent_flow_metrics.py`
- `docs/plans/agent_subtask_efficiency_experiment.md`
