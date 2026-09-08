# MPF-1 Generation Lifecycle

- date: 2026-09-05 15:19
- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- parent: MPF-20260905
- subtask: MPF-1
- base_revision: b52a4f0
- started_at: 2026-09-05T15:19:10+08:00
- completed_at: 2026-09-05T15:36:00+08:00
- status: done

## Requirement

Implement MPF-1 generation/plan-revision lifecycle, stale completion
rejection, supersede/cancel/ack transitions, and migration to the approved
mailbox freshness plan. Preserve unrelated dirty work and do not start MPF-2.

## Summary

Added `scripts/agent_mailbox.py` as the shared generation-aware lifecycle
engine and routed `agent_start.sh` / `agent_complete.sh` through it. Runnable
notify now requires `generation` and `plan_revision`; start records claim
snapshots; complete rejects terminal, stale generation, stale plan revision,
row/thread mismatch, and stale dependency snapshots; transition and stop-ack
commands preserve historical events and keep claimed stopped rows visible
until acknowledgement. Migration supports dry-run/idempotent legacy upgrades
and refuses ambiguous duplicate lineages.

## Result

MPF-1 acceptance passed in isolated temp-repo tests. MPF-2 remains unstarted;
scheduler/poller stop propagation is intentionally left for its owner scope.

## Pointers

- `scripts/agent_mailbox.py`
- `scripts/agent_notify.sh`
- `scripts/agent_start.sh`
- `scripts/agent_complete.sh`
- `scripts/test_agent_mailbox_freshness.sh`
- `docs/status/evidence/mailbox_plan_freshness/2026-09-05_1519_mpf1/RESULT.md`
