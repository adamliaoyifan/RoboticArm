# MPF-2 Scheduler Poller Freshness

- date: 2026-09-05 15:55
- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- parent: MPF-20260905
- subtask: MPF-2
- base_revision: b52a4f0
- started_at: 2026-09-05T15:55:13+08:00
- completed_at: 2026-09-05T16:03:09+08:00
- status: done

## Requirement

Implement MPF-2 scheduler/poller stop propagation, stop-first owner behavior,
idempotent stop delivery, replacement blocking until acknowledgement, and
highest-generation dependency freshness. Preserve unrelated work and do not
start MPF-INTEGRATION.

## Summary

Updated `scripts/agent_scheduler.py` and `scripts/agent_poll_self.py` to treat
claimed `superseded`/`cancelled` rows that remain in `OPEN.md` as stop items.
Poller queues stop items before active/ready work and therefore does not
auto-claim replacements while a stopped generation is unacknowledged.
Scheduler reports live Codex stop actions, file-only stop visibility, and
idempotent stop notice leases; replacement work for the same owner waits for
StopAck. Dependency readiness now uses the highest generation for a lineage
instead of the first matching thread on disk.

## Result

MPF-2 acceptance passed in isolated temp-repo tests. MPF-INTEGRATION remains
unstarted; live mailbox and runtime registry migration are still integration
scope.

## Pointers

- `scripts/agent_scheduler.py`
- `scripts/agent_poll_self.py`
- `scripts/test_agent_scheduler_poller_freshness.sh`
- `docs/status/evidence/mailbox_plan_freshness/2026-09-05_1555_mpf2/RESULT.md`
