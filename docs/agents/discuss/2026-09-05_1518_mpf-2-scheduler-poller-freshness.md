# 2026-09-05 -- MPF-2 scheduler and poller freshness

- status: done
- to_role: eng
- to_agent: codex
- to_model: gpt-5
- kind: subtask
- parent: MPF-20260905
- subtask: MPF-2
- depends_on: MPF-1
- revision: b52a4f0

## Post -- reviews/codex-reviews-main -- 2026-09-05 15:18 -- codex/gpt-5.6-sol

Own MPF-2 end to end after MPF-1 passes. Implement scheduler/poller stop-first behavior for live and file-only owners, idempotent stop delivery, replacement blocking until acknowledgement, highest-generation dependency selection, and downstream stale rejection after upstream replan. Preserve unrelated work. Record focused test evidence and an exact output commit; close only after MPF-2 acceptance passes.

## Pointers

- `docs/plans/mailbox_plan_freshness.md`

## Open

- After MPF-1 passes, implement MPF-2 stop propagation and highest-generation dependency freshness.

## Claim -- eng/codex -- 2026-09-05 15:55 -- codex/gpt-5

- started_at: 2026-09-05T15:55:13+08:00
- claimed_generation: 1
- claimed_plan_revision: b52a4f0
- claimed_dependencies: MPF-1=1

## Result -- eng/codex -- 2026-09-05 16:08 -- codex/gpt-5

- outcome: pass
- completed_at: 2026-09-05T16:08:39+08:00
- revision: 90c632917c47fad8b64578e3356d3fd1b02c9a69
- tests: scripts/test_agent_scheduler_poller_freshness.sh pass; scripts/test_agent_mailbox_freshness.sh pass; scripts/test_agent_lifecycle_smoke.sh pass; py_compile pass; bash -n pass; git diff --check pass; scripts/check_agent_contract.sh pass
- summary: MPF-2 implemented scheduler/poller stop-first behavior, live and file-only stop reporting, idempotent stop notice leases, replacement blocking until StopAck, highest-generation dependency readiness, and legacy dependency snapshot compatibility before live migration. MPF-INTEGRATION not started.
- evidence: docs/status/evidence/mailbox_plan_freshness/2026-09-05_1555_mpf2/RESULT.md

