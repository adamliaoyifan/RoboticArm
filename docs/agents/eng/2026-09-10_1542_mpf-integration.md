# 2026-09-10 -- MPF integration and migration

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: MPF-20260905
- subtask: MPF-INTEGRATION
- base_revision: b52a4f041af5efd1370f7c200a55f5cd87a757b7
- started_at: 2026-09-10T15:28:55+08:00
- completed_at: 2026-09-10T15:43:41+08:00

## Summary

Migrated the active legacy mailbox task without rewriting historical threads,
repaired runtime row placement, preserved pre-migration claim freshness, and
passed the complete lifecycle regression on the exact implementation commit.

## Requirement

- Migrate only current runnable mailbox rows and matching threads, preserve
  history and unrelated work, repair registry table insertion, and validate
  migration plus ordinary and stale lifecycle flows end to end.

## Changed

- `scripts/agent_mailbox.py`
- `scripts/agent_register.sh`
- `scripts/test_agent_mailbox_freshness.sh`
- `scripts/test_agent_lifecycle_smoke.sh`
- `docs/agents/README.md`
- `docs/agents/RUNTIME.md`
- `docs/agents/discuss/2026-09-05_1518_mpf-integration.md`

## Verification

- `scripts/test_agent_mailbox_freshness.sh`: pass.
- `scripts/test_agent_scheduler_poller_freshness.sh`: pass.
- `scripts/test_agent_lifecycle_smoke.sh`: pass.
- Python compile and Bash syntax checks: pass.
- `git diff --check`: pass.
- `scripts/check_agent_contract.sh`: live primary pass; exact clean revision
  reports four pre-existing missing D555 thread files listed in the evidence.

## Result

- pass at `5397cafe9fa11e6fe4dcfad352cb6e2fc4bc3a04`.

## Pointers

- `docs/status/evidence/mailbox_plan_freshness/2026-09-10_1528_mpf_integration/RESULT.md`
- `docs/agents/discuss/2026-09-05_1518_mpf-integration.md`
- `docs/plans/mailbox_plan_freshness.md`
