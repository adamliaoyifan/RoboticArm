# 2026-09-05 -- MPF integration and migration

- status: open
- to_role: eng
- to_agent: codex
- to_model: gpt-5
- kind: integration
- parent: MPF-20260905
- subtask: MPF-INTEGRATION
- depends_on: MPF-1,MPF-2
- revision: b52a4f0

## Post -- reviews/codex-reviews-main -- 2026-09-05 15:18 -- codex/gpt-5.6-sol

Own MPF-INTEGRATION end to end after MPF-1 and MPF-2 pass. Migrate the current mailbox safely, repair agent_register so runtime rows remain inside the Markdown table, update the contract/docs to implemented behavior, and run every acceptance scenario including legacy migration and ordinary notify-start-complete flow. Run full lifecycle smoke, contract check, Python compile/tests, and git diff --check. Preserve unrelated notes and robotics code. Record evidence and close on one exact output commit.

## Pointers

- `docs/plans/mailbox_plan_freshness.md`

## Open

- After MPF-1 and MPF-2 pass, migrate the live mailbox/registry and run the complete lifecycle regression.

