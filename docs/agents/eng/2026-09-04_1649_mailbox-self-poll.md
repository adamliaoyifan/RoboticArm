# 2026-09-04 — Cursor mailbox self-poller

- role: eng
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- parent: n/a
- subtask: n/a
- base_revision: n/a
- started_at: 2026-09-04T16:45:00+08:00
- completed_at: 2026-09-04T16:49:41+08:00

## Summary

Added `scripts/agent_poll_self.py` so file-only sessions can poll `OPEN.md`,
queue rows that match their concrete agent/model, and wake only when the ready
set changes. This Cursor/Grok session now watches every 45s. PF-R7 is queued
and waiting on PF-R5 and PF-R6; it is not claimed.

## Changed

- `scripts/agent_poll_self.py`
- `scripts/test_agent_lifecycle_smoke.sh`
- `scripts/check_agent_contract.sh`
- `docs/agents/README.md`
- `docs/agents/RUNTIME.md`
- `AGENTS.md`

## Verification

- `scripts/check_agent_contract.sh`
- `scripts/test_agent_lifecycle_smoke.sh`
- one-shot poll queued `Q-20260904-2` / PF-R7 as `wait`

## Requirement

File-only CLIs should fetch and queue their own mailbox rows without taking
other agents' work or claiming blocked dependencies.

## Result

- pass: poller classifies identity-matched rows; smoke covers wait-then-ready.

## Pointers

- `/tmp/elfin_agent_queue_cursor_grok-4.6.json`
- `docs/agents/discuss/OPEN.md`
