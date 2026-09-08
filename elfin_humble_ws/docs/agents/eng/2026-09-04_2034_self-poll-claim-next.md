# 2026-09-04 20:34 -- self-poll claim-next runner

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: AGENT-WORKFLOW-20260904
- subtask: SELF-POLL-CLAIM-NEXT
- base_revision: e40df9801dcaae39ddd445bc7238a29673751a0b
- started_at: 2026-09-04T20:22:00+08:00
- completed_at: 2026-09-04T20:34:00+08:00

## Requirement

Establish a polling mechanism that automatically finds tasks assigned to the
current concrete agent/model, specifically `codex/gpt-5`, and starts execution
through the owner claim workflow.

## Summary

Extended `scripts/agent_poll_self.py` with `--claim-next`, `--cli`, and
`--start-script`. The poller now identifies the next ready runnable mailbox row
for the current agent/model, calls `scripts/agent_start.sh` to claim it, and
prints a structured execution prompt for the current session or outer watcher.
The same claim path also runs under `--watch --claim-next` on startup and ready
set changes.

## Result

- pass: `scripts/agent_poll_self.py --agent codex --model gpt-5 --cli codex
  --json` found `Q-20260904-8` / `EXP-A1`.
- pass: `scripts/agent_poll_self.py --agent codex --model gpt-5 --cli codex
  --claim-next` claimed `EXP-A1` through `agent_start.sh`.
- pass: `--watch --claim-next` now uses the same claim path and emits the
  execution prompt as the wake payload.
- pass: `python3 -m py_compile scripts/agent_poll_self.py`.

## Pointers

- `scripts/agent_poll_self.py`
- `docs/agents/README.md`
- `docs/agents/discuss/2026-09-04_2021_exp-a1-nbv-readiness-audit.md`
