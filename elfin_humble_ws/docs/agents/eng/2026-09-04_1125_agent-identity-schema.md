# 2026-09-04 — agent identity schema

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Split agent coordination identity into role pools and concrete agents. Mailbox
rows now carry `to_role`, `to_agent`, `from_role`, and `from_agent`; role notes
carry `agent`; and `agent_notify.sh` can target or identify a concrete worker
while still allowing role-pool handoffs with `to_agent=any`.

## Changed

- `AGENTS.md`
- `.cursor/rules/agent-logs.mdc`
- `docs/agents/README.md`
- `docs/agents/*/README.md`
- `docs/agents/*/*.md`
- `docs/agents/discuss/OPEN.md`
- `scripts/agent_notify.sh`
- `scripts/check_agent_contract.sh`

## Verification

- `scripts/check_agent_contract.sh`
- `bash -n scripts/check_agent_contract.sh`
- `bash -n scripts/agent_notify.sh`
- `scripts/agent_notify.sh --help`
- `/tmp` sandbox notify smoke: new row, existing `--thread` row update, bad
  `AGENT_COORD_ROOT` refusal

## Pointers

- `docs/agents/discuss/2026-09-04_1116_agent-notify-followup.md`
