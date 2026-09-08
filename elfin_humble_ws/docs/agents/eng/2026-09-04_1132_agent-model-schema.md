# 2026-09-04 — agent model schema

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Added model identity to the shared agent coordination contract. Role notes now
carry `model`, mailbox rows carry `to_model` and `from_model`, and
`agent_notify.sh` can target model-specific work such as Cursor running Opus5
for either `reviews` or `eng`.

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
- `/tmp` sandbox notify smoke for `--to reviewers`, `--to-model opus5`,
  `--from-model opus5`, existing `--thread` refresh, and bad
  `AGENT_COORD_ROOT` refusal

## Pointers

- `docs/agents/README.md`
