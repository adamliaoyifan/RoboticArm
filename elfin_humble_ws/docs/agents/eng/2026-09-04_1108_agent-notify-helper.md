# 2026-09-04 — agent notify helper

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Added a helper script for cross-agent mailbox notifications and documented how
satellite git worktrees should keep coordination in the primary workspace's
`docs/agents/` tree instead of forking separate mailboxes.

## Changed

- `AGENTS.md`
- `docs/agents/README.md`
- `scripts/agent_notify.sh`
- `scripts/check_agent_contract.sh`
- `docs/agents/eng/2026-09-04_1108_agent-notify-helper.md`

## Verification

- `scripts/check_agent_contract.sh`
- `bash -n scripts/check_agent_contract.sh`
- `bash -n scripts/agent_notify.sh`
- `scripts/agent_notify.sh --help`

## Pointers

- `docs/agents/README.md`
