# 2026-09-04 — agent contract hardening

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Strengthened the shared CLI-agent framework by documenting the mailbox
notification pattern, adding role-note templates, and adding a local contract
checker for mailbox, note, architecture-rule, artifact, and executable hygiene.

## Changed

- `docs/agents/README.md`
- `docs/agents/reviews/README.md`
- `docs/agents/eng/README.md`
- `docs/agents/test/README.md`
- `scripts/check_agent_contract.sh`

## Verification

- `scripts/check_agent_contract.sh`

## Pointers

- `docs/agents/reviews/2026-09-04_1053_agent-framework-review.md`
