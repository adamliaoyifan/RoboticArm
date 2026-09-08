# 2026-09-05 -- Agent communication contract refresh

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Reconciled the durable communication contract with the generation-aware
mailbox helpers. The thread is now explicitly authoritative, `OPEN.md` is a
queue projection, and `RUNTIME.md` is transient presence. New runnable work
requires an exact owner, base and plan revisions, generation, and
`dispatch_ready`. Claimed requirement changes use a higher-generation
replacement instead of in-place edits. Distinct Codex consensus remains the
single requirement-consistency check and does not add another reviewer gate.

## Acceptance

- Session routing matches role, concrete agent, and model.
- New runnable metadata matches the fields enforced by the helper scripts.
- Claim, completion, supersession, and worktree rules have one documented
  source of truth.
- No additional reviewer approval is required after consensus is reached.

## Pointers

- `AGENTS.md`
- `docs/agents/README.md`
- `docs/agents/reviews/README.md`
- `docs/agents/eng/README.md`
- `scripts/agent_mailbox.py`
- `scripts/agent_notify.sh`
