# reviews/

Requirements and plan-review session notes. See [../README.md](../README.md).

Use this role when the durable output is scope, acceptance criteria, risk
review, plan review, architecture review, or a decision that changes what
engineering should build.

```markdown
# YYYY-MM-DD — short title

- role: reviews
- agent: cursor | claude-code | codex | custom-agent-id
- model: gpt-5 | opus5 | sonnet | unknown | custom-model-id
- cli: cursor | claude-code | codex | other
- status: done | open

## Summary

Decision, scope, acceptance criteria, or review outcome in one short paragraph.

## Acceptance

- Observable behavior or checks that define done.

## Risks

- Relevant risk, trade-off, or assumption.

## Pointers

- `docs/plans/...`
- `docs/architecture/...`

## Open

- Question that needs another role. If present, also add/update a discuss
  thread and `docs/agents/discuss/OPEN.md`.
```
