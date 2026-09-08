# reviews/

Requirements and plan-review session notes. See [../README.md](../README.md)
and the mandatory [format contract](../FORMAT.md).

Use this role to clarify a unified requirement with Codex, reach consensus,
decompose it into bounded executable subtasks, and assign concrete owners.
Reviews does not implement or test the dispatched subtasks.

The distinct Codex consultation is the single requirement-consistency check,
not an extra approval tier. After `consensus: reached`, reviews may finalize
and dispatch directly. Do not wait for a second reviewer unless the user
explicitly requests an independent audit.

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

## Consensus

- Codex agent: distinct concrete Codex agent id
- Thread: `docs/agents/discuss/...`
- Result: reached | open

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| ST-1 | concrete owner | none | bounded behavior | observable result | owner-run tests |
| INTEGRATION | concrete owner | ST-1,... | whole chain | unified acceptance | E2E |

## Risks

- Relevant risk, trade-off, or assumption.

## Pointers

- `docs/plans/...`
- `docs/architecture/...`

## Open

- Question that needs another role. If present, also add/update a discuss
  thread and `docs/agents/discuss/OPEN.md`.
```

Do not dispatch subtasks until Codex consensus is recorded. Each runnable row
must have a concrete agent/model owner, base revision, positive generation,
exact plan revision, explicit `dispatch_ready`, and enough acceptance/test
context for that owner to close it without a routine role handoff. Once claimed,
change scope or dependencies only by superseding it with a higher generation.
