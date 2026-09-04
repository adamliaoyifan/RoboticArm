# test/

Eval/sim session notes. Point at `docs/status/evidence/<run>/`; do not copy
dumps. See [../README.md](../README.md).

Use this role when the durable output is test execution, eval results, sim
runs, regression triage, or evidence indexing.

```markdown
# YYYY-MM-DD — short title

- role: test
- agent: cursor | claude-code | codex | custom-agent-id
- cli: cursor | claude-code | codex | other
- status: done | open

## Summary

What was run and the pass/fail result in one short paragraph.

## Commands

- Exact command or launch invocation.

## Evidence

- `docs/status/evidence/<run>/`
- `docs/status/<summary>.md`

## Result

- pass | fail | inconclusive, with one-line reason.

## Open

- Question that needs another role. If present, also add/update a discuss
  thread and `docs/agents/discuss/OPEN.md`.
```
