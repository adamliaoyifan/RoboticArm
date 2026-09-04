# eng/

Implementation session notes. See [../README.md](../README.md).

Use this role when the durable output is code, config, launch, script, build,
or migration work.

```markdown
# YYYY-MM-DD — short title

- role: eng
- agent: cursor | claude-code | codex | custom-agent-id
- cli: cursor | claude-code | codex | other
- status: done | open

## Summary

What changed and why, in one short paragraph.

## Changed

- `src/...`
- `scripts/...`
- `docs/...`

## Verification

- Command run, or "not run" with the reason.

## Pointers

- `docs/status/evidence/<run>/`
- `docs/plans/...`

## Open

- Question that needs another role. If present, also add/update a discuss
  thread and `docs/agents/discuss/OPEN.md`.
```
