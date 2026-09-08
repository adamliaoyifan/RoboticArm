# eng/

Implementation session notes. See [../README.md](../README.md) and the
mandatory [format contract](../FORMAT.md).

Use this role when one concrete agent owns an executable subtask. The same
owner reads the requirement, implements it, runs tests, repairs failures, and
closes the subtask.

Claim assigned work with `scripts/agent_start.sh` before editing. This records
queue wait and prevents stale or dependent subtasks from starting early. Work
only from the thread's exact base and plan revisions. Close passing work with
`scripts/agent_complete.sh` using a real task commit; do not remove mailbox rows
manually and do not report uncommitted worktree state as a passing revision.

```markdown
# YYYY-MM-DD — short title

- role: eng
- agent: cursor | claude-code | codex | custom-agent-id
- model: gpt-5 | opus5 | sonnet | unknown | custom-model-id
- cli: cursor | claude-code | codex | other
- status: done | open
- parent: unified task id
- subtask: assigned subtask id
- base_revision: commit | branch | tag | isolated-worktree-revision
- started_at: ISO-8601 local timestamp
- completed_at: ISO-8601 local timestamp | n/a

## Summary

What changed and why, in one short paragraph.

## Changed

- `src/...`
- `scripts/...`
- `docs/...`

## Verification

- Command run, or "not run" with the reason.

## Requirement

- Owner's concise interpretation and acceptance criteria.

## Result

- pass | blocked, including implementation and test outcome.

## Pointers

- `docs/status/evidence/<run>/`
- `docs/plans/...`

## Open

- Question that needs another role. If present, also add/update a discuss
  thread and `docs/agents/discuss/OPEN.md`.
```

Do not hand routine verification to `test`. Keep fixing within the same
subtask until implementation and required tests pass, or report a genuine
blocker. A separate test note is only for an explicitly assigned independent
audit or integration-evaluation subtask.

If reviews changes accepted scope, dependencies, ownership, or revisions after
Claim, stop the old generation and continue only from its explicitly assigned
higher-generation replacement. Ordinary implementation/test fixes stay in the
same generation.
