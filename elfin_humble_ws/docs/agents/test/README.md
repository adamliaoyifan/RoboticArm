# test/

Eval/sim session notes. Point at `docs/status/evidence/<run>/`; do not copy
dumps. See [../README.md](../README.md) and the mandatory
[format contract](../FORMAT.md).

Use this role only for a separately assigned audit, release certification,
benchmark, or integration-evaluation subtask. Routine implementation tests
belong to the concrete agent that owns the implementation subtask.

```markdown
# YYYY-MM-DD — short title

- role: test
- agent: cursor | claude-code | codex | custom-agent-id
- model: gpt-5 | opus5 | sonnet | unknown | custom-model-id
- cli: cursor | claude-code | codex | other
- status: done | open
- parent: unified task id
- subtask: assigned audit/integration id
- revision: exact revision tested

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

The test owner closes its assigned audit or integration subtask end to end.
It does not become a mandatory downstream stage for every eng subtask.
Use `scripts/agent_start.sh` and `scripts/agent_complete.sh` for the same
claim/completion lifecycle as an implementation owner.

Legacy checkpoint poller for the already-running platform-free-height baseline:

```bash
scripts/poll_eng_completed.py              # one pass; write test notes + evidence
scripts/poll_eng_completed.py --interval 60
```

Do not use this poller for new owner-closed subtasks. Each completed legacy
part gets `docs/agents/test/YYYY-MM-DD_HHMM_poll-<ckpt>-<slug>.md` and
`docs/status/evidence/eng_poll/<run_id>/`. Skip cache is
`docs/status/evidence/eng_poll/state.json`.
