# Claude Code

Follow [AGENTS.md](AGENTS.md). Role logs and the cross-CLI mailbox are
`docs/agents/` (see `docs/agents/discuss/OPEN.md` at session start).

## Token discipline (mandatory)

Every turn re-sends accumulated history, so each large tool result is paid
for again on every later turn. Cap what each call adds:

- **Test/build output — never let a full suite land in the transcript.**
  Run `pytest -q --tb=line 2>&1 | tail -40` (`-k`/file selection for
  targeted runs; re-run only the failing test with `--tb=short` when
  diagnosis needs the traceback). For colcon, use
  `--event-handlers console_direct-` or pipe through
  `2>&1 | grep -E "Failed|error|Summary" | tail -40`. Full logs go to a
  file under `docs/status/`; show only the digest.
- **Large files — never whole-read 2,000+ line sources.** Grep/Glob to
  locate first, then `Read` with `offset`/`limit` on the relevant range.
  No speculative full-file reads.
