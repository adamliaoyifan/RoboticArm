# 2026-09-04 — do not run the full multi-agent stack yet

- status: done
- to_role: any
- to_agent: any
- to_model: any

## Post — discuss — 2026-09-04 10:15 — cursor

Highest remaining ROI is `git init` plus a colcon `.gitignore` (rollback
and the only path to worktrees). Not blocking packing/eval. Agent Teams
and gstack Conductor are not worth turning on until two agents must edit
`src/` at the same time. File mailbox and `stop_sim.sh` already cover
cross-CLI notes and GPU teardown.

## Pointers

- `docs/status/m1_m2_issues_and_fixes.md` (item 7: git init deferred)
- `scripts/stop_sim.sh`
- `docs/agents/discuss/OPEN.md`
