# Agent contract (all CLIs)

This workspace is used by several CLI agents at once (Cursor, Claude Code,
Codex, others). Follow these files; do not invent a parallel log tree.

1. **Role notes** — `docs/agents/README.md`
2. **Mailbox** — `docs/agents/discuss/OPEN.md` (read at session start)
3. **Sim teardown** — `.cursor/rules/sim-lifecycle.mdc` and `scripts/stop_sim.sh`
4. **Architecture** — `docs/architecture/` (normative; violations are defects)

Cross-agent discussion is **file-based and asynchronous**: post or reply in
`docs/agents/discuss/<thread>.md`, list unanswered asks in `OPEN.md`. The
other CLI only answers the next time it runs. There is no live RPC.

Roles are responsibility pools, not unique workers. `eng`, `reviews`, and
`test` may each be served by multiple concrete agents such as `codex`,
`claude-code`, or `cursor`; use `agent` / `to_agent` / `from_agent` fields to
identify the concrete worker when it matters. Also record `model` /
`to_model` / `from_model` when the model choice matters; for example, Cursor
running Opus5 may serve both `reviews` and `eng`.

When using satellite git worktrees for code isolation, keep cross-agent
coordination in the primary workspace's `docs/agents/` tree. Do not create a
second mailbox per worktree.

If you produce a decision, code change, plan, or eval, write a role note.
Skip ack-only turns. Test evidence stays under `docs/status/evidence/`.
