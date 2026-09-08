# Agent contract (all CLIs)

This workspace is used by several CLI agents at once (Cursor, Claude Code,
Codex, others). Follow these files; do not invent a parallel log tree.

1. **Role notes and workflow** — `docs/agents/README.md`
2. **Exact document format** — `docs/agents/FORMAT.md`
3. **Mailbox** — `docs/agents/discuss/OPEN.md` (read at session start)
4. **Sim teardown** — `.cursor/rules/sim-lifecycle.mdc` and `scripts/stop_sim.sh`
5. **Architecture** — `docs/architecture/` (normative; violations are defects)

Cross-agent discussion is **file-based and asynchronous**: post or reply in
`docs/agents/discuss/<thread>.md`, list unanswered asks and runnable handoffs
in `OPEN.md`. The other CLI only acts the next time it runs. There is no live
RPC unless a separate scheduler controls a registered CLI session.

The scheduler entry point is `scripts/agent_scheduler.py`. It watches
`OPEN.md` and matches rows to registered sessions in `docs/agents/RUNTIME.md`.
Already-running CLIs are only directly schedulable when that registry row has
a fresh heartbeat, `state=idle`, and a supported dispatch capability such as
Codex `queue`; otherwise the task remains file-based until the agent next
reads `OPEN.md`. File-only sessions can poll matching rows with
`scripts/agent_poll_self.py` and keep a local queue. Register or refresh a
running CLI with `scripts/agent_register.sh`.

Roles are responsibility pools, not unique workers. `eng`, `reviews`, and
`test` may each be served by multiple concrete agents such as `codex`,
`claude-code`, or `cursor`; use `agent` / `to_agent` / `from_agent` fields to
identify the concrete worker when it matters. Also record `model` /
`to_model` / `from_model` when the model choice matters; for example, Cursor
running Opus5 may serve both `reviews` and `eng`.

When using satellite git worktrees for code isolation, keep cross-agent
coordination in the primary workspace's `docs/agents/` tree. Do not create a
second mailbox per worktree.

`reviews` only clarifies and decomposes unified requirements. Before dispatch,
it must discuss the requirement with a distinct Codex agent and record
consensus. This is the requirement-consistency check; it does not require a
second reviewer approval after consensus is reached. Each resulting subtask has
a concrete agent/model owner, generation, exact plan revision, and explicit
`dispatch_ready` state. That owner reads the requirement, implements, tests,
fixes failures, commits, records evidence, and closes the subtask end to end.
Claim runnable work with `scripts/agent_start.sh`; close it with
`scripts/agent_complete.sh`. Routine subtasks do not hand testing to a separate
test role. The plan includes a final integration subtask when the unified task
needs whole-chain regression.

The discuss thread is authoritative for task lifecycle; `OPEN.md` is only the
current queue projection and `RUNTIME.md` is only transient scheduler presence.
Do not rewrite claimed requirements in place. Supersede them with a higher
generation in the same parent/subtask lineage.

If you produce a decision, code change, plan, or eval, write a role note.
Skip ack-only turns. Test evidence stays under `docs/status/evidence/`.
