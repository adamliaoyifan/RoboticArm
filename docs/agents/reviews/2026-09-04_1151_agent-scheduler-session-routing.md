# 2026-09-04 -- Agent scheduler session routing

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

An already-running CLI is schedulable only when it is registered with a
stable session id and exposes a supported message or control interface. The
scheduler must use a capability adapter per CLI and select one of `queue`,
`resume`, or `spawn`; it must not inject keystrokes into arbitrary terminals.

Codex 0.153.2 supports queuing a message to an existing session with
`codex queue --thread <id-or-name> --message <text>`. Claude Code and Cursor
support session resume and non-interactive execution, but concurrent resume of
a session that is already active is not treated as a safe live-message API.
For those sessions, dispatch only through a managed background/persistent
worker interface when available; otherwise wait until idle or spawn a new
worker.

The listener/scheduler should maintain a runtime registry containing agent,
role, model, CLI, session id, PID, worktree, state, heartbeat, and dispatch
capabilities. Claim mailbox work with a lease before delivery, enforce one
active task per session, and use a new worktree plus resource isolation when
spawning parallel engineering or test workers.

## Pointers

- `docs/agents/README.md`
- `docs/agents/discuss/OPEN.md`
- `scripts/agent_notify.sh`

## Open

- Implement the listener, runtime registry, CLI adapters, leases, and scheduler
  as a separate engineering task.
