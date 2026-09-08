# Agent runtime registry

This file is the scheduler-facing registry for already-running CLI sessions.
It is optional for normal asynchronous mailbox use. A session is schedulable
only when it appears here, has a fresh heartbeat, is `idle`, and declares a
supported live dispatch capability.

The scheduler does not dispatch an already-claimed thread again. It treats a
thread with a `Claim` event and no passing `Result` as still owned by the
claiming agent.

Refresh a row from the running CLI:

```bash
scripts/agent_register.sh --id codex-eng-a --role eng --agent codex-a \
  --model gpt-5 --cli codex --session <session> --capabilities queue
```

| id | role | agent | model | cli | session | worktree | state | capabilities | heartbeat |
|---|---|---|---|---|---|---|---|---|---|
| cursor-test-grok | test | cursor | grok-4.6 | cursor | 0099ca8b-111a-4f89-b3c4-e667f9b34e3f | /home/adamliao/work/elfin_humble_ws | idle | file | 2026-09-07T18:29:28+08:00 |
| cursor-eng-grok-b | eng | cursor-grok-b | grok-4.6 | cursor | 4b3f539b-0a3a-4c74-8daa-f3b43ce5f208 | /home/adamliao/work/elfin_humble_ws | idle | file | 2026-09-05T17:11:47+08:00 |

Capability notes:

- `queue`: the scheduler may send a live message through a supported adapter.
  Currently implemented for `cli=codex` via `codex queue --thread`.
- `file`: the agent reads `docs/agents/discuss/OPEN.md` on its next normal
  turn, or polls matching rows with `scripts/agent_poll_self.py`. The
  scheduler reports these as `file-only`; it does not claim a lease.

Unknown terminals, active Claude/Cursor sessions without a safe message API,
and stale heartbeats remain file-based. Do not inject keystrokes into an
arbitrary terminal.
| codex-reviews-gpt5-01a06a53 | reviews | codex | gpt-5 | codex | 01a06a53-6433-7180-ae5b-c7963c8f1e26 | /home/adamliao/work/elfin_humble_ws | idle | queue | 2026-09-05T15:08:56+08:00 |
| claude-eng-pfr5 | eng | claude | glm-5.3 | claude-code | 4a4943b4-4405-44ac-b4a1-18bbf73ba7d7 | /home/adamliao/work/elfin_humble_ws | idle | file | 2026-09-05T19:57:54+08:00 |
| codex-eng-mpf | eng | codex | gpt-5 | codex | 01a06a53-6433-7180-ae5b-c7963c8f1e26 | /home/adamliao/work/elfin_humble_ws | idle | queue | 2026-09-05T15:18:21+08:00 |
| codex-reviews-main-gpt56sol | reviews | codex-reviews-main | gpt-5.6-sol | codex | 01a06a0a-e91e-7c42-bdf1-2b582f91dd04 | /home/adamliao/work/elfin_humble_ws | idle | queue | 2026-09-05T18:03:10+08:00 |
