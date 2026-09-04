# Agent role logs

Multiple agents (Cursor, Claude Code, Codex, and other CLIs) share this
workspace. Chat transcripts stay in each product. Durable notes go here,
split by **role**, not by model.

Roles are responsibility pools, not unique workers. `eng`, `reviews`, and
`test` may each be handled by more than one concrete agent. Use:

- `role`: responsibility, such as `eng`, `reviews`, or `test`
- `agent`: concrete worker identity, such as `codex`, `claude-code`,
  `cursor`, or `codex-pick-fix`
- `cli`: the CLI product that wrote the note, such as `codex`,
  `claude-code`, or `cursor`

Any CLI that can read and write this tree can talk to the others **asynchronously**
through `discuss/` threads plus the mailbox `discuss/OPEN.md`. There is no
live socket: the other agent only replies the next time it is started and
reads `OPEN.md`.

| Directory | Role | Write when |
|---|---|---|
| `reviews/` | 需求评审 | scope, acceptance, plan review, open questions |
| `eng/` | 研发 | code/config/launch changes and why |
| `test/` | 测试 | eval/sim runs; **pointers** to evidence, not copies |
| `discuss/` | 讨论 / 跨 agent 线程 | architecture talk, trade-offs, questions to other roles |

Normative rules stay in `docs/architecture/` and `.cursor/rules/`.
Raw eval artifacts stay in `docs/status/evidence/<run>/`.
This tree only stores short journals that point at those.

Cross-CLI contract: root `AGENTS.md` (Claude Code also follows `CLAUDE.md`).

## Session start (every CLI)

1. Read `docs/agents/discuss/OPEN.md`.
2. If a row's `to_role` is your role or `any`, and `to_agent` is `any` or
   matches your concrete agent id, open that thread, **append a reply**, then
   update or remove the row.
3. Do not start a parallel thread for the same question.

## Cross-agent thread

New topic: `docs/agents/discuss/YYYY-MM-DD_HHMM_<slug>.md`

Replies **append** to that same file. Never overwrite earlier sections.

```markdown
# YYYY-MM-DD — short title

- status: open | done
- to_role: reviews | eng | test | discuss | any
- to_agent: any | codex | claude-code | cursor | custom-agent-id

## Post -- eng/codex-pick-fix -- 2026-09-04 10:07 -- codex

Question or proposal. One short paragraph.

## Pointers

- `docs/plans/...`

## Open

- Should place planning stay in `world`?

## Reply -- reviews/claude-code -- 2026-09-04 10:20 -- claude-code

Answer. If this closes the question, set `status: done` in the header
and delete the row from `OPEN.md`.
```

Mailbox row in `discuss/OPEN.md`:

| Column | Meaning |
|---|---|
| `id` | `Q-YYYYMMDD-N` |
| `to_role` | role that should answer, or `any` |
| `to_agent` | concrete target agent, or `any` for the role pool |
| `from_role` | role that asked |
| `from_agent` | concrete asking agent |
| `cli` | `cursor` / `claude-code` / `codex` / other |
| `thread` | filename in `discuss/` |
| `question` | one line |

Two agents must not rewrite `OPEN.md` at the same wall-clock moment; keep
the table tiny and re-read it immediately before editing.

## Notification pattern

An agent notifies another role by creating or appending a discuss thread, then
adding one row to `discuss/OPEN.md`. The target role sees that row at its next
session start, opens the thread, appends a reply, then updates or removes the
row. This is asynchronous handoff, not live chat.

Preferred helper:

```bash
scripts/agent_notify.sh \
  --to eng \
  --to-agent any \
  --from test \
  --from-agent cursor-eval \
  --cli codex \
  --slug yolo-regression \
  --question "Eval packing_eval_carryon_n50 regressed at YOLO_NOT_READY." \
  --pointer docs/agents/test/YYYY-MM-DD_HHMM_eval.md \
  --pointer docs/status/evidence/packing_eval_carryon_n50/
```

When running from a satellite git worktree, point the helper at the primary
workspace so the mailbox does not split:

```bash
AGENT_COORD_ROOT=/home/adamliao/work/elfin_humble_ws \
  scripts/agent_notify.sh --to eng --to-agent any --from test --from-agent codex --cli codex --slug ... --question ...
```

Example: a test agent finishes an eval and needs eng to inspect a regression.

1. Write a test note:
   `docs/agents/test/YYYY-MM-DD_HHMM_<eval-slug>.md`
2. Put raw artifacts under:
   `docs/status/evidence/<run>/`
3. Create or append:
   `docs/agents/discuss/YYYY-MM-DD_HHMM_<regression-slug>.md`
4. Re-read `docs/agents/discuss/OPEN.md`, then add:

```markdown
| Q-YYYYMMDD-N | eng | any | test | cursor-eval | cursor | YYYY-MM-DD_HHMM_<regression-slug>.md | Eval <run> regressed; please inspect failure in <one line>. |
```

The eng agent replies in the same thread. If the ask is answered, it removes
the row from `OPEN.md`; if still blocked, it leaves the row with a shorter
current question.

## Parallel worktrees

Use worktrees when two agents need to edit `src/` at the same time. Keep one
branch per task and one worktree per active editing agent:

```bash
git worktree add ../elfin_humble_ws_eng_pick -b agent/eng/pick-fix master
git worktree add ../elfin_humble_ws_test_eval -b agent/test/eval-gate master
```

Code changes live in the task worktree. Cross-agent messages still go through
the primary workspace's `docs/agents/` tree, using `AGENT_COORD_ROOT` or by
running the notification command from the primary workspace. Sim/eval worktrees
must use a distinct `ROS_DOMAIN_ID` and still stop agent-started sims with
`scripts/stop_sim.sh`.

## Other roles (reviews / eng / test)

```text
docs/agents/<role>/YYYY-MM-DD_HHMM_<slug>.md
```

`HHMM` is local time. New file per note. Do not overwrite another agent's file.

```markdown
# YYYY-MM-DD — short title

- role: reviews | eng | test
- agent: codex | claude-code | cursor | custom-agent-id
- cli: cursor | claude-code | codex
- status: done | open

## Summary

One short paragraph.

## Pointers

- `docs/plans/...` or `src/...` or `docs/status/evidence/<run>/`

## Open

- (omit if none; to ping another role, also add a discuss thread + OPEN.md row)
```

## Do not

- Paste chat transcripts or tool dumps.
- Copy `dumps/`, PLY, PNG, or ledger JSONL here.
- Expect a reply in the same turn from a CLI that is not running.
- Stop a sim the user is watching; agent-started sims still use
  `scripts/stop_sim.sh` after the run.
