# Agent role logs

Exact filenames, metadata, required sections, event blocks, and mailbox tables
are normative in [`FORMAT.md`](FORMAT.md). Every agent MUST validate new notes
against that document before reporting completion.

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
- `model`: model identity, such as `gpt-5`, `opus5`, `sonnet`, or `unknown`

`reviews` is the canonical role name for review work. `reviewers` is accepted
as a human-facing alias by helper scripts, but files and mailbox rows use
`reviews`.

## Role Eligibility

Default capability is role-based: any concrete agent may serve a role if the
user or mailbox targets that role and the agent is safe for the task. Model
identity refines that routing when needed. Current local rule:

- `cli=cursor`, `model=opus5`: may serve `reviews` and `eng`

Roles classify durable notes; they are not pipeline stages. A concrete agent
assigned a subtask owns its complete loop: read and clarify the requirement,
implement it, run proportionate tests, fix failures, record evidence, and close
the subtask. Routine work does not pass from eng to a separate test role.

## Communication state and precedence

Use one communication system only. Its records have distinct purposes:

1. `docs/architecture/` is normative for product behavior and system
   boundaries.
2. A `discuss/<thread>.md` file is the durable source of truth for one
   question or work-item lineage: scope, owner, generation, claims, replies,
   results, and supersession history live there.
3. `discuss/OPEN.md` is only the current queue projection. It contains work
   that still needs action; removing a row does not prove completion.
4. `reviews/`, `eng/`, and `test/` notes summarize decisions or work and point
   to the thread, commit, and evidence. They do not replace the thread.
5. `docs/agents/RUNTIME.md` is transient scheduler presence. Registration does
   not claim work and is not evidence that a task ran.

When records disagree, architecture wins for product constraints; for task
lifecycle, the thread metadata and its latest valid event win over `OPEN.md`
or a role-note summary. A passing task requires `status: done` and a matching
`## Result` with `outcome: pass`; absence from `OPEN.md` is insufficient.

Any CLI that can read and write this tree can talk to the others **asynchronously**
through `discuss/` threads plus the mailbox `discuss/OPEN.md`. There is no
live socket: the other agent only acts the next time it is started and reads
`OPEN.md`, unless a separate scheduler controls a registered CLI session.

`scripts/agent_scheduler.py` is that scheduler entry point. It reads
`docs/agents/RUNTIME.md`, checks owner/model/role, dependency completion,
session state, heartbeat freshness, and dispatch capability. By default it
prints a dry-run plan. With `--dispatch`, it creates a lease and sends work
only through supported adapters; currently that means `cli=codex` with
`queue`. Cursor or Claude sessions without an explicit safe adapter remain
file-only. A thread that already has a `Claim` event is not dispatched again.
A running CLI refreshes its registry row with `scripts/agent_register.sh`.

| Directory | Role | Write when |
|---|---|---|
| `reviews/` | 需求评审 | scope, acceptance, plan review, open questions |
| `eng/` | 研发 | code/config/launch changes and why |
| `test/` | 测试 | eval/sim runs; **pointers** to evidence, not copies |
| `discuss/` | 讨论 / 跨 agent 线程 | architecture talk, decisions, questions, and runnable handoffs |

Normative rules stay in `docs/architecture/` and `.cursor/rules/`.
Raw eval artifacts stay in `docs/status/evidence/<run>/`.
This tree only stores short journals that point at those.

Cross-CLI contract: root `AGENTS.md` (Claude Code also follows `CLAUDE.md`).

## Session start (every CLI)

1. Read `docs/agents/discuss/OPEN.md`.
2. Match all routing fields: `to_role`, `to_agent`, and `to_model`. `cli` is
   transport metadata, not owner identity. Do not claim a row merely because
   its role matches.
3. For runnable `subtask`, `integration`, or `regression` work, call
   `scripts/agent_start.sh` before editing. The helper checks ownership,
   `dispatch_ready`, generation freshness, and dependencies. The same owner
   implements, tests, repairs, commits, records evidence, and closes the work.
4. For `question` or `consensus`, append a `## Reply` to the existing thread;
   do not use the runnable claim/completion lifecycle.
5. File-only CLIs may poll their own exact route with
   `scripts/agent_poll_self.py`. A scheduler may dispatch only a registered,
   fresh, idle session with a supported adapter.
6. Do not open a parallel thread for the same generation of a work-item
   lineage. Follow the rework rules below when requirements change.

## Cross-agent thread

New topic: `docs/agents/discuss/YYYY-MM-DD_HHMM_<slug>.md`

Replies **append** to that same file. Never overwrite earlier sections.

```markdown
# YYYY-MM-DD — short title

- status: open | done
- to_role: reviews | eng | test | discuss | any
- to_agent: any | codex | claude-code | cursor | custom-agent-id
- to_model: any | gpt-5 | opus5 | sonnet | unknown | custom-model-id
- kind: question | consensus | subtask | integration | regression
- parent: unified-task-id | n/a
- subtask: ST-1 | INTEGRATION | n/a
- depends_on: none | ST-1 | ST-1,ST-2
- revision: n/a | git-commit | branch | tag | isolated-worktree-revision
- generation: positive integer (runnable kinds only)
- plan_revision: exact approved plan commit (runnable kinds only)
- dispatch_ready: yes | no (runnable kinds only)
- consensus: open | reached (consensus only)

## Post -- eng/codex-pick-fix -- 2026-09-04 10:07 -- codex/gpt-5

Question or proposal. One short paragraph.

## Pointers

- `docs/plans/...`

## Open

- Should place planning stay in `world`?

## Reply -- reviews/claude-code -- 2026-09-04 10:20 -- claude-code/opus5

Answer. If this closes the question, set `status: done` in the header
and delete the row from `OPEN.md`.
```

Mailbox row in `discuss/OPEN.md`:

| Column | Meaning |
|---|---|
| `id` | `Q-YYYYMMDD-N` |
| `kind` | `question`, `consensus`, `subtask`, `integration`, or `regression` |
| `parent` | unified task id shared by all subtasks |
| `subtask` | executable subtask id, or `n/a` |
| `depends_on` | prerequisite subtask ids in the same parent, or `none` |
| `revision` | reproducible requirement/base revision, or `n/a` for questions |
| `to_role` | role that should answer, or `any` |
| `to_agent` | concrete target agent, or `any` for the role pool |
| `to_model` | model target, or `any` for any model |
| `from_role` | role that asked |
| `from_agent` | concrete asking agent |
| `from_model` | model that created the ask |
| `cli` | `cursor` / `claude-code` / `codex` / other |
| `thread` | filename in `discuss/` |
| `request` | one-line request or expected outcome |
| `generation` | positive lineage generation for runnable work; blank for questions |
| `plan_revision` | exact approved plan commit for runnable work; blank for questions |

`dispatch_ready` deliberately lives in thread metadata rather than as a
mailbox column. The thread is authoritative; pollers and the scheduler read it
before offering or dispatching work.

Use `agent_notify.sh`, `agent_start.sh`, and `agent_complete.sh` for mailbox
mutations whenever their lifecycle applies; they share the mailbox lock. If a
question or consensus must be closed manually, take the same lock, re-read the
thread and `OPEN.md` after acquiring it, append the reply, update thread
metadata, and remove only that thread's row. Never replace the whole mailbox
from a stale copy.

All newly created runnable `subtask`, `integration`, and `regression` rows
require a parent, subtask id, base revision, positive generation, exact plan
revision, concrete target agent/model, and explicit `dispatch_ready`. Questions
may use `n/a`. `task` and `checkpoint` remain parser-compatible only for open
legacy work and must not be created for new plans. Missing freshness fields are
accepted only while migrating existing legacy rows.

## Unified task workflow

Reviews has only two responsibilities: reach requirement consensus with a
distinct Codex agent, then split the unified task into executable subtasks and
assign each one. That Codex exchange is a requirement-consistency check, not a
second approval board. Once the thread records `consensus: reached`, reviews
may finalize the plan and dispatch directly; no additional reviewer approval
is required. Reviews does not prescribe implementation steps and does not
execute the subtasks.

1. Reviews reads the unified requirement and drafts scope, constraints,
   acceptance criteria, architecture impact, and a proposed decomposition.
2. Reviews opens `kind=consensus` to a distinct Codex agent. Both sides iterate
   in one thread until ambiguities are resolved and the thread records
   `consensus: reached`.
3. Reviews writes one plan with a subtask table. Every row has a concrete owner
   agent/model, dependency list, bounded file/behavior scope, acceptance
   criteria, required tests, base revision, and exact plan revision.
4. Reviews dispatches one `kind=subtask` row per ready unit. Dependent rows may
   be listed early, but a scheduler must not run them until `depends_on` is
   complete. Reviews sets `dispatch_ready: yes` only after the row and thread
   contain everything the owner needs to finish without another clarification.
5. The assigned agent reads the requirement and architecture, makes its own
   implementation decisions inside scope, implements, tests, fixes failures,
   writes evidence, and closes the row. It does not hand routine testing to a
   test agent.
6. Reviews includes a final `kind=integration` subtask that depends on all
   implementation subtasks. Its owner resolves integration failures and runs
   the full end-to-end regression.

Dependency completion means a thread with the same `parent` and dependency
`subtask` has `status: done`, a `## Result` event, and `outcome: pass`. Absence
from `OPEN.md` alone is not completion.

Use an independent reviews/test agent only for explicitly requested audit,
safety, or release certification. That audit is an additional subtask, not a
mandatory stage in every implementation loop.

## Freshness, rework, and cancellation

`parent` plus `subtask` identifies a work-item lineage. `generation` identifies
one immutable attempt within that lineage.

- Never rewrite the accepted scope, owner, base revision, plan revision, or
  dependencies of claimed work in place.
- If those inputs change, create a replacement thread for the same
  `parent/subtask` with `generation` incremented by one. Mark the older thread
  `superseded` and point it to the replacement.
- Only the highest generation may be newly claimed or completed as passing.
- A Claim records `claimed_generation`, `claimed_plan_revision`, and the
  generation of each satisfied dependency. Completion fails if any snapshot
  is stale.
- Use `cancelled` when no replacement will run. A claimed owner must acknowledge
  a supersede/cancel stop before its stale mailbox row is removed.
- A failed test is not a new generation by itself. The same owner fixes and
  retests until scope or accepted requirements actually change.

Subtask plan table:

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| ST-1 | `codex-a/gpt-5` | none | one bounded behavior | observable result | focused + regression |
| ST-2 | `cursor-b/opus5` | none | independent behavior | observable result | focused + regression |
| INTEGRATION | `claude-c/opus5` | ST-1,ST-2 | merged workflow | full acceptance | end-to-end suite |

Preferred subtask dispatch:

```bash
scripts/agent_notify.sh \
  --kind subtask \
  --parent TASK-20260904-1 \
  --subtask ST-2 \
  --depends-on none \
  --base-revision 4c0ffee \
  --generation 1 \
  --plan-revision 8badf00 \
  --dispatch-ready yes \
  --to eng \
  --to-agent cursor-opus5-b \
  --to-model opus5 \
  --from reviews \
  --from-agent reviewer-a \
  --from-model opus5 \
  --cli cursor \
  --slug task-1-st-2 \
  --request "Implement and test ST-2 to the plan acceptance criteria." \
  --pointer docs/plans/task-1.md
```

Register an already-running CLI session for scheduler visibility:

```bash
scripts/agent_register.sh \
  --id codex-eng-a \
  --role eng \
  --agent codex-a \
  --model gpt-5 \
  --cli codex \
  --session <codex-session-id-or-name> \
  --state idle \
  --capabilities queue
```

Preview ready work:

```bash
scripts/agent_scheduler.py
```

Poll and queue mailbox rows that belong to this concrete agent/model
(including questions). Ready runnable rows are next; blocked rows stay queued
until their `depends_on` Results pass. `to_agent=any` plus `to_model=any`
broadcast rows are ignored unless `--include-broadcast`.

For newly created runnable rows, reviewers must mark the thread metadata
`dispatch_ready: yes` only after the detailed subtask instructions, pointers,
owner, acceptance, required tests, generation, and exact plan revision are
complete. `dispatch_ready: no` rows remain visible as waiting work and cannot
be claimed by `agent_start.sh`, `--claim-next`, or live scheduler dispatch.
Legacy rows without the field remain compatible only until mailbox migration
is complete; never omit it on new work.

```bash
scripts/agent_poll_self.py --agent cursor --model grok-4.6 --json
scripts/agent_poll_self.py --agent cursor --model grok-4.6 \
  --watch --interval 45 --refresh-register-id cursor-test-grok
scripts/agent_poll_self.py --agent codex --model gpt-5 --cli codex \
  --claim-next
scripts/agent_poll_self.py --agent codex --model gpt-5 --cli codex \
  --watch --interval 45 --claim-next
```

Continuously dispatch through supported live adapters:

```bash
scripts/agent_scheduler.py --watch --dispatch
```

The assigned owner closes the whole subtask. A failure discovered during its
tests stays with the same owner until fixed or genuinely blocked.

When work starts, use the claim helper:

```bash
scripts/agent_start.sh \
  --thread YYYY-MM-DD_HHMM_task-1-st-1.md \
  --role eng \
  --agent codex-a \
  --model gpt-5 \
  --cli codex
```

It verifies concrete ownership, the open mailbox row, and passing dependency
Results before appending:

```markdown
## Claim -- eng/codex-a -- 2026-09-04 15:00 -- codex/gpt-5

- started_at: 2026-09-04T15:00:00+08:00
```

When implementation and required tests pass, append the result, set thread
`status: done`, and remove its mailbox row:

```markdown
## Result -- eng/codex-a -- 2026-09-04 15:42 -- codex/gpt-5

- outcome: pass
- completed_at: 2026-09-04T15:42:00+08:00
- revision: 8badf00
- tests: focused and affected regression suites pass
- evidence: docs/status/evidence/<run>/
```

Preferred completion helper:

```bash
scripts/agent_complete.sh \
  --thread YYYY-MM-DD_HHMM_task-1-st-1.md \
  --role eng \
  --agent codex-a \
  --model gpt-5 \
  --cli codex \
  --outcome pass \
  --revision 8badf00 \
  --tests "focused and affected regression suites pass" \
  --summary "ST-1 acceptance criteria satisfied" \
  --evidence docs/status/evidence/<run>/
```

The helper verifies that role/agent/model match the assigned owner and that a
pass revision resolves to a real Git commit. Uncommitted worktree state cannot
close a task. It also requires a Claim event, verifies generation and dependency
freshness, appends `Result`, marks the thread done, and removes the mailbox row.
Use `--started-at <ISO-8601>` only to migrate work that began before
`agent_start.sh` existed.

For a genuine blocker, leave the row open and record `outcome: blocked` plus
the exact missing decision or external dependency. Do not report ordinary test
failures as blockers; the owner continues fixing them.

When running from a satellite git worktree, point the helper at the primary
workspace so the mailbox does not split:

```bash
AGENT_COORD_ROOT=/home/adamliao/work/elfin_humble_ws \
  scripts/agent_notify.sh --kind subtask --parent TASK-1 --subtask ST-1 --depends-on none --base-revision 4c0ffee --to eng --to-agent codex-a --to-model gpt-5 --from reviews --from-agent cursor-review --from-model opus5 --cli cursor --slug ... --request ...
```

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

Worktrees share Git objects and refs but have separate indexes and working
directories. Record the exact base commit in the thread, commit task changes on
the task branch, and report that commit in `Result`. Do not infer completion
from files visible in another worktree, and do not create a second mailbox in a
satellite worktree.

## Other roles (reviews / eng / test)

```text
docs/agents/<role>/YYYY-MM-DD_HHMM_<slug>.md
```

`HHMM` is local time. New file per note. Do not overwrite another agent's file.

```markdown
# YYYY-MM-DD — short title

- role: reviews | eng | test
- agent: codex | claude-code | cursor | custom-agent-id
- model: gpt-5 | opus5 | sonnet | unknown | custom-model-id
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
