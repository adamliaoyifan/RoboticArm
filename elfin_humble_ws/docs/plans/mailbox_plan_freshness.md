# Mailbox Plan Freshness Hardening

Date: 2026-09-05

- status: approved
- parent: MPF-20260905
- source: `docs/agents/reviews/2026-09-05_1455_mailbox-plan-freshness.md`

## Objective

Guarantee that an agent can pass only the currently approved generation of an
owner-assigned mailbox task. A queued prompt, active claim, or previously
passing dependency must not remain authoritative after its plan is superseded
or cancelled.

The mailbox remains Markdown and file based. This work does not introduce a
second task database or attempt to kill arbitrary terminal processes.

## Required Semantics

### Identity and authority

- Runnable task identity is `(parent, subtask, generation)`.
- `generation` is a positive integer, monotonically increasing within one
  `(parent, subtask)` lineage. Generation reuse and rollback are invalid.
- `plan_revision` identifies the exact Git commit containing the approved plan
  and acceptance criteria. It is distinct from `revision`, which remains the
  implementation base revision.
- Runnable `OPEN.md` rows and runnable thread metadata carry both fields.
- The highest generation in a lineage is authoritative. At most one thread in
  that lineage may have `status: open`.
- A claimed runnable thread is not edited in place to change scope,
  acceptance, owner, dependencies, generation, or plan revision. Such a change
  creates a replacement generation and supersedes the old thread.

### Lifecycle states

Runnable threads support:

- `open`: may be queued or claimed;
- `done`: current generation completed with a passing Result;
- `superseded`: replaced by a named higher generation;
- `cancelled`: intentionally stopped without replacement.

Supersede/cancel transitions are atomic under the mailbox lock and append an
event with actor, timestamp, reason, old generation, and replacement pointer
when applicable. Historical Claim and Result events remain immutable.

### Claim and completion

- `agent_start.sh` records the claimed `generation`, `plan_revision`, and the
  resolved generation of every dependency.
- Before recording a passing Result, `agent_complete.sh` re-reads the mailbox
  under lock and rejects closure when:
  - the thread is not the highest generation for its lineage;
  - thread and `OPEN.md` generation/plan revision differ;
  - Claim generation/plan revision differ from current metadata;
  - any claimed dependency generation is no longer current and passing;
  - the thread is `superseded` or `cancelled`.
- A stale completion must exit nonzero with a stable reason and must not append
  a passing Result, set `status: done`, or release dependents.

### Owner stop notification

- A superseded/cancelled claimed row remains visible as a stop item until its
  owner acknowledges it; the replacement is a separate row/thread.
- `agent_poll_self.py` reports stop items before active or ready work, names the
  replacement when present, and does not auto-claim the replacement while an
  unacknowledged stop exists for the same owner.
- `agent_scheduler.py` sends a stop message through supported live dispatch
  adapters and records an idempotent notice lease. File-only sessions see the
  same stop item on their next poll.
- A small owner acknowledgement command appends a StopAck and removes only the
  terminal row from `OPEN.md`. It cannot make the old generation runnable.
- Hard completion rejection is the safety boundary; stop delivery is
  best-effort because arbitrary external terminals cannot be forcibly killed.

### Dependency freshness

- Dependency resolution selects the highest generation of the dependency
  lineage, never the first matching thread on disk.
- A dependency passes only when that highest generation is `done` with a
  passing Result.
- A dependent claim snapshots dependency generations. If an upstream task is
  later superseded or cancelled, completion of the dependent is stale and
  rejected even if an older upstream generation passed.

### Compatibility and migration

- Questions remain exempt from generation and plan-revision requirements.
- Existing runnable rows/threads are migrated once under the mailbox lock.
  Legacy parsing may default missing generation to `1` only during migration;
  newly created runnable work must provide the new fields.
- Migration is dry-run capable, idempotent, preserves thread history, and
  refuses ambiguous duplicate lineages or a non-resolvable approved plan
  revision instead of guessing.
- `check_agent_contract.sh` rejects new-schema drift, duplicate current
  generations, invalid terminal transitions, and mismatched row/thread fields.

## Subtasks

All subtasks use the same concrete owner because lifecycle behavior spans the
shell helpers and Python scheduler/poller. They run sequentially to avoid two
agents changing the mailbox schema at once.

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| MPF-1 | `codex/gpt-5` | none | Generation/plan-revision schema, atomic supersede/cancel/ack lifecycle, claim snapshots, completion rejection, migration | A stale or terminal claim can never append a passing Result or release dependents | Focused temp-repo tests for new, claimed, superseded, cancelled, migrated, malformed, and concurrent transitions |
| MPF-2 | `codex/gpt-5` | MPF-1 | Scheduler/poller stop reporting, live stop dispatch, highest-generation dependency resolution | Stop precedes replacement work; old dependency generations never satisfy or close downstream work | Scheduler/poller unit or temp-repo tests covering live/file owners, idempotency, replacement ordering, and upstream replan after downstream claim |
| MPF-INTEGRATION | `codex/gpt-5` | MPF-1,MPF-2 | Migrate current mailbox, repair runtime-registry table insertion, update contract/docs, run whole lifecycle regression | Current mailbox and registry pass the new contract; legacy migration is demonstrated; ordinary owner flow still works | Full lifecycle smoke including documented registry files, contract check, Python compile/tests, `git diff --check` |

## File Boundaries

Expected owned surface:

- `scripts/agent_notify.sh`
- `scripts/agent_start.sh`
- `scripts/agent_complete.sh`
- `scripts/agent_scheduler.py`
- `scripts/agent_poll_self.py`
- `scripts/agent_register.sh`
- lifecycle transition/ack/migration helpers and shared parser if introduced;
- `scripts/test_agent_lifecycle_smoke.sh` and focused mailbox tests;
- `scripts/check_agent_contract.sh`
- `docs/agents/README.md`, role templates, `AGENTS.md`, and this plan only
  where needed to document the implemented contract;
- `docs/agents/discuss/OPEN.md` and existing open thread metadata only through
  the reviewed migration.
- `docs/agents/RUNTIME.md` only through registration and migration fixtures or
  the operational row updates needed by the task.

Do not modify robotics algorithms, simulation code, unrelated agent notes, or
historical thread event bodies.

## Acceptance Scenarios

1. Generation 1 is queued, claimed, then superseded by generation 2. Generation
   1 completion fails; its owner sees stop; generation 2 remains claimable only
   after stop acknowledgement for that owner.
2. A queued but unclaimed generation is superseded before dispatch. Scheduler
   never sends the stale owner prompt.
3. A claimed task is cancelled. Poller/scheduler report stop and completion
   cannot pass it.
4. Dependency A1 passes; B1 claims against A1; A2 then supersedes A1. B1
   completion is rejected until a current B generation is based on passing A2.
5. Duplicate generation, generation rollback, row/thread mismatch, invalid
   plan revision, and ambiguous migration all fail closed with stable reasons.
6. Stop delivery and acknowledgement are idempotent across repeated scheduler
   and poller runs.
7. A normal unsuperseded task still follows notify -> start -> complete and
   releases its dependent.
8. Existing mailbox history remains readable and no historical Claim/Result
   event is rewritten.
9. Registering a new or existing session keeps every runtime row inside the
   registry Markdown table even when explanatory text follows the table;
   repeated registration updates one row without appending rows after prose.

## Dispatch Gate

No implementation row is runnable until:

1. a distinct `codex/gpt-5` records `consensus: reached` on this requirement;
2. any amendments are incorporated here;
3. the approved plan text has an exact Git revision;
4. MPF-1 is dispatched with that exact plan revision.

## Consensus

- reviewer: `codex/gpt-5`
- session: `01a06a53-6433-7180-ae5b-c7963c8f1e26`
- thread: `docs/agents/discuss/2026-09-05_1505_mailbox-plan-freshness-consensus.md`
- result: reached on plan SHA-256
  `585a007cefc8c2c64a16d2ded7f0004825fdb075482f5e39af850dbe50015c6c`
