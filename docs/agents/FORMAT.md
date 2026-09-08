# Agent document format contract

This file defines the mandatory file and Markdown formats used by every CLI
agent in this workspace. `MUST`, `MUST NOT`, `SHOULD`, and `MAY` are normative.
Workflow semantics are defined in `docs/agents/README.md`; this document defines
how those semantics are encoded.

## 1. General syntax

1. Files MUST be UTF-8 Markdown with LF line endings.
2. A document MUST start with exactly one H1 title.
3. Metadata MUST immediately follow the H1, with one blank line before it.
4. Each metadata field MUST use exactly `- key: value` on one physical line.
5. Metadata keys MUST NOT be duplicated.
6. Required section headings MUST match this document exactly, including case.
7. Paths, revisions, commands, topic names, and identifiers SHOULD use
   backticks in prose and lists.
8. Raw logs, images, PLY files, JSONL streams, and chat transcripts MUST NOT be
   stored under `docs/agents/`; put evidence under `docs/status/evidence/`.
9. A Markdown table cell MUST NOT contain a literal `|` or newline. Use
   `agent_notify.sh`, which escapes mailbox cells.
10. Timestamps MUST include a date and time. Lifecycle timestamps MUST use an
    ISO-8601 UTC offset, for example `2026-09-05T17:30:00+08:00`.

## 2. Identifiers and values

| Field | Required format |
|---|---|
| `role` | `reviews`, `eng`, or `test` in role notes; `discuss` and `any` are routing-only values |
| `agent` | Stable concrete worker id, for example `codex`, `codex-reviews-main`, `claude`, or `cursor-grok-b` |
| `model` | Exact model route, for example `gpt-5`, `glm-5.3`, or `grok-4.6` |
| `cli` | CLI product: `codex`, `claude-code`, `cursor`, or another explicit product id |
| `status` | Role note: `open` or `done`; thread: `open`, `done`, `superseded`, or `cancelled` |
| `kind` | New work: `question`, `consensus`, `subtask`, `integration`, or `regression` |
| `parent` | Stable unified-task id, or `n/a` only for a question |
| `subtask` | Stable executable id, or `n/a` for question/consensus |
| `depends_on` | `none` or comma-separated subtask ids from the same parent |
| `revision` | Exact base commit preferred; `n/a` only where explicitly allowed |
| `base_revision` | Role-note name for the exact code base used by an eng task |
| `generation` | Positive integer; starts at `1` and increases for replacement attempts |
| `plan_revision` | Exact Git commit containing the approved plan |
| `dispatch_ready` | `yes` or `no`; required for new runnable threads |
| `outcome` | `pass` or `blocked` in runnable Result events |
| mailbox `id` | `Q-YYYYMMDD-N`, for example `Q-20260905-4` |

`role`, `agent`, and `model` together identify an owner. `cli` records the
transport and MUST NOT be used as a substitute for owner identity.

## 3. Filenames

Role notes and discuss threads MUST use:

```text
YYYY-MM-DD_HHMM_<slug>.md
```

- `HHMM` is local 24-hour time.
- `<slug>` MUST be descriptive and SHOULD contain only lowercase ASCII
  letters, digits, hyphens, and underscores.
- A new note MUST use a new filename. Agents MUST NOT overwrite another
  agent's note.
- Replies append to the existing discuss thread and MUST NOT create another
  thread for the same question or generation.

## 4. Common role-note format

Every file under `reviews/`, `eng/`, or `test/`, except `README.md`, MUST have
these fields and sections:

```markdown
# 2026-09-05 -- Short title

- role: eng
- agent: concrete-agent-id
- model: exact-model-id
- cli: cli-product
- status: done

## Summary

One concise statement of the work and outcome.

## Pointers

- `docs/plans/example.md`
```

Rules:

- `## Summary` and `## Pointers` are mandatory even when the note is short.
- `## Pointers` MUST contain at least one repository path, evidence path,
  thread, or exact commit. Use `- none` only when no durable pointer exists.
- An acknowledgement-only turn SHOULD NOT create a role note.
- A handoff that did not claim or execute a task MUST use the common format
  only. It MUST NOT put `parent` or `subtask` in metadata. It MAY mention the
  related task in the body.

## 5. Reviews note

A reviews decision or plan note SHOULD use:

```markdown
# 2026-09-05 -- Short review title

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Decision and scope.

## Acceptance

- Observable pass condition.

## Consensus

- Codex agent: distinct-codex-id
- Thread: `docs/agents/discuss/YYYY-MM-DD_HHMM_slug.md`
- Result: reached

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| ST-1 | `owner/model` | none | Bounded behavior | Observable result | Focused regression |

## Risks

- Relevant risk or assumption.

## Pointers

- `docs/plans/example.md`
```

`## Acceptance`, `## Consensus`, and `## Subtasks` MAY be omitted for a narrow
review finding that does not approve or dispatch a unified plan. `## Summary`
and `## Pointers` remain mandatory.

## 6. Eng task note

If an eng role note includes `parent`, it represents claimed executable work
and MUST use all fields and sections below:

```markdown
# 2026-09-05 -- ST-1 implementation

- role: eng
- agent: concrete-owner
- model: exact-model-id
- cli: cli-product
- status: done
- parent: TASK-20260905
- subtask: ST-1
- base_revision: 0123456789abcdef0123456789abcdef01234567
- started_at: 2026-09-05T16:00:00+08:00
- completed_at: 2026-09-05T17:00:00+08:00

## Summary

What changed and why.

## Requirement

- Owner's exact interpretation of scope and acceptance.

## Changed

- `src/package/file.py`

## Verification

- `python3 -m pytest ...`: pass.

## Result

- pass at `fedcba9876543210fedcba9876543210fedcba98`.

## Pointers

- `docs/status/evidence/run/`
```

Rules:

- `base_revision`, `started_at`, `completed_at`, `## Requirement`, and
  `## Result` become mandatory whenever `parent` is present.
- A running or blocked note uses `status: open` and `completed_at: n/a` until
  the task passes.
- `## Verification` MUST list exact commands and results. Use `not run: reason`
  when execution was impossible.
- A passing Result MUST point to a real Git commit. Dirty worktree state is not
  a revision.

## 7. Test task note

If a test role note includes `parent`, it MUST use:

```markdown
# 2026-09-05 -- Independent audit

- role: test
- agent: concrete-owner
- model: exact-model-id
- cli: cli-product
- status: done
- parent: TASK-20260905
- subtask: AUDIT-1
- revision: fedcba9876543210fedcba9876543210fedcba98

## Summary

What was tested and the overall result.

## Commands

- `exact command`

## Evidence

- `docs/status/evidence/run/`

## Result

- pass

## Pointers

- `docs/plans/example.md`
```

The tested `revision` MUST be exact. Test evidence MUST be referenced, not
copied into the note.

## 8. Discuss thread metadata

Every discuss thread MUST begin with common routing metadata:

```markdown
# 2026-09-05 -- Short thread title

- status: open
- to_role: reviews
- to_agent: codex
- to_model: gpt-5
- kind: question
- parent: n/a
- subtask: n/a
- depends_on: none
- revision: n/a
```

The first body section MUST be `## Post -- ...` or `## Summary`.

### 8.1 Question

- `parent`, `subtask`, and `revision` MAY be `n/a`.
- The target MAY be a concrete owner or a role-pool broadcast.
- Close only after a Reply resolves the Open item.

### 8.2 Consensus

Consensus metadata MUST add:

```markdown
- consensus: open
```

Additional rules:

- `parent` and `revision` MUST NOT be `n/a`.
- It MUST route `reviews` to `reviews`.
- `to_agent` MUST be a distinct Codex id and MUST differ from `from_agent`.
- Closing requires both `status: done` and `consensus: reached`.
- Consensus is requirement clarification, not a second reviewer approval tier.

### 8.3 Runnable work

`subtask`, `integration`, and `regression` threads MUST add:

```markdown
- generation: 1
- plan_revision: 0123456789abcdef0123456789abcdef01234567
- dispatch_ready: no
```

Additional rules:

- `parent`, `subtask`, `revision`, `generation`, and `plan_revision` MUST be
  concrete.
- `to_agent` and `to_model` MUST be concrete; `any` is forbidden.
- Reviews MUST be the sender for new `subtask` and `integration` work.
- Set `dispatch_ready: yes` only after scope, acceptance, tests, pointers,
  owner, dependencies, and revisions are complete.
- Once claimed, owner/scope/dependencies/revisions MUST NOT be edited in place.
  Create a higher-generation replacement instead.

## 9. Thread events

Event headings MUST use exactly:

```markdown
## Post -- reviews/reviewer-id -- 2026-09-05 16:00 -- codex/gpt-5

Initial request or additional context.

## Reply -- reviews/codex -- 2026-09-05 16:15 -- codex/gpt-5

Answer or consensus analysis.
```

Runnable Claim:

```markdown
## Claim -- eng/owner-id -- 2026-09-05 16:30 -- cursor/grok-4.6

- started_at: 2026-09-05T16:30:00+08:00
- claimed_generation: 1
- claimed_plan_revision: 0123456789abcdef0123456789abcdef01234567
- claimed_dependencies: none
```

Passing Result:

```markdown
## Result -- eng/owner-id -- 2026-09-05 17:30 -- cursor/grok-4.6

- outcome: pass
- completed_at: 2026-09-05T17:30:00+08:00
- revision: fedcba9876543210fedcba9876543210fedcba98
- tests: focused and affected regression suites pass
- summary: Acceptance criteria satisfied.
- evidence: docs/status/evidence/run/
```

Blocked Result:

```markdown
## Result -- eng/owner-id -- 2026-09-05 17:30 -- cursor/grok-4.6

- outcome: blocked
- completed_at: 2026-09-05T17:30:00+08:00
- summary: Exact external decision or unavailable dependency.
- evidence: docs/status/evidence/run/
```

A blocked Result leaves the thread and mailbox row open. Ordinary test failure
is not a blocker; the owner continues repairing it.

Supersede, cancel, and stop acknowledgement events MUST be created through the
mailbox lifecycle helper. A superseding replacement MUST preserve
`parent/subtask` and increment `generation`.

## 10. OPEN.md mailbox

The mailbox header MUST have exactly these columns in this order:

```markdown
| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request | generation | plan_revision |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
```

Rules:

- Use `scripts/agent_notify.sh`; do not construct rows by hand.
- A row MUST mirror its thread's routing and lifecycle metadata.
- Runnable rows MUST populate `generation` and `plan_revision`.
- Question and consensus rows leave those two cells blank.
- `dispatch_ready` lives only in thread metadata.
- Removing a row does not prove completion; a passing Result does.
- Never reuse a runnable thread to send a question. Create a separate question
  thread or append a Reply without changing runnable metadata.

## 11. RUNTIME.md registry

The runtime table MUST have exactly these columns:

```markdown
| id | role | agent | model | cli | session | worktree | state | capabilities | heartbeat |
|---|---|---|---|---|---|---|---|---|---|
```

All session rows MUST remain directly below that table header and separator.
Explanatory prose MUST come before or after the complete table, never between
registered rows. Use `scripts/agent_register.sh` rather than editing rows by
hand. Allowed states are `idle`, `busy`, and `down`.

Registration is presence only. It does not create a task Claim or prove work
completion.

## 12. Evidence and revisions

- Test and simulation artifacts MUST live under
  `docs/status/evidence/<topic>/<run>/`.
- A note or Result MUST point to evidence with repository-relative paths.
- The evidence summary MUST record the tested Git commit and dirty-file count.
- A passing runnable Result MUST resolve to an existing Git commit.
- Worktree names, branch names, or labels such as `*-wt` MAY describe a base,
  but MUST NOT be the final passing revision.

## 13. Lifecycle table

| Kind | Initial state | Start action | Passing close | Blocked/rework |
|---|---|---|---|---|
| `question` | `status: open` | append Reply | set `done`, remove row | keep open with exact unresolved question |
| `consensus` | `status: open`, `consensus: open` | append Reply | set `consensus: reached` and `done`, remove row | list exact amendments in same thread |
| runnable | `status: open`, `dispatch_ready: no` | reviews sets `yes`; owner Claims | Result pass, status done, row removed | Result blocked stays open; changed scope creates higher generation |

## 14. Required commands

Before reporting a communication/document change complete, run:

```bash
git diff --check
scripts/check_agent_contract.sh
```

Both commands MUST pass. If failures predate the current change, report every
remaining file and missing field explicitly; do not claim contract compliance.

## 15. Common invalid forms

- `- role: reviewers` -- invalid canonical role; use `reviews`.
- `- parent: TASK-1` without the companion task metadata -- invalid role note.
- A handoff note with task metadata but no `## Requirement`/`## Result` --
  invalid; either complete the task-note schema or remove task metadata.
- `## summary` -- invalid case; use `## Summary`.
- `status: done` runnable thread without Result pass -- invalid.
- Passing Result with a dirty worktree label instead of a commit -- invalid.
- Appending `--kind question` to an existing runnable thread -- invalid because
  it rewrites authoritative metadata.
- Directly assigning work after changing a claimed task in place -- invalid;
  create and dispatch a higher generation.
