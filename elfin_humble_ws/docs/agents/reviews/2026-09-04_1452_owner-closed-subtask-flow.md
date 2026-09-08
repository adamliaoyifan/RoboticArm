# 2026-09-04 -- Owner-closed subtask workflow

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The workflow no longer treats reviews, eng, and test as mandatory sequential
stages. Reviews reaches requirement consensus with a distinct Codex agent,
decomposes the unified task, and assigns each bounded subtask to a concrete
agent/model. That owner reads, implements, tests, repairs, records evidence,
and closes the subtask. A dependent integration subtask owns final regression.

## Acceptance

- Reviews does not implement dispatched subtasks.
- No new subtask is dispatched before reviews-Codex consensus.
- Runnable mailbox rows carry parent, subtask, dependencies, base revision,
  and concrete agent/model ownership.
- The owner runs required tests and fixes failures without a routine test-role
  handoff.
- Independent audit/test work occurs only as an explicitly assigned subtask.
- Efficiency is measured against lead time, queue wait, handoffs, rework, and
  escaped defects rather than inferred from agent count.

## Consensus

- Bootstrap authority: direct user instruction to Codex in this session.
- Codex agent: `codex`.
- Result: reached for the workflow change. Future requirement reviews require
  a distinct reviews proposer and Codex consensus participant.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| WF-1 | `codex/gpt-5` | none | protocol and role rules | owner-closed flow documented | contract check |
| WF-2 | `codex/gpt-5` | WF-1 | mailbox/helper schema | owner/dependency routing enforced | helper smoke |
| WF-3 | `codex/gpt-5` | WF-2 | efficiency experiment | baseline and decision rule recorded | metric review |

## Risks

- Broad subtasks recreate monolithic work, so reviews must keep scope bounded.
- Self-testing can miss correlated defects; high-risk plans should add an
  independent audit or integration owner.
- Different models and task complexity confound timing comparisons; three
  comparable unified tasks are required before claiming improvement.

## Pointers

- `AGENTS.md`
- `docs/agents/README.md`
- `docs/plans/agent_subtask_efficiency_experiment.md`
- `scripts/agent_notify.sh`
- `scripts/check_agent_contract.sh`
