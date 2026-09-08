# Scheduler Reviewer Model Allocation

- date: 2026-09-05 16:13
- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- task: model-allocation-strategy
- status: done

## Summary

Scheduler should remain deterministic code with explicit contract checks,
leases, dependency rules, and freshness gates; it should not rely on a stronger
LLM for core routing safety. Stronger models are most valuable in reviews,
integration review, and high-risk engineering subtasks where ambiguous
requirements, architecture tradeoffs, failure interpretation, or cross-task
conflicts must be resolved. Routine dispatch can be script-driven from
structured eng/test results and next-todo fields.

## Pointers

- `docs/plans/mailbox_plan_freshness.md`
- `docs/agents/README.md`
