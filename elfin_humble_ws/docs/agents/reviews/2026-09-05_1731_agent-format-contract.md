# 2026-09-05 -- Agent format contract

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Added one normative format contract for role notes, discuss threads, lifecycle
events, `OPEN.md`, `RUNTIME.md`, evidence pointers, and revisions. It separates
ordinary handoff notes from claimed task notes and provides exact templates and
invalid examples suitable for mechanical validation.

## Pointers

- `docs/agents/FORMAT.md`
- `docs/agents/README.md`
- `scripts/check_agent_contract.sh`
