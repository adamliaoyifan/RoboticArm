# 2026-09-10 -- Independent wave dependency normalization

- role: reviews
- agent: codex-reviews-main
- model: gpt-5
- cli: codex
- status: done

## Summary

The wave still has no unfinished product prerequisite. TCIG-1 and SIM-R1-1
are accepted, pinned source inputs rather than runnable scheduler dependencies.
TCIG-2 and SIM-R1-5 require replacement generations because their owners could
not produce valid helper-created claims against legacy generationless threads.
TCIG-5 and TCIG-7 retain their valid generation-1 claims and unchanged scope.

## Acceptance

- Replacement TCIG-2 and SIM-R1-5 rows use `depends_on: none`, a higher
  generation, this clarification's exact plan revision, and the same owners.
- Generation-1 TCIG-2 and SIM-R1-5 are superseded and their paused owners
  acknowledge stop before the stale rows are removed.
- TCIG-5 and TCIG-7 continue at `4425f22`; no product requirement changed.

## Consensus

- Codex agent: `codex-independent-ready-consensus`
- Thread: `docs/agents/discuss/2026-09-10_1518_independent-ready-wave-consensus.md`
- Result: reached; metadata clarification does not alter the accepted scope.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| TCIG-2 g2 | `codex-tcig-map-eng/gpt-5.6-sol` | none | Same approved TCIG-2 scope | Valid helper Claim on exact new plan revision | Unchanged plan gates |
| SIM-R1-5 g2 | `codex-sim-r1-eng/gpt-5.6-sol` | none | Same approved SIM-R1-5 scope | Valid helper Claim on exact new plan revision | Unchanged plan gates |

## Risks

- Accepted legacy commits remain architectural inputs even though they are no
  longer named as live mailbox dependencies.

## Pointers

- `docs/plans/independent_ready_wave_20260910.md`
- `docs/agents/discuss/2026-09-10_1518_independent-ready-wave-consensus.md`
