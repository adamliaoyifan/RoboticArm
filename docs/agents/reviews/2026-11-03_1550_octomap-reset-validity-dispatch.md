# 2026-11-03 -- OctoMap reset validity follow-up dispatch

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

The user decided that reset immediately invalidates OCC for Surface, placement and planning. New requests cannot read the old map; old requests cannot publish or consume old results. Planning resumes only after a valid new-epoch map and freshly sampled robot state. This new contract supersedes no S3 result. S3 run01 remains product `fail` with its original frozen judge; its 3.36 MiB held view and 72-byte buffer observations need ownership and validity evidence under the new contract.

## Acceptance

- `docs/plans/OCTOMAP_RESET_VALIDITY_NEXT_20261103.md` defines state transitions, linearization, owner accounting, downstream admission, runner/judge corrections, exact finite workloads, thresholds and evidence budgets.
- Extra S3 L requests are allowed up to two per stream, retained and fully judged; normal planned IDs remain mandatory.
- Reset safety uses separate V1/V2/V3 gates. An expected epoch mismatch is accepted only with proof that its result was never admitted downstream.
- Claude owns implementation plus focused verification and a dependent whole-chain scoring task; the latter binds the actual implementation commit before it can be claimed.

## Subtasks

| ID | Owner agent/model | Depends on | Base revision | Scope | Acceptance | Required tests | Commit evidence |
|---|---|---|---|---|---|---|---|
| OCC-RESET-VALIDITY-IMPL | `claude/glm-5.3` | none | product `3bb26935f091055d77b9defa87511489cbfb32e6`, S3 tools `90049efeeebc75f1a940e8d7bbbe89e59a599f44` | State, Surface, placement/planning gate, ownership, runner/judge | Plan §§2–6 V1/V2/V3 and negative fixtures | 210 invalidation, 30 recovery, 240 lock cases, 60 ownership cycles; disk judge | Clean product/tool commit plus `docs/status/evidence/` pointer |
| OCC-RESET-VALIDITY-INTEG | `claude/glm-5.3` | OCC-RESET-VALIDITY-IMPL | Bind passing implementation commit | Frozen whole chain and score | Plan §§5–8; pass/fail/not_evaluated separate | P/H 9 windows, L 120 min, R pressure matrix, four-operation O9, replay | Clean scored commit and run manifest with exact hashes |

## Risks

- The S3 report describes a 72-byte residual after release, but lacks the owner/reference timeline needed to prove whether the last owner released. The implementation must instrument and resolve that question.
- Existing shared worktree contains unrelated edits. Claude should isolate code in a fresh worktree and keep the main workspace mailbox authoritative.
- A 10–80 Hz pressure sweep may still not reach pressure. The frozen plan requires a named `not_evaluated` result rather than a passing coverage claim.

## Pointers

- `docs/plans/OCTOMAP_RESET_VALIDITY_NEXT_20261103.md`
- `docs/status/evidence/occ_freespace_final_20261102/scoring_run01/FINAL_REPORT.md`
- `docs/architecture/production_orchestration.md`
