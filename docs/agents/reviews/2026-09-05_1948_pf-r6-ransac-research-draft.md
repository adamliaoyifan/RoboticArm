# 2026-09-05 -- PF-R6 RANSAC research draft

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Amended Q-20260905-15 from an immediate z-mode/median implementation into an
isolated support-plane method comparison. The proposed Grok task must identify
the exact automotive triangle/constrained-RANSAC method and benchmark it,
z-mode/median, and the current baseline before recommending a production
implementation candidate.

## Acceptance

- At least one constrained-RANSAC candidate is actually prototyped and timed.
- Every method receives identical non-privileged support candidates.
- Accuracy, fail-closed negative controls, and latency gates are all reported.
- Top RANSAC and production modules remain unchanged.
- A positive result only recommends a method to the existing PF-R6 owner.

## Consensus

- Codex agent: `codex-ransac-consensus/gpt-5.5`
- Thread: `docs/agents/discuss/2026-09-05_1950_pf-r6-ransac-research-consensus.md`
- Result: reached

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| PF-R6-RANSAC-RESEARCH | `cursor-grok-b/grok-4.6` | none | Isolated literature identification and offline support-plane benchmark | Reproducible reject/continue/implementation-candidate result | Prototype unit tests, frozen fixtures, negative controls, timing comparison, contract check |

## Risks

- Automotive methods may rely on broad continuous road geometry that is absent
  from the pickup annulus; non-transfer is an acceptable research result.
- A fast z estimator can select coherent wrong-height clutter. Speed alone
  cannot qualify a method.
- The active PF-R6 implementation has uncommitted checkpoint changes; research
  must use an exact committed baseline and must not absorb that dirty state.

## Pointers

- `docs/plans/pf_r6_support_plane_method_research.md`
- `docs/agents/discuss/2026-09-05_1942_pf-r6-ransac-optimization-review.md`
- `docs/agents/discuss/2026-09-05_1731_pf-r6-generation2-codex-takeover.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/metrics_extract.json`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/stage_probe.json`
