# PF-R5 Vintage Segmenter Decision

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R5
- revision: a7ee351cede4d28a92f5dac9f3a40d540db02ba8

## Summary

PF-R5 is not accepted at `top_surface_rate=0.923` because the approved Gate 4
requires at least `0.95` and both 30-trial runs reproduce the same systematic
vintage-mesh miss. Geometry accuracy and the STL-derived observable GT are
accepted as the basis for continued evaluation, but they do not waive online
semantic availability. PF-R5 remains with `claude/glm-5.3` for a bounded
segmenter-side repair and complete rerun.

## Acceptance

- Repair vintage-mesh detection at the failing and neighboring XY/yaw poses
  without online GT, spawner geometry, or a relaxed evaluator threshold.
- A lower global confidence threshold is acceptable only if targeted negative
  controls prove robot-base and scene false positives remain rejected.
- Add focused regression coverage for the vintage failure mode.
- Rerun the complete held-out 30-trial PF-G4S matrix and pass every Gate 4
  metric, coverage, raw-only, privileged-input, and teardown requirement.
- Produce final evidence from one isolated clean worktree at an exact committed
  revision. Run5/run6 remain diagnostic because their summaries identify
  `4b02520` with 92/94 dirty files rather than the final source revision.
- PF-R6 and PF-R7 remain blocked until PF-R5 has a passing Result.

## Consensus

- Codex agent: prior PFH remediation consensus
- Thread: `docs/agents/discuss/2026-09-04_1453_pfh-remediation-r5.md`
- Result: existing gate retained; no scope or threshold amendment

## Pointers

- `docs/agents/eng/2026-09-04_1930_pfr1-r5-summary.md`
- `docs/status/evidence/platform_free_height/2026-09-04_1915_pfr5-g4s-run6/summary.json`
- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/platform_free_height_test_plan.md`

## Open

- PF-R5 owner must return one clean exact-revision passing run before
  dependency release.
