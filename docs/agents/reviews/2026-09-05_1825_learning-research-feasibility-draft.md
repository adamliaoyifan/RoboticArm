# 2026-09-05 -- Learning research feasibility draft

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Approved a research-only learning feasibility program that keeps the current
model-based pipeline as baseline and deterministic safety kernel. The first
runnable candidate is LRF-P1 for `eng/cursor-grok-b/grok-4.6`; it must return
controlled occlusion metrics before reviews considers any shadow integration.

## Acceptance

- Research inference uses only hardware-observable inputs.
- Production packages, launch defaults, topics, and safety authority remain
  unchanged.
- LRF-P1 returns reproducible candidate-versus-baseline metrics and maps every
  promotion gate to pass, fail, or not measurable.
- Only a separate later task may add a shadow adapter or replace a module.

## Consensus

- Codex agent: `codex-lrf-consensus/gpt-5.5`
- Thread: `docs/agents/discuss/2026-09-05_1827_learning-research-feasibility-consensus.md`
- Result: reached

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| LRF-P1 | `cursor-grok-b/grok-4.6` | none | Offline occlusion-aware 3D perception spike | Reproducible L0 report and objective L1 gate decision; no production replacement | Leakage, OOD, import-isolation, frozen split, candidate/baseline metrics |
| LRF-A1 | `cursor-grok-b/grok-4.6` | LRF-P1 | Future task-aware learned NBV spike | Same-budget downstream improvement without bypassing coordinator gates | Policy contract, deterministic fixtures, simulation metrics |
| LRF-PL1 | `cursor-grok-b/grok-4.6` | LRF-P1,LRF-A1 | Future safe-candidate ranking spike | Three-box improvement with zero accepted hard-gate violations | Offline replay and simulation regression |

## Risks

- Six checked-in suitcase meshes may be too small for a defensible learning
  split; the report must identify this rather than overstate generalization.
- A completion metric can improve while collision risk worsens. Conservative
  under-bound and calibration gates therefore block shadow eligibility.
- Optional model weights or licenses may make a method unreproducible; this is
  a valid negative feasibility result.

## Pointers

- `docs/plans/learning_research_feasibility.md`
- `docs/architecture/perception_architecture.md`
- `docs/architecture/production_orchestration.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/agents/discuss/2026-09-05_1751_hardware-perception-planning-briefing.md`
