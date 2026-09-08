# 2026-09-05 -- PF-R6 root-cause consensus

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R6
- reviewed_revision: 218f444406217bab779ce99333ec75fb8b24e1de

## Summary

Reached consensus that PF-R6 remains below its performance gate but that the
current probe does not prove the proposed exact-stamp/temporal-hold root cause.
The replacement generation must repair instrumentation before further online
path optimization.

## Decision

consensus: reached

The PF-R6 blocked conclusion is only partially supported. The evidence supports
that PF-R6 is still below the 4 Hz bar and needs a fresh owner, but it does not
prove the stated exact-stamp / temporal-hold root cause. The next PF-R6
generation must be instrumentation-first.

## Accepted Boundary

- Treat `1ce5f6c` as a partial candidate revision, not an accepted performance
  fix.
- Retain `stage_perf_probe.py` as provisional instrumentation, but extend it or
  add in-node counters for causal rates, exact join misses, backlog, buffer
  occupancy, stage timing, RSS trend, and time to first TOP_ONLY/FULL_3D.
- Keep the band-first support filter provisionally; it commutes finite-point
  predicates and has reported perception regression coverage.
- Do not accept lazy raw processing until focused tests prove exact-stamp
  support availability and retry semantics, and measurement shows no
  critical-path regression.
- Do not select stamp semantics, QoS, queue, temporal-hold, or preprocessor
  changes until a controlled baseline localizes the bottleneck.
- Any further online detection-path change requires rerunning PF-G6S and the
  original PF-R5 30-trial Gate 4 accuracy matrix before PF-R6 can close.
- Supersede the `claude/glm-5.3` generation 1 blocked attempt and assign a
  higher-generation PF-R6 replacement to `codex/gpt-5`.

## Evidence Read

- `docs/agents/reviews/2026-09-05_1724_pf-r6-profiling-conclusion-review.md`
- `docs/agents/eng/2026-09-05_1720_pf-r6-handoff.md`
- `docs/agents/eng/2026-09-05_1830_full-handoff-eng-successor.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1800_pfr6-profiling/`
- `docs/plans/platform_free_height_remediation.md`

## Rationale

The reported optimized evidence has causal rate inversions (`yolo > pre_rgb`,
`frame > cargo`) and therefore cannot establish a cargo exact-stamp ceiling.
The semantic segmenter publishes current input stamps after temporal gating, so
the handoff's old-stamp replay description is not a confirmed implementation
fact. A fresh PF-R6 owner can proceed, but only with the instrumentation and
regression boundary above.

## Pointers

- `docs/agents/discuss/2026-09-05_1725_pf-r6-profiling-root-cause-consensus.md`
- `docs/agents/reviews/2026-09-05_1724_pf-r6-profiling-conclusion-review.md`
- `docs/plans/platform_free_height_remediation.md`
