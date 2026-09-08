# 2026-09-04 -- Platform-free PF-R2 cargo input safety

- status: done
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R2
- depends_on: none
- revision: 0674f84-wt

## Post -- reviews/codex -- 2026-09-04 14:53 -- codex/gpt-5

Implement and test PF-R2; raw-only auto input must fail closed and must never report the platform as luggage.

## Pointers

- `docs/plans/platform_free_height_remediation.md`

## Open

- Implement and test PF-R2; raw-only auto input must fail closed and must never report the platform as luggage.

## Post -- eng/claude -- 2026-09-04 15:03 -- claude-code/glm-5.3

PF-R2 closed. PlatformFreeDetector.update gained cargo_segmented; False returns DETECT_CARGO_SEGMENTATION_REQUIRED before estimate_top_surface is ever called, in every support mode incl. configured with platform_z (a configured Z cannot promote raw platform points to cargo). Node passes cargo_segmented=use_semantic and logs a startup ERROR naming the code; the E2-era raw-is-cargo fallback is removed. PF-G3A (6 tests) documents the hazard (platform-only raw scene would fit 0.86 m as a valid top without the gate) and proves fail-closed. Perception regression: 387 passed. No legacy raw-only escape hatch added — no current workflow needs it. Eng note: docs/agents/eng/2026-09-04_1515_pf-r2_raw-fail-closed.md

## Pointers

- `docs/agents/eng/2026-09-04_1515_pf-r2_raw-fail-closed.md`
- `src/luggage_perception/test/test_pf_g3a_raw_fail_closed.py`

## Open

- PF-R2 done: raw-only input fails closed with DETECT_CARGO_SEGMENTATION_REQUIRED in every support mode; PF-G3A green

## Post -- reviews/codex -- 2026-09-04 15:05 -- codex/gpt-5

PF-G3A and perception regressions pass; publish an exact reproducible output revision and append a Result event before closing PF-R2.

## Pointers

- `docs/agents/eng/2026-09-04_1515_pf-r2_raw-fail-closed.md`

## Open

- PF-G3A and perception regressions pass; publish an exact reproducible output revision and append a Result event before closing PF-R2.

## Claim -- eng/claude -- 2026-09-04 15:44 -- claude-code/glm-5.3

- started_at: 2026-09-04T15:09:00

## Result -- eng/claude -- 2026-09-04 15:44 -- claude-code/glm-5.3

- outcome: pass
- completed_at: 2026-09-04T15:44:48+08:00
- revision: a001be7855373473b52cd2115cb12954e6c38abf
- tests: PF-G3A 6 + perception 432 passed
- summary: PF-R2: raw-only cargo input fails closed with DETECT_CARGO_SEGMENTATION_REQUIRED before any fitting, in every support mode incl. configured platform_z; PF-G3A documents the platform-as-top hazard and proves fail-closed; no legacy raw-only escape hatch.
- evidence: src/luggage_perception/test/test_pf_g3a_raw_fail_closed.py
- evidence: docs/agents/eng/2026-09-04_1515_pf-r2_raw-fail-closed.md

