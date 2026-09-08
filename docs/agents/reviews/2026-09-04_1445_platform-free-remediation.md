# 2026-09-04 - Platform-free remediation handoff

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Approved a focused remediation sequence for the failed platform-free
evaluation. The implementation must preserve the E0 contract through the live
planning adapter, fail closed on unsegmented raw cargo, and enforce stamped
status/TF. The eval harness is corrected before semantic accuracy and
performance are accepted. `claude/glm-5.3` owns implementation, testing,
failure repair, and evidence for PF-R1 through PF-R6; `cursor/grok-4.6` owns
the final independent PF-R7 integration audit.

## Acceptance

- G0 live ROS adapter preserves top/height validity and pick contact Z.
- Raw-only/auto cannot identify the 0.86 m platform as luggage.
- Missing/stale status and missing historical TF cannot create FULL_3D.
- Gate 4 enforces non-empty full-geometry/support/dimension metrics.
- Thirty-trial semantic accuracy and accepted-profile 4 Hz performance pass.
- Final G0-G4/G6 regression uses an exact reproducible revision.
- Gate 5 remains required before hardware acceptance.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| PF-R1 | `claude/glm-5.3` | none | ROS adapter contract | Live top-Z contract survives conversion | PF-G0A |
| PF-R2 | `claude/glm-5.3` | none | Raw-only fail closed | Platform is never valid cargo | PF-G3A |
| PF-R3 | `claude/glm-5.3` | none | Stamped status and semantic TF | Missing/stale inputs remain top-only | PF-G2A |
| PF-R4 | `claude/glm-5.3` | none | Eval harness | Required metrics cannot pass empty | PF-G4H |
| PF-R5 | `claude/glm-5.3` | PF-R1,PF-R2,PF-R3,PF-R4 | Semantic online accuracy | Gate 4 passes | PF-G4S |
| PF-R6 | `claude/glm-5.3` | PF-R4,PF-R5 | Performance | Gate 6 passes | PF-G6S |
| PF-R7 | `cursor/grok-4.6` | PF-R1,PF-R2,PF-R3,PF-R4,PF-R5,PF-R6 | Independent integration audit | G0-G4/G6 pass | PF-E2E |

## Risks

- The current implementation is an uncommitted working tree on `0674f84`;
  Claude must produce exact checkpoint revisions without reverting unrelated
  changes.
- The prior G4 run used raw-only input and cannot establish semantic accuracy.
- The prior 1.92 Hz figure includes orchestration gaps and cannot isolate
  detector performance.
- No real rosbag exists, so hardware acceptance remains pending.

## Pointers

- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/platform_free_height_test_plan.md`
- `docs/status/evidence/platform_free_height/2026-09-04_1412_g0g6/`
- `docs/agents/discuss/2026-09-04_1143_platform-free-height-eng.md`
