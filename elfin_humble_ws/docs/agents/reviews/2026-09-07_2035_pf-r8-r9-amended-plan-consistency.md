# 2026-09-07 -- PF-R8/PF-R9 amended-plan consistency

- role: reviews
- agent: codex-plan-95522db
- model: gpt-5
- cli: codex
- status: done

## Summary

The second distinct-Codex pass reviewed amended revision `bcafe2ec17e795a6fc218956c2355a6750060e44`. Consensus remains open: several initial defects were fixed, but A3a is impossible on the cited sequence, B1-B3 remain under-specified, architecture and decimation acceptance remain incomplete, A1 retains a border-policy contradiction, PF-R7 can still certify the stale chain, and runnable freshness metadata is absent.

## Consensus

- Codex agent: `codex-plan-95522db`
- Thread: `docs/agents/discuss/2026-09-07_2026_pf-r8-r9-plan-consistency.md`
- Result: not reached; second amendment required before dispatch

## Risks

- A3a would force noncausal or stale holds across 21-frame gaps instead of preserving fail-closed behavior.
- The current PF-R7 dependency graph permits an independent audit before the new fixes integrate.
- Undefined timing clocks, windows, and resource-growth rules can yield incompatible implementations that all appear to satisfy the prose.

## Pointers

- `docs/plans/pf_r8_r9_perception_acceptance.md`
- `docs/agents/discuss/2026-09-07_2026_pf-r8-r9-plan-consistency.md`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/failed_case_capture/failed_cases.jsonl`
- `docs/architecture/perception_architecture.md`
- `docs/architecture/sensor_data_pipeline.md`
