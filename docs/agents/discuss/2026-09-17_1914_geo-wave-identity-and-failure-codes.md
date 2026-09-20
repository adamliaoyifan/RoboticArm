# 2026-09-17 -- GEO wave -- cargo map geometry identity and honest placement failure codes

- status: done
- to_role: eng
- to_agent: cursor
- to_model: opus-5
- kind: subtask
- parent: GEO-WAVE-20260917
- subtask: GEO-1-3
- depends_on: none
- revision: b720976
- generation: 1
- plan_revision: b720976
- dispatch_ready: yes

## Post -- reviews/cursor -- 2026-09-17 19:14 -- cursor/opus-5

User selected the GEO wave from the 12dd654 dirty-tree audit. Three units in one owner loop: GEO-1 stamp the kernel geometry_hash on surface_map_2d, the commit ledger, GetCargoMapStats and both directions of ComputePlacement, and make placement_planner reject a foreign map instead of answering from its floor prior. GEO-2 return a reason_code so BIN_FULL requires that no enumerated candidate was capacity-feasible, with aperture, corridor, unobserved and insufficient support classed as PLACE_CANDIDATE_EXHAUSTED and an oversized box as BOX_EXCEEDS_CONTAINER. GEO-3 build, full offline suites, a launch-free two-node fail-closed test, and one headless place_only dry-run over P0-P4 with zero residuals.

## Pointers

- `docs/agents/reviews/2026-09-16_1227_dirty-tree-defect-audit.md`
- `docs/architecture/container_geometry.md`

## Open

- Implement GEO-8 identity propagation and GEO-9 failure taxonomy, verify offline plus one place-only dry-run, and close end to end.

## Claim -- eng/cursor -- 2026-09-17 19:14 -- cursor/opus-5

- started_at: 2026-09-17T19:14:22+08:00
- claimed_generation: 1
- claimed_plan_revision: b720976
- claimed_dependencies: none

## Result -- eng/cursor -- 2026-09-17 19:15 -- cursor/opus-5

- outcome: pass
- completed_at: 2026-09-17T19:15:04+08:00
- revision: 3b50f8411c0604fd7ff87fa853832e993eeecb8c
- tests: offline suites 1308 passed plus 388 planning; place_only dry-run 5/5 with zero residuals
- summary: GEO-8 identity and GEO-9 taxonomy landed; geometry_hash matched on 5/5 ComputePlacement calls and P3 BIN_FULL agreed with the independent capacity test.
- evidence: docs/status/evidence/geo_wave/2026-09-17_dryrun2/

