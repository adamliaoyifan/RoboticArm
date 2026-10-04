# 2026-10-05 -- Route matrix lineage

- role: eng
- agent: cursor
- model: grok-4.7
- cli: cursor
- status: done

## Summary

New offline matrix manifests name the three planner commits they descend
from. The 2026-10-04 matrix stays at `e5c9a87`. Its 16 reference-feasible
held-out cases are development data: the two goal-seed misses were already
used to choose the next seed change, so that split cannot accept a later
planner.

## Changes

- `scripts/st09_i/route_matrix_eval.py` writes `lineage` on every new
  manifest: `f9ba0e8` DESCEND pair baseline, `bdb9394` seven-face drawing
  rule, `dd6d48e` candidate scope. `commit` and `dirty_count` remain the
  run revision.
- `route_matrix_20261004/manifest.json` is unchanged (`e5c9a87`, no
  `lineage` key).

## Evidence

Whole-chain isolation is `7b194de`, recorded separately. Nine live tests
executed with no skip and no duplicate `rclpy.init`: 5 passed, 4 failed
(`PLANNING_BUDGET_EXHAUSTED` twice, `EDGE_BUILD_FAILED` / MoveIt `-26`
twice). Those product failures are not rewritten here.

`docs/status/evidence/frame_migration/ST09-parallel-I-s5/bi_route_infra_20261005/manifest.json`

## Open

Atlas yaw is still unverified. The next matrix is a new frozen split with
at least 35 reference-feasible held-out cases, measured at the current
planner before any goal-seed change.

## Pointers

- `scripts/st09_i/route_matrix_eval.py`
- `docs/agents/eng/2026-10-04_2310_st09-i-state-conditioned-search.md`
- `docs/status/evidence/frame_migration/ST09-parallel-I-s5/route_matrix_20261004/manifest.json`
