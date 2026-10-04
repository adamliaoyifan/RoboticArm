# 2026-10-04 -- ST09-I state-conditioned route search + offline matrix

- role: eng
- agent: cursor
- model: claude-opus-5.5
- cli: cursor
- status: blocked

## Summary

Implemented state-conditioned search inside the existing planner. The
offline scenario matrix gate FAILS on feasible-case rate: 14/16 held-out
reference-feasible cases (0.875, Wilson95 lower 0.64) against the 0.95 /
0.90 bar. The baseline (08d25cf) scores 10/16 on the same cases. Invalid
cases, continuity, proxy collision, timing and the slot64 first-DESCEND
checks pass. No sim was run; no budget or retreat gate changed.

## Changes (worktree `integration/st09-converge-i-g3-fix`)

- `08d25cf` baseline: DESCEND pair rank + exact hover refine (prior step).
  Also contains `.cursor/rules/container-geometry.mdc`, which the review
  reply asked to keep out; history not rewritten, needs a decision.
- `e5c9a87`: `RouteSearch(continuation=...)`; same-branch destinations
  first on INSERT/DESCEND (deferred, not deleted);
  `MAX_SAME_FAILURE_PER_GOAL = 3` per (goal, edge class, reason);
  `detail.search_outcome` (`SEARCH_INCOMPLETE` / `UNREACHABLE`) plus
  `detail.coverage` beside the frozen Reason. `GraphRoutePlanner`: branch
  ladders back-seeded from each goal up to the joint-space transit;
  frontier continuation for INSERT/DESCEND/RETREAT; `region_poses` from the
  anchor plan (`route_producer` passes them). `placement.md` updated
  (atlas = prior, reachability identity, outcome table).
- `1e9740d`: zero refined IK goals -> `SEARCH_INCOMPLETE`; matrix harness
  per-case aggregation and 35-case minimum-sample rule; evidence.
- `df21b96`: ladder pose prefers the current candidate's plan over stale
  graph anchors.

## Evidence

`docs/status/evidence/frame_migration/ST09-parallel-I-s5/route_matrix_20261004/`
(commit e5c9a87, dirty 0; summary recomputed with `--resummarize`).

| Metric | baseline | state |
|---|---|---|
| held-out feasible cases solved | 10/16 | 14/16 |
| dev feasible cases solved | 20/32 | 27/32 |
| invalid false success | 0/6 | 0/6 |
| INSERT/DESCEND max step / flips / proxy collisions | 0.035 / 0 / 0 | 0.035 / 0 / 0 |
| wall p95 (offline) | 0.25 s | 0.32 s |

- Atlas on and off give identical results: atlas seeds reached only 336
  of 774 atlas-on runs and never changed an outcome. Atlas mode is not
  meaningfully exercised (tool-yaw convention of the query unverified).
- 252/300 generated cases are `undetermined` (the reference search found
  no route), so only 48 are scorable; the 0.90 Wilson bar needs >= 35
  held-out feasible cases.
- Both remaining state failures (`carryon|*|pick_tooldown|occ0|h0|wall_y`):
  first divergent stage is goal seeding. Start and tool-down seeds refine
  onto no goal; the reference succeeded only with random extra seeds.
- Slot64 Run02: first DESCEND fraction 1.0, tier 0. Its full route fails
  offline because slot_approach/exit (z ~1.01-1.11) sit at the reach
  boundary; reported as a regression case, not counted as feasible.

Limits: offline edges (straight TCP lines, linear joint interpolation),
payload-only collision proxy (no arm links), offline wall time.

## Open

- Review-reply scope defect (accumulated `slot_upper`/goal nodes across
  candidates in `plan_place_route`) is not fixed; required before Run03.
- Gate fail decision: goal-seed coverage (forward ladder seeds), atlas yaw
  check, and a larger feasible sample, then a fresh held-out split.

## Pointers

- `src/luggage_planning/luggage_planning/grid_path_search.py`
- `src/luggage_planning/luggage_planning/reachable_graph.py`
- `scripts/st09_i/route_matrix_eval.py`
- `tests/st09_parallel/B/test_state_conditioned_search.py`
