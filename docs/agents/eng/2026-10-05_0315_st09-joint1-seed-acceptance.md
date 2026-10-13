# 2026-10-05 -- Joint-1 seed and held-out acceptance

- role: eng
- agent: cursor
- model: grok-4.7
- cli: cursor
- status: blocked

## Summary

The frozen g2 matrix was not regenerated. One deterministic tool-down
seed, joint 1 shifted by -pi, was added after the dev split showed it
refines every zero-goal miss that the nominal tool-down and start priors
miss. The single held-out acceptance still fails the rate gate. G3 Run03
was not started.

## Whole-chain failures

The two `PLANNING_BUDGET_EXHAUSTED` rows are not zero-IK. The saved
select row has IK successes from anchor, tool-down, and start, three
goals tried, five goals still open, and `SEARCH_INCOMPLETE` at about 9 s.
The two MoveIt `-26` lines are the same `PORTAL_TRANSIT`, same start
joints and same goal joints, about 3 s apart, on the shared whole-chain
fixture. They are one repeated edge failure, not two new seed defects.

## Dev seed

On 63 dev cases with zero IK goals under atlas-on state search, nominal
tool-down and start refined none. The joint-1 shift refined all 63.
Atlas queries for those poses were empty. Uniform reference extras also
solved them and were not copied.

## Held-out acceptance (123 feasible, frozen ids, one rep)

| Planner | Atlas | Solved | Rate | Wilson 95% lower |
|---|---|---|---|---|
| state | on | 112/123 | 0.911 | 0.847 |
| state | off | 116/123 | 0.943 | 0.887 |
| baseline | on | 64/123 | 0.520 | 0.433 |
| baseline | off | 64/123 | 0.520 | 0.433 |

Invalid false success, branch flips, proxy collisions, and routes over
10 s are 0. Slot64 first DESCEND is tier 0 with fraction 1.0. Atlas on
is not an improvement over atlas off on this run.

Pre-change state/atlas-on was 97/123. Budgets, IK tolerances, and the
0.01 rad retreat gate are unchanged. Slot64 replay exited 0. Grid and
producer tests: 67 passed.

## Evidence

`docs/status/evidence/frame_migration/ST09-parallel-I-s5/route_matrix_g2_20261005/summary.json`

## Pointers

- `src/luggage_planning/luggage_planning/route_producer.py`
- `scripts/st09_i/route_matrix_eval.py`
- `docs/status/evidence/frame_migration/ST09-parallel-I-s5/bi_route_infra_20261005/manifest.json`
