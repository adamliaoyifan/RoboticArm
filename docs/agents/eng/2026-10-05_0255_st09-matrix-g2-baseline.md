# 2026-10-05 -- Frozen matrix baseline, gate fail

- role: eng
- agent: cursor
- model: grok-4.7
- cli: cursor
- status: blocked

## Summary

The scored baseline on the frozen g2 matrix fails the held-out rate.
Planner code is still `dd6d48e`. No goal-seed change was made: on the dev
split, every zero-goal miss was solved only by the reference's uniform
random seeds, and those queries returned no atlas seeds.

## Held-out state planner (123 feasible cases, one rep)

| Atlas | Solved | Rate | Wilson 95% lower |
|---|---|---|---|
| on | 97/123 | 0.789 | 0.708 |
| off | 82/123 | 0.667 | 0.579 |

Baseline, atlas on and off, both 64/123. Invalid false success 0. Branch
flips 0. Proxy collisions 0. Routes over 10 s: 0. Slot64 first DESCEND
fraction 1.0 at rank tier 0. Atlas on and off are not the same result:
state mode gains 15 held-out cases with the atlas.

## Dev seed attribution

63 dev cases that the reference solved and the state planner missed had
zero IK goals with atlas on. Re-refining those goals: hover, atlas,
start, and tool-down solved none; a reference uniform sample solved all
63. The atlas query was empty on all 63. The joint farthest from the
tool-down seed was spread across joints 0, 3, 4, and 5. That is not a
finite candidate-conditioned family, so those random samples were not
copied into the planner.

## Evidence

`docs/status/evidence/frame_migration/ST09-parallel-I-s5/route_matrix_g2_20261005/summary.json`

The 2026-10-04 manifest is still `e5c9a87`.

## Pointers

- `docs/status/evidence/frame_migration/ST09-parallel-I-s5/route_matrix_g2_20261005/freeze.json`
- `scripts/st09_i/route_matrix_eval.py`
