# 2026-09-05 -- TCIG-4 closure-review amendments

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done
- parent: TCIG-20260904
- subtask: TCIG-4
- base_revision: a3dba5e7e0c2fb219062cfb2a091267f6f2d606d
- started_at: 2026-09-05T17:04:00+08:00
- completed_at: 2026-09-05T17:10:54+08:00

## Summary

Applied the three TCIG-4 closure-review findings on isolated branch
`agent/eng/tcig-4` without changing generation, owner, base, or plan revision.
Empty hull-eroded payload-center intervals now block even with no placed boxes.
Place sweeps keep points and yaw in `container_link`. Carry height ignores
wedge-only AABB overlap.

Implementation `d567ad5`; evidence tip `10a93e8`.

## Requirement

Repair reviews findings 1-3 so Gate G4 matches real hull semantics: impossible
first-box corridors fail closed, rotated containers use geometry-frame yaw, and
carry height agrees with hull-filtered obstacles.

## Changed

- `src/luggage_packing/luggage_packing/insertion_corridor.py`
- `src/luggage_packing/test/test_insertion_corridor.py`
- `src/luggage_perception/luggage_perception/corridor_audit.py`
- `src/luggage_perception/test/test_corridor_audit.py`
- `src/luggage_planning/luggage_planning/waypoint_generator.py`
- `src/luggage_planning/scripts/waypoint_generator_node.py`
- `src/luggage_planning/test/test_waypoint_generator.py`

## Verification

- `python3 src/luggage_packing/test/test_insertion_corridor.py`: 13 passed.
- `python3 src/luggage_perception/test/test_corridor_audit.py`: 13 passed.
- `python3 src/luggage_planning/test/test_waypoint_generator.py`: 26 passed.
- `python3 -m unittest discover -s src/luggage_packing/test -p 'test_*.py'`: 80 passed.
- `python3 -m py_compile` on the four exclusive modules: pass.
- `git diff --check a3dba5e..HEAD`: pass.

## Result

- pass at `10a93e898d618bc30144189d5e344992706ddf04`.

## Pointers

- `docs/agents/reviews/2026-09-05_1658_tcig-4-closure-review.md`
- `docs/agents/discuss/2026-09-05_1619_tcig-4-insertion-corridor.md`
- `docs/status/evidence/true_container_inner_geometry/d567ad52571abb2d351c06f44c709f713a0de97f/g4/`
- worktree `/home/adamliao/work/elfin_humble_ws_eng_tcig4` branch `agent/eng/tcig-4`
