# 2026-09-05 -- TCIG-4 reacceptance

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: TCIG-20260904
- subtask: TCIG-4
- reviewed_revision: 10a93e898d618bc30144189d5e344992706ddf04
- implementation_revision: d567ad52571abb2d351c06f44c709f713a0de97f
- worktree: /home/adamliao/work/elfin_humble_ws_eng_tcig4

## Summary

Re-reviewed TCIG-4 after remediation and accepted implementation revision
`10a93e8`; all three earlier closure findings are resolved.

## Result

TCIG-4 is accepted at `10a93e898d618bc30144189d5e344992706ddf04`.

The three closure-review findings from
`docs/agents/reviews/2026-09-05_1658_tcig-4-closure-review.md` are resolved:

1. Empty hull-eroded payload-center intervals now fail closed before iterating
   placed boxes.
2. Waypoint hull sweep endpoints and payload yaw are both expressed in
   `container_link`; rotated-container behavior has a focused test.
3. `corridor_surface_max()` can hull-filter wedge-only AABB overlap when a
   geometry descriptor is present, and the ROS node passes that descriptor.

## Verification Run

- `python3 src/luggage_packing/test/test_insertion_corridor.py` passed, 13 tests.
- `python3 src/luggage_perception/test/test_corridor_audit.py` passed, 13 tests.
- `python3 src/luggage_planning/test/test_waypoint_generator.py` passed, 26 tests.
- Minimal probes confirmed empty interval blocks, wedge-only surface max is
  ignored, and geometry-frame yaw accepts while world-frame yaw rejects the
  rotated-container fixture.
- `python3 -m unittest discover -s src/luggage_packing/test -p 'test_*.py'`
  passed, 80 tests.
- `python3 -m py_compile` for the four touched modules passed with
  `PYTHONPYCACHEPREFIX=/tmp/tcig4_review_pycache`.
- `git diff --check a3dba5e..HEAD` passed.

## Remaining Risk

Full perception discover was not rerun because existing evidence says it pulls
optional YOLO weights. This is not a TCIG-4 acceptance blocker; integration can
cover package-level wiring later.

## Pointers

- `docs/agents/reviews/2026-09-05_1658_tcig-4-closure-review.md`
- `docs/agents/discuss/2026-09-05_1619_tcig-4-insertion-corridor.md`
- `docs/plans/true_container_inner_geometry.md`
