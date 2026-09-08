# 2026-09-05 -- TCIG-4 closure review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: TCIG-20260904
- subtask: TCIG-4
- reviewed_revision: 37c157f5f1f66a8f103d9d54d49387ea003b319e
- implementation_revision: bbbcf7a949ea68554ca269fc1dd724ac5c730a79
- worktree: /home/adamliao/work/elfin_humble_ws_eng_tcig4

## Summary

Completed the TCIG-4 closure review at `37c157f`; acceptance was withheld and
three concrete amendments were required before integration.

## Findings

1. High - Empty hull-eroded center interval is accepted when there are no placed
   boxes. `corridor_blocked()` computes `payload_center_y_interval()`, but the
   fail-closed empty-interval check is inside the loop over `boxes`. With an
   empty ledger, a physically impossible insertion corridor returns `False`
   instead of blocked. This violates TCIG-4's hull-eroded corridor semantics.
   Minimal repro on the reviewed branch produced `interval (0.0, -0.95)` and
   `blocked_empty_boxes False`.
   File: `/home/adamliao/work/elfin_humble_ws_eng_tcig4/src/luggage_packing/luggage_packing/insertion_corridor.py:142`.

2. High - Waypoint hull sweep uses yaw in the wrong frame when the container is
   rotated. `waypoint_generator_node` converts sweep endpoints into
   `container_link`, then `build_sequence()` defaults `payload_yaw` to
   `slot_yaw` from the world-frame slot pose. TCIG-1 `contains_swept_box()`
   expects points and yaw in one geometry frame. For non-square payloads and
   non-zero container yaw, the sweep can reject/accept against the wrong
   oriented footprint. No TCIG-4 test covers a rotated container.
   Files:
   `/home/adamliao/work/elfin_humble_ws_eng_tcig4/src/luggage_planning/scripts/waypoint_generator_node.py:349`,
   `/home/adamliao/work/elfin_humble_ws_eng_tcig4/src/luggage_planning/luggage_planning/waypoint_generator.py:323`.

3. Medium - `corridor_surface_max()` still uses broad-phase AABB overlap only
   and is called by the ROS node without hull filtering. A committed box whose
   AABB lies only in the removed wedge is no longer an obstacle in
   `audit_corridor()`, but it can still raise `surface_max` / carry height in
   waypoint generation. This is not as severe as a false feasible placement, but
   it leaves corridor-height behavior inconsistent with the TCIG-4
   broad-phase-only rule.
   Files:
   `/home/adamliao/work/elfin_humble_ws_eng_tcig4/src/luggage_perception/luggage_perception/corridor_audit.py:60`,
   `/home/adamliao/work/elfin_humble_ws_eng_tcig4/src/luggage_planning/scripts/waypoint_generator_node.py:230`.

## Verification Run

- `python3 src/luggage_packing/test/test_insertion_corridor.py` passed, 12 tests.
- `python3 src/luggage_perception/test/test_corridor_audit.py` passed, 13 tests.
- `python3 src/luggage_planning/test/test_waypoint_generator.py` passed, 25 tests
  with one ResourceWarning from an unclosed fixture file.
- `python3 -m unittest discover -s src/luggage_packing/test -p 'test_*.py'`
  passed, 79 tests.
- `python3 -m py_compile` for the four touched modules passed after rerunning
  outside sandbox because the isolated worktree is outside the writable root.
- `git diff --check a3dba5e..37c157f` passed.

## Review Result

TCIG-4 is not accepted yet. The reported G4 evidence is real for the covered
cases, but the three findings above need amendments before TCIG-4 should feed
TCIG-6 or integration.

## Pointers

- `docs/agents/discuss/2026-09-05_1619_tcig-4-insertion-corridor.md`
- `docs/plans/true_container_inner_geometry.md`
