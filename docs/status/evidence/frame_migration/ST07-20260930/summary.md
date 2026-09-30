# ST07 generation 2

Worktree `/tmp/elfin-frame-migration-st07`, branch
`integration/frame-migration-st07-20260929`. Base `daf291a` (plan) whose
parent `074daf3` is the accepted ST06 product head. No Gazebo, no
hardware motion, no master merge.

## What landed

- World TCP is `box_to_tool_target` then `transform_pose`.
  `slot_from_contract` uses that same transform.
- Occupancy samples the box-center polyline. Service waypoints are the
  TCP polyline. A full-height wall can add one `gap_detour`.
- `SelectPlacePathV2` and `BuildMotionSequenceV2` share
  `place_tcp.py`. v1 services keep their fields, the centered `h/2`
  shortcut, and the yaw-only quaternion.
- Grid setup still calls `query` with the slot yaw. `yaw_period_rad = π`
  raises `ATLAS_SEMANTICS_MISMATCH` before any cell is expanded.

## Batteries on this base, before the product edit

- planning 661 passed, 171 subtests
- packing 127 passed, 1 pre-existing failure
  (`test_matching_geometry_answers_with_its_hull_hash`)
- perception 1048 passed, 7 skipped, 1 pre-existing failure
  (`test_vacuum_geometry_is_classified_sim_backend`)
- description 187 passed
- migration checker: baseline 50/50 exit 0; migration exit 2 with 13
  pending (F01, F08, F11, F12, F13, F15, F16, F17, F18, F21, F22, F23, F24)

## Batteries after

- planning 676 passed (the previous 661 plus 15 new selector cases)
- packing 127 passed and the same pre-existing failure
- perception 1048 passed, 7 skipped, and the same pre-existing failure
- description 187 passed, including contracts 15/15
- migration checker: baseline 50/50 exit 0; migration exit 2 with the
  nine rows F01, F15, F16, F17, F18, F21, F22, F23, F24
- F08, F11, F12, F13 verified. F06 and F09 proofs were reissued because
  those rows already hashed `occupancy_place_paths.py` and
  `waypoint_generator.py`; their state stayed verified

## Stage suite

`test_frame_migration_selector.py`: 15 passed. Case log
`cases.jsonl` (89 lines). Fixture arrays under `fixtures/` and
`src/luggage_planning/test/data/frame_migration/`.
