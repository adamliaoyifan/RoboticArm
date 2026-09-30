# ST07 repair — service boundary

Parent commit `904a6de24b5ead98b963e005e885e23daadd053d` accepted stale
surface identity, all-invalid cells, a v1 grasp, and a missing stamp, and
raised `IndexError` on a truncated surface. This repair decodes
`SelectPlacePathV2` and `BuildMotionSequenceV2` through the existing v2
bindings before selection.

## Replay

`reproduction.json` uses the same generated-message mutations as
`ST07-audit-20260930`. Control still returns path hash `d2af5c264f917eef`.
The other six cases return before `select_place_v2`:

| Case | reason_code |
|---|---|
| stale map revision | `MAP_REVISION_MISMATCH` |
| wrong geometry hash | `GEOMETRY_IDENTITY_MISMATCH` |
| `valid=false` with floor z still set | `INVALID_GRID` |
| grasp `contract_version=1` | `CONTRACT_VERSION_MISMATCH` |
| slot `stamp_valid=false` | `INVALID_STAMP` |
| empty `state` | `INVALID_GRID` |

`valid=false` with canonical z=0 and `state=free` is
`UNKNOWN_SURFACE_HAS_VALUE`. A legal unknown cell decodes to `None`.

## Tests

- `test_frame_migration_selector.py`: 16 passed
- `test_select_place_path*.py`: 6 passed
- `test_trajectory_selector.py`: 14 passed
- `test_occupancy_place_paths.py`: 26 passed
- `test_frame_contracts.py`: 15 passed
- planning pytest: 677 passed, 171 subtests passed
- migration checker: baseline 50/50 exit 0; migration exit 2; pending
  F01, F15, F16, F17, F18, F21, F22, F23, F24

F08 and F13 proofs in this directory replace the `904a6de` proofs. F11 and
F12 are reissued here after the transform admission change. No Gazebo and no
hardware motion.

## Second repair

`container_transform_dict` now requires source `container_link`, target
`world`, three finite translation components, and a null or nonnegative
integer stamp. Those checks run inside `request_dict`, before
`select_place_v2`. `wrong_frame` returns `FRAME_CONTRACT_MISMATCH`.
Translation `[NaN, 0, 0]` returns `INVALID_NUMERIC_PAYLOAD`.

`BuildMotionSequenceV2` rejects any phase other than `pick` or `place`
with `INVALID_RECORD`, and a non-finite or non-positive `box_wdh` with
`INVALID_NUMERIC_PAYLOAD`, before `build_place_legs_v2`.

The v1 test calls `TrajectorySelectorNode._select` on one fixed
`SelectPlacePath` request, once on this tree and once on `074daf3`'s
occupancy, path candidates, and callback. The two responses reserialize
to the same CDR. Carry orientations keep z=w=0.
