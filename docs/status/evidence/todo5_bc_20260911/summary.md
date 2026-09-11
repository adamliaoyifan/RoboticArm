# Todo 5 B/C acceptance summary

- date: 2026-09-11T11:24:39+08:00
- base_revision: 81ef5c559d58a31622a6952e34104ea1aea88781
- simulation: not started

## Scope

- Cargo-map Add/duplicate Add/Remove/Reset, revision, ledger, occupied volume,
  and `surface_2d` height behavior.
- ROS-free ComputePlacement with fixed JSON cases for empty, existing box,
  exact boundary, aperture, hull, overlap, corridor, and no candidate.
- Existing `sim_world.launch.py` `use_cargo_map` / `use_packing` wiring.

## Results

- Focused mapper plus all packing tests: `99 passed in 1.61s`.
- All packing tests alone: `81 passed in 0.87s` before the final two fixture
  cases were added; those cases are included in the 99-test command above.
- `python3 -m py_compile` on both algorithm modules and both ROS nodes: pass.
- `git diff --check`: pass.
- `colcon build --packages-select luggage_msgs luggage_packing
  luggage_perception --symlink-install`: all selected packages built; the
  message package regenerated `GetCargoMapStats.occupied_volume`.
- Non-Gazebo ROS 2 smoke on isolated domains 79/80: mapper and planner became
  ready; empty stats reported revision 1 and zero occupancy; first Add reported
  `committed 1 boxes (rev 2)`; duplicate Add reported
  `already committed 1 boxes (rev 2)`; occupied volume became `0.061875`;
  Remove succeeded and stats returned zero occupancy at revision 3; Reset
  succeeded; ComputePlacement returned `success=True`, 672 feasible candidates,
  and an `outside_aperture` reject histogram.

## Broader-suite note

A broad perception-plus-packing collection reached 696 passes, then reported
five environment/pre-existing failures: three detector instrumentation tests
were run before the rebuilt `luggage_msgs` overlay was sourced, and two vintage
YOLO tests require CUDA while this execution environment exposes no GPU. The
focused acceptance suite above was rerun after sourcing the rebuilt overlay.
