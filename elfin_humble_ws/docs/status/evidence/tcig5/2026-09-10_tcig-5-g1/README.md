# TCIG-5 generation 1 evidence

- parent: TCIG-20260904
- subtask: TCIG-5
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- base_revision: 4425f227a74b789944e002a1adc8b2144553116c
- started_at: 2026-09-10T15:27:20+08:00

## Checked-in hull denominators

`src/luggage_description/config/container_inner_geometry.yaml` seven-face hull:

- usable volume: `4.22433625 m^3`
- floor area: `2.28715 m^2`

Cuboid fallback remains `L*W*H` / `L*W`. Rectangular `4.344` and `1.49*1.97` are not used.

## Commands

Workspace: `/home/adamliao/work/elfin_humble_ws_eng_tcig5`

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/luggage_description:$PWD/src/luggage_packing:$PWD/src/luggage_bringup:$PWD/src/luggage_gazebo${PYTHONPATH:+:$PYTHONPATH}"

pytest src/luggage_description/test -q
pytest src/luggage_packing/test -q
pytest src/luggage_bringup/test -q
pytest src/luggage_gazebo/test/test_pack_eval_g5_metrics.py -q

colcon build --packages-select luggage_description luggage_packing luggage_gazebo --symlink-install
source install/setup.bash
colcon test --packages-select luggage_description luggage_packing luggage_gazebo --event-handlers console_cohesion+
colcon test-result --packages-select luggage_description luggage_packing luggage_gazebo --verbose
```

`pytest src/luggage_gazebo/test` as a directory also passed after overlaying an existing `luggage_msgs` install (`/tmp/pfr10_g2_int/install` plus this worktree `install`).

## Results

| suite | result |
|---|---|
| `pytest src/luggage_description/test` | 143 passed |
| `pytest src/luggage_packing/test` | 85 passed |
| `pytest src/luggage_bringup/test` | 3 passed |
| `pytest src/luggage_gazebo/test/test_pack_eval_g5_metrics.py` | 3 passed |
| `pytest src/luggage_gazebo/test` (with msgs overlay) | 58 passed |
| `colcon test` description + packing + gazebo | 288 tests, 0 failures |
| `git diff --check` | clean |

## Scope

- Added `src/luggage_packing/luggage_packing/geometry_metrics.py`.
- Migrated `pack_eval_driver.py`, `active_loading_bag_harness.py`, `multi_box_gazebo_matrix.py`, `packing_replay_eval.py`.
- `packing_replay.py` was not edited.
- Replay `floor_coverage`, `V_container`, `overall_fill_rate`, `reachable_volume_ratio`, and `reachable_fill_rate` are labeled `legacy_non_authoritative`. G5 owns exact `volume_fraction` from `V_placed / V_exact`.
