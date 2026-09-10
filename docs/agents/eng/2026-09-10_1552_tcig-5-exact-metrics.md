# 2026-09-10 -- TCIG-5 exact G5 metrics

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done
- parent: TCIG-20260904
- subtask: TCIG-5
- base_revision: 4425f227a74b789944e002a1adc8b2144553116c
- started_at: 2026-09-10T15:27:20+08:00
- completed_at: 2026-09-10T15:56:00+08:00

## Summary

G5 now takes exact usable volume and floor area from a normalized TCIG-1
descriptor. The four migrated packing entry points no longer use rectangular
`4.344` or `1.49*1.97`. Replay floor and reachability fields are labeled
`legacy_non_authoritative` until TCIG-6 owns them.

## Requirement

- Add `src/luggage_packing/luggage_packing/geometry_metrics.py` as the only
  G5 adapter from a normalized TCIG-1 descriptor to exact usable volume and
  floor area.
- Migrate `pack_eval_driver.py`, `active_loading_bag_harness.py`,
  `multi_box_gazebo_matrix.py`, and `packing_replay_eval.py`.
- Checked-in seven-face hull: volume `4.22433625 m^3`, floor `2.28715 m^2`.
  Cuboid fallback is `L*W*H` / `L*W`. Never fall back to rectangular `4.344`
  or `1.49*1.97`.
- `volume_fraction` is packed box volume divided by exact hull volume.
- `floor_coverage` includes a box only when its bottom is within
  `FLOOR_CONTACT_TOL_M = 1e-3` of TCIG `floor_z`, clips the footprint to the
  exact floor polygon, and unions overlapping footprints.
- Reports include numerator, denominator, schema version, and geometry hash.
- `packing_replay_eval.py` recomputes authoritative `volume_fraction` from
  `V_placed`. Replay `floor_coverage`, `V_container`, `overall_fill_rate`,
  `reachable_volume_ratio`, and `reachable_fill_rate` are
  `legacy_non_authoritative`. Do not edit `packing_replay.py`.
- `luggage_bringup` stays COLCON_IGNORE; add ROS-independent direct tests.
  No Gazebo runtime required.

## Changed

- `src/luggage_packing/luggage_packing/geometry_metrics.py`
- `src/luggage_packing/test/test_geometry_metrics.py`
- `src/luggage_packing/scripts/packing_replay_eval.py`
- `src/luggage_gazebo/scripts/pack_eval_driver.py`
- `src/luggage_gazebo/CMakeLists.txt`
- `src/luggage_gazebo/package.xml`
- `src/luggage_gazebo/test/test_pack_eval_g5_metrics.py`
- `src/luggage_bringup/scripts/active_loading_bag_harness.py`
- `src/luggage_bringup/scripts/multi_box_gazebo_matrix.py`
- `src/luggage_bringup/test/test_tcig5_bringup_metrics.py`
- `docs/status/evidence/tcig5/2026-09-10_tcig-5-g1/`

## Verification

- `pytest src/luggage_description/test -q`: 143 passed.
- `pytest src/luggage_packing/test -q`: 85 passed.
- `pytest src/luggage_bringup/test -q`: 3 passed.
- `pytest src/luggage_gazebo/test/test_pack_eval_g5_metrics.py -q`: 3 passed.
- `pytest src/luggage_gazebo/test -q` with `luggage_msgs` overlay: 58 passed.
- `colcon test --packages-select luggage_description luggage_packing luggage_gazebo`: 288 tests, 0 failures.
- `git diff --check`: clean.
- `scripts/check_agent_contract.sh`: passed in the primary mailbox tree. The isolated worktree `OPEN.md` at `4425f22` still lists DSIM rows whose threads were not on that commit.

## Result

- pass at `fcdc3e711bd730b76f0a20e73840f52320d6b892`.

## Pointers

- `docs/agents/discuss/2026-09-10_1524_tcig-5-exact-metrics-g1.md`
- `docs/plans/independent_ready_wave_20260910.md`
- `docs/status/evidence/tcig5/2026-09-10_tcig-5-g1/`
- `src/luggage_packing/luggage_packing/geometry_metrics.py`
