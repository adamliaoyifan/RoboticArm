# 2026-09-04 — self-review of G0-G6 execution isolation

- role: test
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- checkpoint: G0-G6
- revision: 0674f84-wt

## Summary

Reviewed run `2026-09-04_1412_g0g6`. Unit/colcon tests were not pytest-xdist
parallel. colcon test ran packages one after another with per-package
cache dirs. ROS isolation for Gate 4 was one stack on `ROS_DOMAIN_ID=7` and
`ROS_LOCALHOST_ONLY=1`; eval correctly shared that domain. Gaps: first pytest
overlapped a background colcon build; one mixed-package pytest collided on
`test_ros_message_adapters`; Gate 6 Hz was taken on the loaded Gate 4 spawn
loop, not an idle perception profile. Gate 4 `use_semantic:=false` is a
procedure miss, not an isolation miss.

## Commands

- not re-run; reviewed evidence from `2026-09-04_1412_g0g6`.

## Evidence

- `docs/status/evidence/platform_free_height/2026-09-04_1412_g0g6/`

## Result

- pass: no dual Gazebo; stop_sim residual 0; colcon tests isolated per package.
- fail: G4/G6 not a clean isolated performance/semantic profile.

## Pointers

- `docs/agents/test/2026-09-04_1423_platform-free-height-g0g6.md`
- `docs/status/evidence/platform_free_height/2026-09-04_1412_g0g6/gate0/colcon_test.log`
- `docs/status/evidence/platform_free_height/2026-09-04_1412_g0g6/gate0/pytest_offline.log`
