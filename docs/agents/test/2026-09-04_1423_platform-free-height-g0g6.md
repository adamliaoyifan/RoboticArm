# 2026-09-04 — platform-free height Gates 0-6

- role: test
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- checkpoint: G0-G6
- revision: 0674f84-wt

## Summary

Executed Gates 0-6 against eng E0-E5 (`0674f84` + working tree). G1–G3
synthetic/integration passed. G0 fails the live pick contract:
`pick_from_detected` drops `height_valid` / `top_surface_pose`. G4 30-trial
Gazebo eval (`use_semantic:=false`, `support_mode=auto`, `platform_z` omitted)
spawned all 30 boxes but produced 333 TOP_ONLY frames of the 0.86 m platform
plane (0 FULL_3D, 0 false `height_valid=true`). G5 has no bag. G6 Hz 1.92 < 4;
`stop_sim.sh` left zero residual processes.

## Commands

- `colcon build --packages-select luggage_msgs luggage_perception luggage_planning luggage_packing luggage_gazebo`
- `colcon test --packages-select luggage_perception luggage_planning luggage_packing luggage_msgs` (801 passed)
- `python3 scripts/platform_free_height_gate1_metrics.py --out .../gate1`
- `python3 -m pytest src/luggage_perception/test/test_platform_free_pipeline.py`
- `ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false use_semantic:=false ... sequence_ids:=carryon,standard,large observe_pose_name:=pickup_observe support_mode:=auto`
- `python3 scripts/platform_free_height_gate4_eval.py --trials 30 --settle-sec 3.5`
- `scripts/stop_sim.sh`

## Evidence

- `docs/status/evidence/platform_free_height/2026-09-04_1412_g0g6/`

## Result

- fail: G0 live pick adapter; G4 online accuracy (raw-only); G6 rate.
- pass: G1 metrics (top Z max 0.32 mm, height max 0.30 mm); G2/G3 unit; no false measured height; no detector GetCurrentBox client.
- inconclusive: G5 (no rosbag).

## Pointers

- `docs/agents/eng/2026-09-04_1216_platform-free-height-e0-e5.md`
- `docs/plans/platform_free_height_test_plan.md`
- `src/luggage_planning/luggage_planning/ros_message_adapters.py`
- `docs/agents/discuss/2026-09-04_1143_platform-free-height-eng.md`

## Open

- Eng: forward E0 fields through `pick_from_detected`.
- Eng: Gate 4 must be re-run with semantic cargo (raw-only fitted the platform).
