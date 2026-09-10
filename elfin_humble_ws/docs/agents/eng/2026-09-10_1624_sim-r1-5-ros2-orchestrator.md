# 2026-09-10 -- SIM-R1-5 ROS 2 production orchestrator

- role: eng
- agent: codex-sim-r1-eng
- model: gpt-5.6-sol
- cli: codex
- status: done
- parent: SIM-R1-20260904
- subtask: SIM-R1-5
- base_revision: ebdf4be468ed1efb7419391ca99a2930b1ff0b30
- started_at: 2026-09-10T15:36:06+08:00
- completed_at: 2026-09-10T16:24:36+08:00

## Summary

Migrated `luggage_bringup` to ament, preserved the ROS 1 orchestrator as uninstalled reference code, and added thin ROS 2 production/operator adapters with an explicit Start boundary.

## Requirement

- Remain motionless before deliberate correlated Start, reject stale or duplicate operator events, preserve vacuum on carrying faults, and expose stable status/failure reasons.
- Keep production launch free of simulation, ground-truth, spawn, clear-box, and fake-backend shortcuts.

## Changed

- `src/luggage_bringup/CMakeLists.txt`
- `src/luggage_bringup/package.xml`
- `src/luggage_bringup/luggage_bringup/production_adapter.py`
- `src/luggage_bringup/luggage_bringup/production_orchestrator.py`
- `src/luggage_bringup/scripts/operator_control.py`
- `src/luggage_bringup/launch/production_orchestrator.launch.py`
- focused pure, static, and isolated ROS graph tests

## Verification

- SIM-R1 contract suite: 14 passed, 14 subtests passed; bringup suite: 9 passed.
- Required three-package `colcon build` and `colcon test`: pass; 260 tests, 0 errors, 0 failures.
- The isolated ROS graph remained at zero reset, exploration, motion, named-pose, and vacuum effects for 60.21 seconds before explicit Start.
- `git diff --check`: pass.

## Result

- pass at `2df9d09d535ac075cc6f731b9bc7faa1129acc23`.

## Pointers

- `docs/agents/discuss/2026-09-10_1534_sim-r1-5-ros2-orchestrator-g2.md`
- `docs/status/evidence/sim_r1/98db923/r5/summary.md`
