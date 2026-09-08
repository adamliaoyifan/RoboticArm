# 2026-09-08 -- Trajectory feasibility vs slot selection

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

A geometrically legal slot is not a kinematically legal path. Do not train a joint-space policy. Keep the corridor polyline template; push MoveIt earlier as a filter; fail over to the next SlotSpec; optionally search a few template parameters (corridor height, yaw, wrist branch). Humble ComputePlacement currently returns one geometry winner without atlas or cartesian dry-run, which is why slots can be chosen and then PLAN_PLACE fails.

## Pointers

- `docs/plans/corridor_constraints.md`
- `src/luggage_packing/scripts/placement_planner_node.py`
- `src/luggage_planning/scripts/placement_motion_filter_node.py`
- `src/luggage_packing/luggage_packing/placement_reachability.py`
- `src/luggage_planning/luggage_planning/motion_executor.py`
- `docs/agents/eng/2026-09-05_1740_waypoint-cartesian-ik.md`
