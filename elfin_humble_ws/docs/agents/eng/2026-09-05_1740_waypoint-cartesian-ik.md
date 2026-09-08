# 2026-09-05 -- Waypoint cartesian path and IK singularities

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

This workspace generates pick/place motion as world-frame
`suction_contact_frame` polylines, not a learned 6-axis trajectory model.
`waypoint_generator.build_sequence()` turns a 3D target into segments; MoveIt
KDL then solves IK. Named poses use joint FollowJointTrajectory. Free-space
`pose_target` uses OMPL. Vertical and corridor segments use
`GetCartesianPath` with 1 cm steps. Place `insert` / `descend` / `retreat`
forbid OMPL fallback.

Cartesian IK can hit Elfin wrist, elbow, or shoulder singularities, especially
tool-down inside the container. `jump_threshold` is `0.0`, so joint jumps are
not rejected. Near-singularity failure should reject the slot, not weave
between boxes. A later learned policy may replace `(h, y, yaw)` or a few SE(3)
waypoints only; collision, corridor, and IK stay hard filters.

## Pointers

- `docs/plans/corridor_constraints.md`
- `src/luggage_planning/luggage_planning/waypoint_generator.py`
- `src/luggage_planning/luggage_planning/motion_executor.py`
- `src/elfin_moveit_config/config/kinematics.yaml`
- `docs/plans/closed_loop_pick_retreat_nodes.md`
