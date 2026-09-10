# 2026-09-10 -- Deployment place awareness review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The current ROS 2 placement chain is primarily initialized by static scene geometry and records planned placement commits rather than continuously estimating container and cargo state. Improve it by retaining CAD, kinematics, calibration, and safety policy as versioned priors while adding sensed container localization, uncertainty-aware scene updates, live cargo integration, reachability-aware candidate selection, and measured post-place verification.

## Priorities

- P0: require one explicit measured deployment profile and verify identical geometry identity across all consumers.
- P1: estimate `base_link -> container_link` and the aperture online, then update TF and MoveIt collision geometry only through a gated, versioned scene snapshot.
- P1: verify actual box pose after release and commit measured occupancy instead of the requested slot.
- P2: filter and rank multiple placement candidates using IK, full swept-volume collision checks, and bounded alternate-candidate retries.
- P2: integrate settled live depth into the container occupancy model and represent unknown space explicitly.
- P3: propagate calibration and perception uncertainty into aperture, hull, support, and clearance margins.

## Risks

- Online perception must not directly move static TF or collision objects without confidence, freshness, jump, and consistency gates.
- Removing geometry priors entirely would reduce safety; container CAD, robot kinematics, tool geometry, sensor calibration, and hard motion limits remain required priors.
- Container/platform tilt and arbitrary non-convex container geometry require an explicit architecture extension beyond the current container geometry contract.

## Pointers

- `docs/architecture/container_geometry.md`
- `docs/architecture/production_orchestration.md`
- `src/luggage_perception/scripts/cargo_volume_mapper_node.py`
- `src/luggage_perception/luggage_perception/container_opening_estimator.py`
- `src/luggage_packing/scripts/placement_planner_node.py`
- `src/luggage_planning/scripts/scene_manager_node.py`
- `src/luggage_planning/scripts/waypoint_generator_node.py`
