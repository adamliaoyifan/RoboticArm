# 2026-09-10 -- Container awareness clarification

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The current ROS 2 place chain has static prior awareness rather than reliable online container awareness. MoveIt receives a prior-positioned container collision mesh, waypoint generation receives a prior opening pose, and packing receives the configured hull plus planned placement commits; however, sensed opening estimates do not drive ROS 2 place waypoints, the cargo-map node explicitly does not integrate sensor points, candidate selection has no active reachability score, and post-place physical verification is not yet accepted. Deployment configuration should expose one coherent site profile at the composition boundary, but configuration alone is insufficient: runtime localization, scene updates, uncertainty margins, motion-feasibility filtering, and measured post-place commit are required for scene generalization.

## Risks

- Exposing duplicated per-node coordinates would increase drift; nodes should derive geometry from one versioned scene authority.
- A physically moved container leaves the static TF, collision mesh, opening waypoints, and committed occupancy mutually consistent in software but collectively wrong relative to reality.
- Planned-only occupancy compounds pose errors over successive placements.

## Pointers

- `src/luggage_planning/scripts/scene_manager_node.py`
- `src/luggage_planning/scripts/waypoint_generator_node.py`
- `src/luggage_perception/scripts/cargo_volume_mapper_node.py`
- `src/luggage_packing/scripts/placement_planner_node.py`
- `docs/architecture/container_geometry.md`
- `docs/architecture/production_orchestration.md`
