# 2026-09-10 -- Deployment place parameter audit

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Static audit finds that the ROS 2 placement math is container-relative and can follow a correctly updated shared `scene_tf`, but the current repository is not yet hardware-deployment-ready. Site motion without recalibration can misalign planning, collision geometry, and execution; sensed opening geometry is not consumed by the ROS 2 place waypoint path; ROS 2 placement launch does not pass its selected `scene_tf_config` into `placement_planner`; and closed-loop placement acceptance remains open.

## Risks

- Hardware launch still permits the simulation example scene instead of requiring an explicit surveyed site config.
- Corrected container pose can move otherwise valid slots outside practical reachability; the ROS 2 solver selects by packing score and reports no reachability score or alternate-candidate motion retry.
- Static opening pose, aperture, inner hull, robot/pedestal pose, tool/camera extrinsics, payload geometry, and motion clearances are deployment inputs and must be validated as one coherent revision.
- The current solver assumes horizontal placement and axis-aligned `0/90` degree footprints; platform or container tilt is outside the normative geometry scope.

## Pointers

- `src/luggage_description/config/scene_tf.yaml.example`
- `src/luggage_description/launch/scene_hardware.launch.py`
- `src/luggage_packing/scripts/placement_planner_node.py`
- `src/luggage_planning/scripts/waypoint_generator_node.py`
- `src/luggage_gazebo/launch/sim_world.launch.py`
- `docs/architecture/container_geometry.md`
- `docs/agents/discuss/2026-09-10_1445_pf-r10-g3-closed-loop-place.md`
