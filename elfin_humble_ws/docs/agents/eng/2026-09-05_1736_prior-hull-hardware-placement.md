# 2026-09-05 -- Prior hull vs hardware collision and placement

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

Calibrated prior hull plus `container_link` is the intended wall model for sim
and hardware. It is not a collision safety net. A wrong TF or YAML is a
common-mode failure: packing, the cargo map, and MoveIt STL all move with the
same frame and can still hit the real container. Perception error hits cargo
height and occupancy, not the wall model, until a validated sensed hull exists.
Online placement currently picks the highest 2.5D score among gated candidates;
hardware needs a residual-budgeted hull margin before tightening density.

Walls use the TCIG descriptor from calibrated YAML or a later validated sensor
estimate, not observed free space. `container_link` locates that descriptor and
the collision STL; YAML inner bounds do not generate MoveIt geometry. Hull
margin on all seven faces must exceed measured TF plus box-size plus control
error. Empty-map floor prior is the highest-risk first placement. Select with
hard gates first, then rank support, observed confidence, and compactness.

## Pointers

- `docs/architecture/container_geometry.md`
- `docs/plans/true_container_inner_geometry.md`
- `src/luggage_description/config/scene_tf.yaml`
- `src/luggage_packing/scripts/placement_planner_node.py`
- `src/luggage_packing/luggage_packing/placement_solver.py`
