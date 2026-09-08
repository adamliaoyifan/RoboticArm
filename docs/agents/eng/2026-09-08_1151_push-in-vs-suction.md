# 2026-09-08 -- Push-in vs suction place for interior fill

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

Interior volume is geometrically packable; the current atlas measures tool-down suction poses, not box occupancy. A gripper that can push along the opening axis can put a box deeper than the arm can hold it, but that is a new contact skill (force, unknown occupancy, stack disturbance) and does not authorize weaving between boxes. The cheaper related lever is still lateral insertion / lower transit on the existing suction stack.

## Pointers

- `src/luggage_packing/luggage_packing/packing_replay.py` (`unlock_floor_atlas`)
- `docs/plans/corridor_constraints.md`
- `src/luggage_planning/data/reachability_atlas/s20_container_collision_aware.yaml`
