# TCIG-4 Gate G4

- parent: TCIG-20260904
- subtask: TCIG-4
- base_revision: a3dba5e7e0c2fb219062cfb2a091267f6f2d606d
- plan_revision: bd942eba3120cf521010c5ba628b2489f5e31546
- output_revision: bbbcf7a949ea68554ca269fc1dd724ac5c730a79
- worktree: /home/adamliao/work/elfin_humble_ws_eng_tcig4
- branch: agent/eng/tcig-4

## Gate

- Low-Z wall spanning the eroded hull width blocks a deep EMS even when it does
  not span nominal AABB width.
- Wedge-only broad-phase overlap is not an obstacle (packing and audit).
- Swept payload crossing the slanted face is rejected before motion segments
  are returned.
- Safe center, side, and stacked high-Z corridors remain accepted.
- Existing cuboid obstacle, carry-height, aperture, and packing tests stay green.

## Tests

```text
PYTHONPATH=src/luggage_description:src/luggage_packing:src/luggage_perception:src/luggage_planning
python3 src/luggage_packing/test/test_insertion_corridor.py          # 12 passed
python3 src/luggage_perception/test/test_corridor_audit.py           # 13 passed
python3 src/luggage_planning/test/test_waypoint_generator.py         # 25 passed
python3 -m unittest discover -s src/luggage_packing/test -p 'test_*.py'  # 79 passed
python3 -m py_compile insertion_corridor.py corridor_audit.py waypoint_generator.py waypoint_generator_node.py
git diff --check
```

Additional planning/perception unit files around vacuum, atlas, floor geometry,
box geometry, and stamp windows also passed. Full perception discover was not
used because it pulls optional YOLO weights.

## Notes

- Opening side is fail-closed except `negative_x`.
- `corridor_audit` imports `insertion_corridor` so both call the same TCIG-1
  operations; perception `package.xml` gained an exec_depend on packing.
- Packing still accepts omitted geometry as a cuboid from `inner_size` so
  existing rectangular callers keep working.
