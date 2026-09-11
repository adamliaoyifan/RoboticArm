# TCIG-4 Gate G4 review amendments

- parent: TCIG-20260904
- subtask: TCIG-4
- generation: 1
- base_revision: a3dba5e7e0c2fb219062cfb2a091267f6f2d606d
- plan_revision: bd942eba3120cf521010c5ba628b2489f5e31546
- reviewed_revision: 37c157f5f1f66a8f103d9d54d49387ea003b319e
- output_revision: d567ad52571abb2d351c06f44c709f713a0de97f
- worktree: /home/adamliao/work/elfin_humble_ws_eng_tcig4
- branch: agent/eng/tcig-4

## Review findings addressed

1. High: `corridor_blocked()` fail-closes on an empty hull-eroded payload-center
   interval before iterating the ledger. `boxes=[]` cannot make an impossible
   corridor look free.
2. High: waypoint hull sweeps use `payload_yaw` in `container_link`. The ROS
   node converts world slot yaw minus container world yaw. A rotated-container
   test proves world-frame default yaw rejects a safe non-square chamfer sweep
   and geometry-frame yaw 0 accepts it.
3. Medium: `corridor_surface_max()` ignores wedge-only AABB overlap when a
   geometry descriptor is present. Audit and the ROS node pass the TCIG-1
   descriptor so carry height matches obstacle filtering.

## Tests

```text
PYTHONPATH=src/luggage_description:src/luggage_packing:src/luggage_perception:src/luggage_planning
python3 src/luggage_packing/test/test_insertion_corridor.py          # 13 passed
python3 src/luggage_perception/test/test_corridor_audit.py           # 13 passed
python3 src/luggage_planning/test/test_waypoint_generator.py         # 26 passed
python3 -m unittest discover -s src/luggage_packing/test -p 'test_*.py'  # 80 passed
python3 -m py_compile insertion_corridor.py corridor_audit.py waypoint_generator.py waypoint_generator_node.py
git diff --check a3dba5e..HEAD
```

Full perception discover was not used because it pulls optional YOLO weights.
The unclosed launch-file ResourceWarning is gone.

## Notes

- Same generation as the original TCIG-4 claim: scope, owner, base, and plan
  revision are unchanged.
- Cuboid callers that omit geometry still build a rectangular hull; AABB-only
  `corridor_surface_max` remains for those tests.
