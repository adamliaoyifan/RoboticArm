# 2026-09-04 15:08 — PF-R1 complete ROS adapter contract

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R1
- base_revision: 0674f84 (working tree)
- started_at: 2026-09-04 15:02 local
- completed_at: 2026-09-04 15:08 local

## Summary

`pick_from_detected` dropped every E0 field, so the live pick path
(`waypoint_generator_node -> build_sequence -> pick_contact_top_z`)
raised `DETECT_FULL_GEOMETRY_REQUIRED` on every detection (the G0
failure). The adapter now carries the complete contract through as
plain planning attributes.

## Changed

- `src/luggage_planning/luggage_planning/ros_message_adapters.py`:
  `pick_from_detected` preserves acquisition stamp/frame,
  `top_surface_pose` (converted to the planning `Pose` type, not the
  mutable ROS message), `top_surface_valid/confidence`,
  `height_valid/confidence`, `height_source`, and `aspect_ratio`.
- `src/luggage_planning/test/test_pf_g0a_adapter_contract.py` (new):
  PF-G0A suite.

## Verification

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/luggage_planning/test/test_pf_g0a_adapter_contract.py \
  src/luggage_planning/test/test_ros_message_adapters.py \
  src/luggage_planning/test/test_waypoint_generator.py -q   # 32 passed
cd src/luggage_planning && python3 -m pytest test/ -q        # 221 passed
cd ../luggage_packing && python3 -m pytest test/ -q          #  75 passed
```

## Requirement

- Real generated `DetectedLuggage` converts with every E0 field intact
  (verified with values that distinguish top vs center-derived Z).
- `pick_contact_top_z` uses `top_surface_pose.z` through the adapter;
  `build_sequence` pick segment Z = top Z + pre_grasp clearance.
- Prior-only / top-only / default messages: pick Z either comes from the
  measured top or raises; positive numeric height never substitutes for
  `height_valid=true` (placement gate consumes the preserved flag).
- Boundary audit: `pick_from_detected` is the only DetectedLuggage ->
  planning-object conversion; eval drivers consume the message directly.

## Result

- pass: PF-G0A green (6 tests), affected package regressions green.

## Pointers

- `docs/plans/platform_free_height_remediation.md` (PF-R1/PF-G0A)
- `src/luggage_planning/test/test_pf_g0a_adapter_contract.py`

## Open

- None.
