# 2026-10-05 -- Atlas query yaw audit

- role: eng
- agent: cursor
- model: grok-4.7
- cli: cursor
- status: done

## Summary

The offline matrix queries container-frame tool yaw, the same frame the
atlas builder stores. World yaw is a different directed bin. No planner
change.

## What was compared

Deploy container yaw in world is pi. The 0.70 m payload artifact records
container yaw in base_link as `-1.5708` and `yaw_semantics:
directed_container_2pi`. For each directed bin the probe builds the matrix
goal quaternion (`route_matrix_eval.TOOL_DOWN_QUAT`, which is the pi
tool-down pose) and recovers yaw three ways.

- Matrix: `atan2` of the goal rotation minus the world container yaw.
- Production: `tool_down_yaw(x, y)` minus the world container yaw.
- Builder: `tool_target_in_base` round-trip of that recovered yaw.

Those three agree. The world tool yaw, with no subtraction, is pi away and
hits another bin. The matrix heading parameter is also pi away from the
query yaw, because its tool-down quaternion is already the pi pose; the
query uses the quaternion, not that parameter.

On atlas cell `(0, 2, 2)` of the 0.70 m payload, heading 0 queries yaw
index 2 (3 contact seeds). The uncorrected world yaw queries index 0
(2 contact seeds). Both bins return seeds, so the old matrix having
identical atlas-on and atlas-off outcomes is not a missed yaw frame.

## Evidence

`docs/status/evidence/frame_migration/ST09-parallel-I-s5/atlas_yaw_audit_20261005/probe.json`

## Pointers

- `scripts/st09_i/atlas_yaw_probe.py`
- `src/luggage_planning/luggage_planning/atlas_frame_contracts.py`
- `scripts/st09_i/route_matrix_eval.py`
