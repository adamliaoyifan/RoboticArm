# 2026-11-12 -- Current planning margin audit

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

Read-only source audit at50fd309d answers the user margin question. The
preserved native resolution comparison used collision_margin_m=0.05m.
Prior service, complete-operation preparation and runtime revalidation source
defaults are also0.05m; no current live deployment parameter override was read.
Actual volume collision conversion adds twice this margin to each of XYZ
size, preserving centres: each face moves outward5cm, so a2cm voxel becomes
12cm per unmerged axis. Native run-merging can produce longer primitives.
Both occupied and unknown blocked runs receive this inflation. Surface-only
fallback applies XY inflation only; do not generalize it to the volume path.

Attachment policy separately declares yaw half range5deg, size growth10% on
W/D/H, XY position half ranges0m, away-from-contact position bound5mm.
Parallel contact/box axes: W/D grow symmetrically, height grows away from
contact. Nonparallel axes conservatively union both possible height sides.
Yaw and measured tilt bounding can further enlarge the resulting envelope.
These are diagnostic policy bounds, explicitly not statistically calibrated
sensor or physical error guarantees. XY=0 means no extra policy allowance,
not zero measured sensor error. Release gap5cm is a separate target offset.
No parameter, geometry, margin or trajectory was changed; no new experiment.

## Pointers

- `src/luggage_planning/luggage_planning/joint_operation_prepare_node.py`
- `src/luggage_planning/luggage_planning/joint_operation_runtime_ports.py`
- `src/luggage_planning/luggage_planning/placement_prior_snapshot.py`
- `src/luggage_planning/launch/placement_prior.launch.py`
- `src/luggage_planning/config/attachment_geometry.yaml`
- `st09_contracts/payload_envelope.py`
- `src/luggage_planning/luggage_planning/route_producer.py`
- `docs/agents/eng/2026-11-12_1206_release-occupancy-resolution.md`
