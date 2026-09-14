# 2026-09-14 -- Lock EEF runtime configuration

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Aligned the compatibility camera YAMLs and the legacy mount verifier with the
canonical D555 plus Mid-360S EEF tree already landed in `6ae5cd3` and
`81c0bb3`. Simulation and hardware launch paths expand the same locked xacro
configuration; no GUI tuning or Gazebo run was used.

## Verification

- Targeted description/perception pytest set: 53 passed, 2 subtests passed.
- `verify_camera_mount_config.py`: passed using the full link6 to D555 chain.
- S20 xacro expansion plus `check_urdf`: passed.
- `git diff --check`: passed.

## Pointers

- `docs/architecture/eef_sensor_frames.md`
- `src/luggage_description/config/realsense_d435.yaml`
- `src/luggage_description/config/realsense_d435_mount.yaml.example`
- `src/luggage_bringup/scripts/verify_camera_mount_config.py`
