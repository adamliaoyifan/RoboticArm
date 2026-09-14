# 2026-09-14 -- D555 canonical integration

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Selectively integrated the D555 transport work from `origin/ros2_humble`
without importing its nested duplicate workspace. The live D555 now feeds the
same canonical preprocessor as simulation through a raw-driver backend with
JPEG/lossless-PNG transport, mapped acquisition timestamps, explicit clock
evidence, single-boundary decode, and an enabled motion gate.

The calibrated camera and Livox parameters from upstream commits `e44048a`,
`d88807a`, and `faf3807` were already present byte-for-byte on `master` via
`e93229b`, `6f71c18`, and `88ae2db`; they were verified and not duplicated.

## Changed

- `src/luggage_perception/launch/d555_canonical_pipeline.launch.py`
- `src/luggage_perception/scripts/d555_transport_adapter_node.py`
- `src/luggage_perception/luggage_perception/device_clock.py`
- `src/luggage_perception/luggage_perception/rgbd_codec.py`
- `src/luggage_perception/config/preprocessor_d555_live.yaml`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/motion_compensation.md`

## Verification

- Focused D555, adapter, clock, preprocessor, and motion tests: 105 passed.
- `colcon build --packages-select luggage_perception --symlink-install`: pass.
- Installed ROS 2 launch argument parsing: pass.
- Live hardware validation: deferred until the D555/site LAN is available;
  exact required evidence is recorded in the result file.

## Pointers

- `docs/agents/reviews/2026-09-14_1155_canonical-vs-real-dataflow-decision.md`
- `docs/status/evidence/d555_canonical_integration/2026-09-14/RESULT.md`
- `origin/ros2_humble@48e034b8a0445bf841003938d9dd82f7c64961a1`
