# 2026-09-04 - Pickup support hardware plan review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The live empty-platform calibration direction is sound, but the current plan is
not implementation-ready. Two P0 gaps must be fixed first: a static support
record cannot detect its own wrong height, and the proposed simulation default
still uses scene/spawner truth instead of exercising the hardware-observable
calibration path. The data contract, lifecycle, launch owner, and quantitative
gates also need to be made explicit.

## Findings

- P0: Move the wrong-height rejection test from Slice 1 to live validation.
  Source, age, and configured confidence can reject a forbidden or expired
  record, but cannot prove that `measured_platform_z` matches the physical
  platform. Slice 1 can only guarantee fail-closed provenance; Slice 2 must
  compare the configured plane with independent live depth evidence.
- P0: Make `live_calibrated` the normal simulation acceptance path too.
  Calling the same `estimate_box()` function is insufficient parity while
  simulation obtains support from `pickup_source`. Keep `scene_config` only as
  an explicit oracle/baseline adapter, never the profile used to accept the
  production perception path.
- P0: Define support as geometry, not only `platform_z`. The contract needs a
  stamped support frame or plane equation, a bounded workspace polygon, fit
  residual/coverage, provenance, and calibration identity. If v1 only supports
  horizontal platforms, define a tilt limit and reject observations beyond it.
  Also resolve the contradiction between anisotropic
  `measured_roi_half_extent_xy` and the estimator's single square `roi_margin`.
- P1: Specify calibration lifecycle and ownership. A startup one-shot can
  observe an already spawned suitcase, a moving wrist, or one noisy frame.
  Require an EMPTY -> SETTLED -> COLLECTING -> VALID/INVALID state sequence,
  multi-frame consensus, stamped TF, motion/geometry gates, timeout,
  idempotency, and invalidation on TF/extrinsic change or excessive plane
  residual. Keep state in a pure algorithm class using `update()` and
  `copy_output()`; the ROS node should remain an adapter.
- P1: Name the ROS 2 hardware bringup target before Slice 3. The current
  `scene_hardware.launch.py` launches scene TF, robot state publisher, and RViz
  only. The detector's other bringup occurrence is the ROS 1
  `luggage_bringup` package, which is excluded from colcon. The plan therefore
  has no current ROS 2 hardware perception launch in which to enforce the
  support source.
- P1: Close the interface decision before implementation. Architecture requires
  every new service/topic to be defined through `luggage_msgs`; a
  detector-private ad-hoc service is not an allowed option. Define request,
  bounded timeout, repeat-call behavior, status/error codes, and how the
  stamped support result appears in diagnostics.
- P1: Make the plane search bounded and testable. "Dominant horizontal low
  plane" can select the floor. A calibrated pickup workspace and plausible Z
  interval must be mandatory inputs; plane inlier bounds should validate
  coverage, not redefine the pickup ROI from whichever surface is visible.
- P1: Replace qualitative acceptance with numeric gates and recorded replay.
  Include plane Z/normal/residual/coverage thresholds, N-frame stability,
  detection height and XY error, stale/missing/moved-platform failures,
  stamped-TF failure, sim profile parity, and at least one real or rosbag cloud
  whose platform is not 0.86 m. Store run artifacts under
  `docs/status/evidence/`.

## Recommended Order

1. Contract slice: define support geometry, provenance, error codes, lifecycle,
   and horizontal-only rejection limits; add pure validation tests.
2. Calibration slice: implement bounded multi-frame plane fitting from
   preprocessed stamped clouds and explicit state transitions.
3. Parity slice: run the same calibration and detection flow in Gazebo; retain
   scene truth only in eval code for scoring.
4. Hardware bringup slice: add or identify the ROS 2 perception launch/profile,
   require measured/live support, and reject missing calibration.
5. Evidence slice: replay non-0.86 m data, then run controlled hardware tests
   against numeric gates before enabling optional online refresh.

## Pointers

- `docs/plans/sim_real_parity_pickup_support.md`
- `docs/architecture/perception_architecture.md`
- `docs/architecture/sensor_data_pipeline.md`
- `src/luggage_perception/scripts/luggage_detector_node.py`
- `src/luggage_perception/luggage_perception/luggage_box_estimator.py`
- `src/luggage_description/launch/scene_hardware.launch.py`
- `src/luggage_bringup/COLCON_IGNORE`
