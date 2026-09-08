# 2026-09-04 - Pickup height scope clarification

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The required decoupling is narrower than the earlier support-calibration plan:
online pickup height should come from the observed luggage top plane, while a
deployment-provided `platform_z` may remain as optional input for full box
height/center reconstruction. Simulation truth is allowed in eval modules but
must not enter online function nodes. Platform tilt is out of scope.

The previously proposed empty-platform calibration estimates the support plane
height and pickup workspace, not camera intrinsics, hand-eye extrinsics, or the
luggage top. Under the clarified scope it should be optional validation of the
configured support height, not a prerequisite for detection.

## Recommended Contract

- Primary observed result: top-plane Z/contact pose, XY center, yaw, width,
  depth, confidence, acquisition stamp, and frame. These are computed from the
  depth/semantic cloud without reading scene truth or using `platform_z` as a
  hard point-removal threshold.
- Optional derived result: `height = top_z - platform_z` and
  `center_z = top_z - height/2`. A missing or suspect platform height may lower
  geometry confidence, but must not invalidate an otherwise valid pickup
  surface observation.
- Eval-only truth: Gazebo spawner/scene state may score online output. For real
  bags, reconstructed or manually measured references remain offline and must
  not be replayed into the online algorithm graph as inputs.
- Optional support validation: when an empty-platform observation is available,
  fit its Z and compare it with deployment `platform_z`; report a mismatch but
  do not make this calibration the source of luggage top height.

## Current-Code Consequence

`estimate_box()` currently uses `platform_z` both to remove points before top
plane fitting and to derive height/center. Split those responsibilities. The
first implementation should remove the hard pre-fit dependency while retaining
the configured value only for derived box geometry and plausibility checks.

## Pointers

- `docs/plans/sim_real_parity_pickup_support.md`
- `src/luggage_perception/luggage_perception/luggage_box_estimator.py`
- `src/luggage_perception/scripts/luggage_detector_node.py`
