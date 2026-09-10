# 2026-09-09 -- D555 simulation depth-path readiness

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The D555 hardware evidence is sufficient to start retargeting simulation from
transported Gazebo XYZ to the canonical depth-image path. No additional
housing-to-left-IR dimension is needed: simulated geometry should be
reconstructed from the colour-aligned depth grid and its `CameraInfo`, then
transformed from the truthful optical frame to world at the acquisition stamp.

The current ROS 2 tree already contains most of this design through the
in-progress PF-R9 generation-2 implementation: Gazebo depth is adapted from
`32FC1` metres to `16UC1` millimetres, the preprocessor transports paired
images rather than camera points, semantic cargo points are deprojected after
pixel selection, and detector support points are locally deprojected before a
stamped TF lookup. The remaining sim work is to remove the Gazebo points
bridge and stale consumers/probes, retarget the camera profile to D555, and
verify the complete no-camera-cloud graph.

The profile change cannot be isolated from observation geometry. D555's
640x360 at 15 Hz profile and approximately 0.26 m near limit put the current
approximately 0.24 m tallest-box observation inside the blind zone. A passing
sim retarget must therefore re-qualify or change `pickup_observe`. HE-2's
calibrated mount transform remains necessary for final absolute world-frame
parity, but it is a later configurable transform replacement and does not
block the data-path work.

## Pointers

- `docs/architecture/sensor_data_pipeline.md`
- `docs/status/evidence/d555_bringup/20260908_1951_hb1/SUMMARY.md`
- `docs/status/evidence/d555_bringup/20260908_2004_hb2/SUMMARY.md`
- `docs/plans/pf_f3_depth_primary_contract.md`
- `src/luggage_gazebo/launch/sim_world.launch.py`
- `src/luggage_gazebo/scripts/depth_image_republisher.py`
- `src/luggage_perception/luggage_perception/depth_deprojection.py`
