# 2026-09-04 - Platform-free box height estimation

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Without a configured `platform_z`, full box height is observable from a
top-down RGB-D frame only when the support surface is also visible. The
recommended online algorithm jointly estimates the luggage top from semantic
cargo points and a local support plane from the same-stamp raw depth points
around the dilated luggage mask. Height is their vertical separation. If the
support plane is not observable, the detector must return a top-surface pose
and width/depth with `height_valid=false`, rather than deriving height from the
lowest visible cargo point.

## Recommended Algorithm

1. Exact-join the semantic mask/cargo cloud and preprocessed raw cloud by the
   acquisition stamp.
2. Fit the luggage top plane and robust 2-D rectangle from cargo points.
3. Form a support annulus outside the dilated luggage mask but inside the
   configured pickup workspace; fit horizontal plane candidates there.
4. Select the plane below the top that is spatially adjacent to the luggage
   footprint and passes residual, coverage, and temporal-stability gates.
5. Compute `height = top_z - support_z` and
   `center_z = (top_z + support_z) / 2`; width/depth come from the top rectangle.
6. Preserve `top_z` as the primary pick input. Support/height failure does not
   invalidate a confident top-surface detection.

## Fallback Policy

- Additional settled camera views may improve support visibility and side
  extent, but cannot recover an occluded bottom without a support assumption.
- Catalog/class dimensions may fill height only as an explicit low-confidence
  prior.
- `min(visible_object_z)` is not an accepted height source for a top-down view.
- Gazebo scene/spawner truth remains eval-only and may score both top and
  reconstructed full dimensions.

## Interface Consequence

`DetectedLuggage` currently has no `height_valid`, height confidence, or
explicit top-surface pose. Before enabling top-only success, the interface must
represent those semantics so downstream collision planning cannot treat a
guessed height as measured geometry.

## Pointers

- `src/luggage_perception/luggage_perception/luggage_box_estimator.py`
- `src/luggage_perception/scripts/semantic_point_filter_node.py`
- `src/luggage_perception/scripts/luggage_detector_node.py`
- `src/luggage_msgs/msg/DetectedLuggage.msg`
- `src/luggage_planning/luggage_planning/waypoint_generator.py`
