# Platform-free Height Estimation - Engineering TODO

Date: 2026-09-04

Source decision:
`docs/agents/reviews/2026-09-04_1111_platform-free-height-estimation.md`

## Goal

Estimate the luggage pickup top surface directly from RGB-D data. When the
support surface is visible in the same observation, estimate full box height
without reading `pickup_source.z` or another simulation truth value.

The online result has two validity levels:

- `TOP_ONLY`: top surface, XY, yaw, width, and depth are measured; height is
  unavailable.
- `FULL_3D`: the local support plane is also measured; height and center Z are
  valid.

Platform tilt is out of scope. Version 1 accepts only horizontal top/support
planes and rejects candidates outside the configured normal tolerance.

## Hard Boundaries

- Online function nodes must not read Gazebo spawner state, `GetCurrentBox`
  geometry, or scene truth to estimate top/support geometry.
- Eval code may read Gazebo truth and compare it with online outputs.
- Simulation and hardware run the same top/support estimators. Backend-specific
  code ends at sensor adapters and configuration.
- A deployment `platform_z` may remain as an optional prior or cross-check, but
  it must not remove points before top fitting or determine pickup contact Z.
- `min(visible_object_z)` is not a valid support or height estimate.
- Missing support must produce `height_valid=false`; it must not synthesize a
  full box silently.

## Target Data Flow

```text
preprocessed raw cloud ───────────────┐
                                     ├─ exact stamp join ─ local support fit
semantic mask ─ semantic cargo cloud ┘                         │
                 │                                             │
                 └─ top plane + top rectangle fit ─────────────┤
                                                               ▼
                              TOP_ONLY or FULL_3D detection result
                                                               │
                     ┌─────────────────────────────────────────┴──────┐
                     ▼                                                ▼
            pick uses top_surface_pose                  map/place checks height_valid
```

Only a fresh semantic `measure` frame may create `FULL_3D`. A `hold_track`
cargo cloud represents an earlier acquisition even if republished with a newer
stamp; it may retain a top-only tracked result but must not be fused with a new
raw cloud as if both were measured together.

## E0 - Interface Contract

Update `luggage_msgs/msg/DetectedLuggage.msg` so validity is machine-readable.
The exact field layout may follow local ROS conventions, but it must represent:

- acquisition `header` (`stamp`, `frame_id`);
- `top_surface_pose`, whose Z is the measured pickup contact surface;
- top-surface validity and confidence;
- `height_valid` and height confidence;
- height source enum: `UNAVAILABLE`, `MEASURED_SUPPORT`,
  `CONFIGURED_SUPPORT`, `CATALOG_PRIOR`;
- existing center pose and dimensions.

Contract rules:

- `top_surface_pose` is the sole Z input for pick waypoint generation.
- Center pose Z and `height` are geometric measurements only when
  `height_valid=true`.
- A catalog prior may populate a candidate height, but remains
  `height_valid=false` and cannot create collision geometry without a deliberate
  downstream policy.
- Existing consumers must be audited and updated atomically. No consumer may
  assume a positive numeric height means measured geometry.

Extend `DetectionFrame.msg` or an equivalent `luggage_msgs` diagnostic message
with support status sufficient to debug:

- `support_valid`, `support_reason`, `support_z`;
- support confidence, fit residual, side coverage, and inlier count;
- geometry level (`TOP_ONLY` or `FULL_3D`).

Required failure/status codes:

- `DETECT_TOP_UNOBSERVABLE`
- `DETECT_SUPPORT_UNOBSERVABLE`
- `DETECT_SUPPORT_UNSTABLE`
- `DETECT_SUPPORT_STAMP_MISMATCH`
- `DETECT_HEIGHT_PRIOR_ONLY`
- `DETECT_FULL_GEOMETRY_REQUIRED` for downstream operations that cannot accept
  top-only data.

## E1 - Pure Algorithm Modules

Refactor the mathematical work out of `luggage_detector_node.py`. Keep modules
ROS-free and deterministic.

Suggested plain-data structures:

- `TopSurfaceEstimate`: top center, top Z, yaw/quaternion, width, depth,
  confidence, stamp, frame.
- `SupportPlaneEstimate`: support Z, residual, normal alignment, side coverage,
  inlier count, confidence, stamp, frame.
- `BoxGeometryEstimate`: top estimate plus optional support-derived height and
  center.

Suggested APIs:

```python
estimate_top_surface(cargo_points_world, workspace, config)
estimate_local_support(raw_points_world, top_estimate, workspace, config)
compose_box_geometry(top_estimate, support_estimate=None, platform_z=None)
```

Top estimator:

1. Validate shape and finite points.
2. Crop by a deployment pickup workspace, not an exact spawned-box pose.
3. Voxel-downsample within the existing supported range.
4. Fit the highest valid horizontal cargo plane.
5. Fit the robust 2-D rectangle and preserve top Z explicitly.

Support estimator:

1. Rotate raw world points into the top rectangle axes.
2. Exclude the luggage footprint plus an inner guard margin.
3. Keep an outer annulus inside the deployment pickup workspace.
4. Restrict candidates below top Z using configurable physical luggage-height
   bounds, not a known platform height.
5. Fit horizontal plane candidates with deterministic RANSAC/robust statistics.
6. Require spatial adjacency and observations on at least the configured number
   of rectangle sides.
7. Score residual, inlier area/count, side coverage, normal alignment, and
   short-window Z stability.

For the horizontal v1 model:

```text
height   = top_z - support_z
center_z = (top_z + support_z) / 2
```

Preserve `estimate_box()` as a compatibility wrapper if needed by existing
tests, but move the online detector to the new split API. Do not add ROS imports
to the algorithm modules. Stateful temporal filtering must use the architecture
`update()` / `copy_output()` ownership pattern.

## E2 - Exact-Stamp Node Wiring

Update the ROS node layer to consume:

- semantic cargo cloud;
- preprocessed raw depth cloud;
- YOLO/mask-derived detection metadata;
- preprocessor geometry/motion status.

Requirements:

- Join by exact `(sec, nanosec)` acquisition stamp with bounded buffers.
- Transform cargo and raw points using TF at that stamp, never latest TF.
- Reject missing stamped TF; do not fall back to current/latest transform.
- Do not fuse fresh raw depth with `hold_track` cargo as a measured height.
- Require settled/`geometry_ok` input for support fitting.
- Publish `TOP_ONLY` when the top is valid but support is not.
- Keep support failure separate from top detection failure.

The detector currently exceeds the architecture's preferred node size. New
join/state/geometry logic belongs in importable modules; the script remains a
ROS adapter.

## E3 - Configuration and Launch

Add backend-neutral parameters to the perception config/profile:

- pickup workspace center and XY half extents;
- plausible min/max luggage height;
- support annulus inner/outer margins;
- horizontal normal tolerance;
- RANSAC distance threshold and minimum inliers;
- minimum observed support sides/coverage;
- support temporal window and maximum Z spread;
- support mode: `auto`, `configured`, `auto_then_configured`, `top_only`;
- optional configured platform Z.

Rules:

- `auto` is the normal simulation and hardware acceptance mode.
- `configured` is an explicit deployment mode, not a hidden fallback.
- `auto_then_configured` reports which source was selected on every result.
- Omitted `platform_z` is valid configuration.
- `scene_tf_config` may describe measured static workspace geometry, but
  `pickup_source.z` must not enter the auto estimator.
- Catalog configuration gets its own explicit parameter instead of requiring
  pickup scene geometry as an indirect dependency.

## E4 - Downstream Consumers

Audit all `DetectedLuggage` consumers with `rg` and update at least:

- waypoint generation: use `top_surface_pose` directly for pick Z;
- planning-scene insertion: require `height_valid`;
- packing/place computation: require valid full dimensions or invoke an
  explicit prior policy;
- vacuum/eval adapters: preserve source and validity fields;
- visualization: distinguish measured full boxes from top-only outlines;
- recorded diagnostics: include acquisition stamp, source, and validity.

No downstream operation may silently convert top-only data into a zero-height
or catalog-sized collision box.

## E5 - Eval Separation

- Keep Gazebo `pickup_source`, spawned pose/size, and `GetCurrentBox` reads in
  `luggage_gazebo` eval drivers only.
- Remove GT fallback/comparison from the online detector in the same migration
  or create a tracked follow-up before acceptance.
- Real rosbag reconstruction and annotations remain offline reference data;
  they are never replayed as inputs to online function nodes.

## Engineering Completion

Engineering is complete when:

- all E0-E5 code/config migrations are implemented;
- unit and integration tests in the companion test plan pass;
- simulation runs with omitted/wrong `platform_z` without changing measured
  top Z;
- top-only and full-geometry semantics are honored by every downstream
  consumer;
- no online node reads simulation truth for geometry;
- an engineering role note records changed files and verification commands.

