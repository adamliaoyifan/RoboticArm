# Sim/Real Parity Review and Pickup Support Plan

Date: 2026-09-04

This note records the current gaps where the online solution depends on
simulation-only or privileged information, then scopes the first optimization:
removing the hard dependency on `scene_tf.yaml` `pickup_source` for pickup ROI
and platform height.

Design rule:

- Simulation and hardware should run the same perception algorithm.
- Simulation may provide convenient adapters, but online nodes must not require
  data that hardware cannot observe or calibrate.
- If a required real-world prior is missing or stale, the real robot must fail
  closed instead of silently falling back to a simulation default.

## Review Findings

### P0: pickup detector uses scene YAML as live support truth

Current path:

- `src/luggage_perception/scripts/luggage_detector_node.py` loads
  `pickup_source_in_world(scene_config)` at startup.
- The detector stores `self._source_xyz` and `self._platform_z`.
- `_pca_from_cloud_msg()` calls `estimate_box()` with:
  - `roi_center_xy=(self._source_xyz[0], self._source_xyz[1])`
  - `platform_z=self._platform_z`
- `src/luggage_perception/luggage_perception/luggage_box_estimator.py`
  crops the cloud by that ROI and filters points below
  `platform_z + min_height_above_platform`, then computes
  `height = top_plane_z - platform_z`.

Why this is not hardware-safe:

- In simulation, `pickup_source` is also where the suitcase is spawned. Hardware
  does not know that pose from a spawner.
- The default scene platform top is `z=0.86m`. If the real table/platform is
  higher, lower, tilted, or shifted in XY, the detector can crop out the object,
  filter away valid box points, estimate the wrong height, or mistake top
  surfaces.
- A valid algorithm is being fed a privileged or stale support prior.

### P0: detector still contains GT fallback

`luggage_detector_node.py` imports `GetCurrentBox` and still has an optional
fallback that returns the spawner's current box when perception fails. The
current Gazebo launch disables it, but the code path should not live inside the
online detector. Ground truth comparison belongs in `luggage_gazebo` eval
drivers, not in `DetectLuggage`.

### P1: perception nodes consume `/luggage/current_box`

`semantic_segmenter_node.py`, `semantic_point_filter_node.py`, and
`luggage_detector_node.py` use `/luggage/current_box` for generation/epoch
state. Hardware can have a task epoch, but not a spawned suitcase id with
truth pose/size/mass. This should become a backend-neutral epoch event that
does not include geometry.

### P1: semantic point filter uses latest TF

`semantic_point_filter_node.py` uses latest TF for world/camera transforms.
That hides problems in static simulation but causes real-robot errors when the
arm or wrist camera moves. It should use the point cloud/image stamp and fail
the frame if stamped TF is unavailable.

### P1: preprocessor defaults are simulation-specific

`sensor_preprocessor.yaml` defaults `input_cloud_data_frame` to `camera_link`
because Gazebo labels the D435 point cloud misleadingly. That is a simulation
adapter setting, not a hardware default. Hardware configs should use the real
driver frame and enable/validate motion gating.

### P1: vacuum controller consumes simulation current-box truth

`vacuum_controller_node.py` defaults to `backend="sim"` and parses current-box
pose/size/mass from `/luggage/current_box`. Hardware attach/detach should use
pressure/contact/controller state, with perception as an advisory input.

### P1: cargo map commits planned geometry as occupancy truth

`cargo_volume_mapper_node.py` currently accepts geometry commits only. The
packing eval driver commits the planned slot after a place success. Hardware
must commit measured post-place occupancy when available, or mark planned-only
occupancy as low confidence.

### P1: hardware launch allows example scene config

`scene_hardware.launch.py` defaults to the example `scene_tf.yaml`. Hardware
launch should require an explicit measured config and reject the package
example unless a deliberate simulation mode is selected.

## Problem 1 Scope: Pickup Support Input

The algorithm we want to keep is:

1. Transform point cloud into a common frame.
2. Crop to pickup workspace.
3. Remove support/table points below the object band.
4. Fit the top horizontal plane.
5. Fit top-face rectangle and yaw.
6. Estimate height from top plane minus support plane.

The unsafe part is where steps 2 and 3 get their support input. The solution
should change the input contract, not fork the estimator into separate
simulation and hardware algorithms.

## Feasible Options

### Option A: measured scene config only

Make hardware provide a measured `scene_tf.yaml` and keep using
`pickup_source`.

Pros:

- Smallest code change.
- Keeps current tests and launch structure mostly intact.

Cons:

- Still treats a static YAML value as live support truth.
- Does not catch a platform moved after calibration.
- Does not address tilt or partial setup mistakes.

Verdict: acceptable as a temporary explicit calibration source, not sufficient
as the final online contract.

### Option B: live empty-platform support calibration

Add a pickup support calibration step before spawning/placing luggage. The node
observes the empty platform/table, fits a horizontal support plane from the
live depth cloud, records `platform_z`, `roi_center_xy`, extents, stamp, frame,
and confidence. Later `DetectLuggage` uses that measured support estimate.

Pros:

- Hardware-feasible and uses the same depth path as detection.
- Catches height mismatch immediately.
- Can still run in simulation with the same algorithm.

Cons:

- Requires an empty platform observation.
- Needs invalidation when the platform moves or camera calibration changes.
- Plane fitting must avoid robot/self points and background.

Verdict: best short-term hardware-safe route.

### Option C: per-detection support plane from mixed cloud

During each `DetectLuggage`, fit both the support plane and the object top from
the same cloud. Use low horizontal planes as support candidates and the highest
horizontal plane as the box top.

Pros:

- No separate calibration step.
- Adapts to small platform changes.

Cons:

- When the suitcase covers most of the support or semantic cargo points omit
  the platform, support plane may be unobservable.
- More sensitive to floor, robot, suction panel, and background planes.
- Harder to validate quickly.

Verdict: useful as a later robustness layer, but risky as the first migration.

### Option D: fiducial or measured fixture frame

Estimate pickup support from AprilTag/ArUco/fixture calibration, then feed the
same support estimate to the detector.

Pros:

- Good for repeatable demos and avoids needing an empty support surface in
  every cycle.
- Separates platform calibration from object detection.

Cons:

- Requires physical markers or a maintained calibration workflow.
- Still needs runtime validation against depth if the fixture is bumped.

Verdict: good companion to Option B; not required for the first software slice.

### Option E: no platform prior

Call `estimate_box(platform_z=None)` and infer height from the lowest observed
object point.

Pros:

- Removes support-prior dependency.

Cons:

- Top-down views often do not see the suitcase bottom.
- Raw depth includes table/support points, so the minimum Z can be the table,
  floor, or noise.
- Height becomes unreliable exactly in the hardware cases we care about.

Verdict: acceptable only for already-isolated semantic cargo clouds and only
as a degraded mode with lower confidence.

## Recommended Plan

Use Option B first, with a clean path for Option D later:

- Introduce a plain-data pickup support estimate in `luggage_perception`.
- Keep `estimate_box()` unchanged initially.
- Add a support-provider layer in `luggage_detector_node.py`:
  - `sim_scene`: reads `scene_tf.yaml` and marks source as simulation prior.
  - `measured_static`: uses a configured measured support estimate.
  - `live_calibrated`: uses a support estimate created from a live empty
    platform observation.
- Real launch must reject `sim_scene` unless explicitly allowed.
- Detection diagnostics must publish which support source was used, its
  `platform_z`, `roi_center_xy`, confidence, and age.

## Implementation Slices

### Slice 1: make the unsafe dependency explicit

Code:

- Add a small algorithm module, for example
  `luggage_perception/pickup_support.py`, with:
  - `PickupSupportEstimate`
  - `support_from_scene_config(scene_config)`
  - validation helpers for confidence, age, extents, and source
- Change `luggage_detector_node.py` to store a support estimate instead of raw
  `self._source_xyz/self._platform_z`.
- Add detector parameters:
  - `pickup_support_source`: `scene_config`, `measured_static`,
    `live_calibrated`
  - `allow_sim_pickup_support`: default `false`
  - `support_max_age_sec`
  - `measured_platform_z`
  - `measured_roi_center_xy`
  - `measured_roi_half_extent_xy`
- Preserve current Gazebo behavior by setting:
  - `pickup_support_source=scene_config`
  - `allow_sim_pickup_support=true`
  in `sim_world.launch.py`.
- Hardware launch should set measured/live source and should fail startup if it
  would use package example scene values.

Tests:

- Unit test `support_from_scene_config()` preserves current
  `pickup_source_in_world()` behavior for simulation.
- Detector support validation rejects `scene_config` when
  `allow_sim_pickup_support=false`.
- A platform z mismatch test should prove that a stale scene support estimate
  fails validation or reports a low confidence diagnostic before detection.

Exit:

- Existing simulation detection tests keep passing.
- Logs/diagnostics make the support source visible.
- Real config cannot accidentally use default `0.86m`.

### Slice 2: empty-platform live support calibration

Code:

- Implement a pure numpy support-plane estimator:
  - input: world-frame points, optional broad workspace bounds
  - filter finite points
  - reject robot/self/background by broad ROI and z range
  - fit dominant horizontal low plane
  - compute `platform_z` as robust median plane height
  - compute `roi_center_xy` from configured workspace center or plane inlier
    bounds
  - return confidence from inlier count, plane normal alignment, plane residual,
    and observed area
- Add a calibration trigger:
  - initial version can be a service or startup parameter-driven one-shot.
  - the service should not return suitcase geometry, only support estimate
    status.
- Store the calibrated estimate in the detector process and optionally publish
  JSON diagnostics.

Tests:

- Synthetic plane tests with noise, partial table, floor clutter, and a high
  box-like object.
- Regression test where platform true z is not `0.86m` and detection succeeds
  when calibrated support is used.
- Regression test where no support plane is visible and `DetectLuggage`
  returns `DETECT_PICKUP_SUPPORT_UNCALIBRATED` or equivalent.

Exit:

- `DetectLuggage` can run with `pickup_support_source=live_calibrated` without
  reading `pickup_source` as truth.
- Failure mode is explicit when calibration is missing or stale.

### Slice 3: use measured support in hardware bringup

Config/launch:

- Split pickup support parameters for sim and hardware.
- Hardware launch should require either:
  - a measured static support estimate, or
  - a completed live calibration.
- Add a startup guard for package example scene config in hardware mode.

Validation:

- Dry run with an intentionally wrong `scene_tf` platform height. Detection
  should still use live/measured support, not the wrong YAML value.
- Dry run with support disabled. Detection should fail closed with a clear
  support error.
- Record a real or replayed depth sample where the platform is not at `0.86m`.

### Slice 4: optional per-detection support refresh

After the first three slices are stable, add Option C as an update mechanism:

- If raw depth contains visible support inliers, refresh `platform_z` slowly
  with bounded drift.
- Never let a single detection frame move support by more than a configured
  threshold.
- Do not refresh from semantic cargo-only clouds that exclude the platform.

## Open Decisions

- Whether the first live calibration service should be added to `luggage_msgs`
  or kept as a detector-private service until the contract stabilizes.
- Whether `roi_center_xy` should be measured from the support plane inliers or
  kept as a calibrated workspace center while only `platform_z` is live-fitted.
  The safer first cut is calibrated XY plus live Z.
- Whether tilted support surfaces matter for the current demo. The current box
  estimator assumes a horizontal support plane; supporting tilt means either
  leveling points into a support frame before `estimate_box()` or extending the
  estimator beyond `platform_z`.
- How to persist a live calibrated support estimate across node restarts. First
  cut can keep it in memory; later hardware bringup can write a measured YAML.

## Acceptance Criteria

- The detector no longer has an implicit dependency on `scene_tf.yaml`
  `pickup_source` for hardware mode.
- Simulation and hardware both call the same `estimate_box()` path.
- Support source is observable in diagnostics.
- `scene_config` support is allowed only in simulation or explicit test mode.
- A wrong simulation default such as `platform_z=0.86m` cannot silently drive a
  hardware detection.
- Unit tests cover wrong platform height, missing support calibration, and
  successful detection with measured support.
