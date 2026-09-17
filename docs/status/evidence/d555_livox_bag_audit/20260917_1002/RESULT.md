# D555/Livox real-bag stream audit

- date: 2026-09-17
- role: reviews
- agent: codex
- model: gpt-5
- source_bags:
  - `/home/adamliao/work/robotarm_bags/record_site_20260908_210234`
  - `/home/adamliao/work/robotarm_bags/2026-09-09/pendant_jog_compressed`
  - `/home/adamliao/work/robotarm_bags/0915/pendant_20260915_200806`
  - `/home/adamliao/work/robotarm_bags/record_site_pendant_20260911_220406`
- command: `source /opt/ros/humble/setup.bash; source install/setup.bash; PYTHONPATH=src/luggage_perception:src/luggage_description:$PYTHONPATH python3 - <<'PY' ...`
- note: direct MCAP reader used because Humble rosbag2 cannot parse these site bags.

## Result

The D555 RGB and aligned-depth streams are aligned by header acquisition stamp.
Recorder log times are not identical and should not be used as the pairing
clock.

## First full site bag

Bag: `/home/adamliao/work/robotarm_bags/record_site_20260908_210234`.

| Topic | Count | Header rate | p50 period | p95 period | Frame |
|---|---:|---:|---:|---:|---|
| `/camera/d555/color/image_raw` | 1535 | 14.27 Hz | 66.667 ms | 66.667 ms | `d555_color_optical_frame` |
| `/camera/d555/aligned_depth_to_color/image_raw` | 1532 | 14.24 Hz | 66.667 ms | 66.667 ms | `d555_color_optical_frame` |
| `/camera/d555/color/camera_info` | 1535 | 14.27 Hz | 66.667 ms | 66.667 ms | n/a |
| `/camera/d555/aligned_depth_to_color/camera_info` | 1532 | 14.24 Hz | 66.667 ms | 66.667 ms | n/a |
| `/camera/d555/depth/color/points` | 1536 | 14.27 Hz | 66.667 ms | 66.667 ms | `d555_depth_optical_frame` |
| `/livox/lidar` | 1074 | 9.99 Hz | 99.998 ms | 100.934 ms | `livox_frame` |
| `/livox/imu` | 20823 | 193.53 Hz | 5.024 ms | 5.904 ms | `livox_frame` |
| `/joint_states` | 10757 | 100.00 Hz | 10.002 ms | 10.310 ms | empty frame |

RGB/depth header alignment:

- exact matches: 1532 / 1535 colour stamps and 1532 / 1532 depth stamps.
- depth stamps without exact colour: 0.
- colour-only stamps: 3.
- nearest header delta p50/p95: 0 ms / 0 ms.
- recorder log-time nearest delta p50/p95: 9.86 ms / 12.77 ms, confirming log time is transport jitter.

D555 aligned-depth quality, sampled 40 frames:

- shape/encoding/frame: 640x360, `16UC1`, `d555_color_optical_frame`.
- median valid pixels: 210,553 / 230,400.
- median valid ratio: 0.914; p05 valid ratio: 0.910.
- median depth distribution per sampled frame: min 452 mm, p50 859 mm, p95 1201 mm, max 1574 mm.

D555 driver point cloud quality, sampled 50 frames:

- frame: `d555_depth_optical_frame`.
- median points: 211,045.
- finite ratio: 1.0.
- median range p50/p95 per frame: 1.03 m / 1.52 m.
- This cloud is depth-native; it is useful as hardware evidence but is not the canonical RGB-D pipeline product.

Mid-360 quality, sampled 80 scans:

- frame: `livox_frame`.
- median points: 19,968; p05 15,062; p95 20,160.
- finite ratio: 1.0.
- median per-scan time span: 100.22 ms.
- median range p50/p95/max per scan: 2.11 m / 6.24 m / 14.42 m.
- median intensity: 7.
- decoded `line` has 4 values but is not a usable ring model for deskew.

## Later compressed D555 bags

`/home/adamliao/work/robotarm_bags/2026-09-09/pendant_jog_compressed`:

- 217 colour and 217 aligned-depth compressed frames.
- RGB/depth exact header matches: 217 / 217.
- rate: 14.33 Hz, p50 period 66.679 ms.
- depth valid ratio median: 0.933.

`/home/adamliao/work/robotarm_bags/0915/pendant_20260915_200806`:

- 1060 colour and 1060 aligned-depth compressed frames.
- RGB/depth exact header matches: 1060 / 1060.
- rate: 15.00 Hz, p50 period 66.658 ms.
- depth valid ratio median: 0.837.
- Livox: 706 scans, 10.00 Hz, median 19,968 points/scan.

`/home/adamliao/work/robotarm_bags/record_site_pendant_20260911_220406`:

- 43 colour and 43 aligned-depth compressed frames.
- RGB/depth exact header matches: 43 / 43.
- rate: 15.04 Hz, p50 period 66.679 ms.
- Livox: 26 scans, 10.00 Hz.

## Simulation recommendation

For simulation, add or keep an independent Mid-360-like `/livox/lidar`
`PointCloud2` in `livox_frame`. Do not synthesize it from D555 depth and do
not make it part of the exact RGB-D pair.

Minimum contract:

- topic: `/livox/lidar`;
- frame: `livox_frame`;
- rate: 10 Hz;
- data: XYZ points in metres, finite returns only;
- FOV/range: 360 degrees horizontal, -7 to +52 degrees vertical, 0.1-40 m;
- density target: about 20k points per scan if matching real bag density;
- timestamp: lidar scan stamp, separate from camera stamp;
- deskew state: `deskewed=false` unless per-point times are simulated;
- no `/livox/imu` claim unless an IMU model with correct units exists.

The current sim xacro has 360 x 32 samples = 11,520 rays/scan at 10 Hz, which
is lower than the measured real median of about 19,968 points/scan. To match
real density while staying raster-based, use either about 360 x 56 or 625 x 32
samples. The former preserves 1-degree horizontal spacing and increases
vertical density; the latter preserves the current vertical count and increases
horizontal density. Both remain a repeating raster, not a real Mid-360
non-repetitive scan.

## Point spacing and dominant-surface residuals

Additional sample pass on 2026-09-17. For Mid-360, sampled every 20th scan and
computed spatial nearest-neighbor distance after filtering finite returns with
range 0.15-12 m. A dominant plane was fit by RANSAC and refined by SVD; plane
residuals are point-to-plane absolute distances. These numbers are diagnostics,
not pure sensor-noise specs: they include surface roughness, incidence angle,
motion during the scan, calibration error, and the chosen plane threshold.

Mid-360, first site bag, 40 sampled scans:

- full-cloud nearest-neighbor mean: 54.2 mm; p50: 39.2 mm; p95: 151.3 mm.
- dominant-plane inliers: median 5,388 points, about 31.0 percent of valid returns.
- dominant-plane nearest-neighbor mean: 25.0 mm; p50: 17.3 mm; p95: 67.4 mm.
- dominant-plane point-to-plane residual mean: 5.75 mm.
- dominant-plane residual std: 5.13 mm; variance: 2.63e-5 m^2.
- dominant-plane range mean: 1.80 m.
- dominant-plane intensity mean: 12.24; intensity variance: 53.83.

Mid-360, `0915/pendant_20260915_200806`, 36 sampled scans:

- full-cloud nearest-neighbor mean: 54.2 mm; p50: 35.8 mm; p95: 157.4 mm.
- dominant-plane inliers: median 5,998 points, about 36.0 percent of valid returns.
- dominant-plane nearest-neighbor mean: 19.6 mm; p50: 13.4 mm; p95: 48.7 mm.
- dominant-plane point-to-plane residual mean: 5.90 mm.
- dominant-plane residual std: 4.64 mm; variance: 2.15e-5 m^2.
- dominant-plane range mean: 1.26 m.
- dominant-plane intensity mean: 9.44; intensity variance: 37.12.

D555 aligned depth, first site bag, 30 sampled frames:

- valid aligned-depth pixels: median 210,527 per frame.
- full valid-depth nearest-neighbor mean: 8.60 mm; p50: 7.16 mm; p95: 19.63 mm.
- dominant-plane inliers: median 6,406 sampled points, about 53.4 percent of the sampled valid set.
- dominant-plane nearest-neighbor mean: 7.40 mm; p50: 6.71 mm; p95: 14.88 mm.
- dominant-plane point-to-plane residual mean: 2.31 mm.
- dominant-plane residual std: 2.01 mm; variance: 4.03e-6 m^2.
- sampled valid-depth range mean: 0.831 m.

Interpretation: D555 depth is much denser in its camera frustum and has lower
single-plane residual in these close-range samples. Mid-360 covers a much
wider volume with lower local surface density and a 100 ms scan duration, so
its quality must be judged with deskew/readiness and per-surface metrics, not
only raw point count.

## Pointers

- `src/luggage_description/config/mid360_origin.xacro`
- `src/luggage_description/urdf/eef_sensor_mount.urdf.xacro`
- `src/luggage_gazebo/launch/sim_world.launch.py`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/motion_compensation.md`
