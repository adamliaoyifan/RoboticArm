# Pendant bag replay YOLO evaluation — 2026-09-10 (retuned)

Open-loop offline replay of the teach-pendant mcap bags through the
YOLO-World (`bbox_fill`) segmenter, `device=cuda`. This revision applies
the real-site tuning from the first-pass review (one-luggage-per-frame
ground truth; the first pass painted every ≤0.1-confidence box into the
mask, which made most masks wrong).

## Tuning (data-driven, from the first pass `detections.jsonl`)

- **Threshold**: `confidence >= 0.3`. First-pass rank-1 det conf was
  p10 0.56–0.92 while the false positives clustered below 0.3.
- **Prompts**: `["luggage on a platform viewed from directly above",
  "suitcase", "luggage"]`, all cargo. Ablation winner over the 8-prompt
  sim set and bare `suitcase/luggage` (which collapses to conf ~0.1);
  `box/floor/container/robot arm` dropped — they produced the 4.7k
  low-conf false boxes. Sim config stays untouched (pass `--config` to
  restore).
- **One-box selection** (`--cargo-select center_conf`): among cargo dets
  ≥0.3 keep the max-confidence one whose box centre lies within
  `0.35 × width` px of the image centre (measured: the true box sits
  there in 97–100% of frames); without a central candidate keep the
  global max-conf and flag `off_center`; below the floor keep nothing
  (honest miss). The mask/overlay/points are rebuilt from the kept box
  alone (`repaint_label_map`).

## Full-run results (tuned; all 4496 joined frames)

| Bag | Pairs | Frames w/ cargo | central / off_center / none | Kept conf p50/p90 |
|---|---|---|---|---|
| record_site_pendant_20260909_164710 | 419 | 419/419 | 419 / 0 / 0 | 0.969 / 0.978 |
| pendant_jog_vaccum3 | 2546 | 2540/2546 | 2539 / 1 / 6 | 0.909 / 0.983 |
| pendant_jog_vaccum4 | 1531 | 1529/1531 | 1505 / 24 / 2 | 0.947 / 0.990 |

Exactly one cargo box per processed frame; inference p95 5.3–6.2 ms/frame.
Verification of the artifact chain on frame `1788943637_390859231`:
`detections.json` = 1 det (0.974), mask = single lower-centre region, and
the on-disk `overlay.png` is byte-identical to
`draw_detections_overlay(rgb, [kept])` regenerated from scratch.

## Full `/livox/lidar` (Mid-360) archive

Every scan is archived per bag — independent of camera-frame coupling
(`--no-lidar-archive` opts out; this sampled evidence copy opts out, the
full tree does not):

```
<bag>/lidar/<sec>_<nsec>/points.npy   # structured (N,): x,y,z,intensity (f4), tag,line (u1), timestamp (f8)
<bag>/lidar/<sec>_<nsec>/lidar.ply    # XYZ PLY for CloudCompare
<bag>/lidar_index.jsonl               # stamp/log_time/n_points/ts_min/ts_max/scan_span_ms/dir per scan
```

All seven Mid360 PointXYZRTLT fields are archived. **`timestamp` is the
per-point absolute unix-epoch nanosecond count carried as FLOAT64
(~0.25 ns resolution at 1.8e18)** — the deskew input. Measured: point
timestamps start at the scan header stamp +0.000 ms and span p5/p50/p95
= 97/100/103 ms (one 10 Hz frame); `ts_min_ns`/`ts_max_ns`/`scan_span_ms`
per scan are precomputed in `lidar_index.jsonl`. `tag`/`line` are
archived as recorded (tag non-zero on a minority of points — echo/noise
flags; line populated per scan line).

| Bag | Scans archived (= msgs) | Total points | Decode failures |
|---|---|---|---|
| record_site_pendant_20260909_164710 | 202 | 3,680,832 | 0 |
| pendant_jog_vaccum3 | 1278 | 22,357,056 | 0 |
| pendant_jog_vaccum4 | 1094 | 21,424,032 | 0 |

2574/2574 scans (counts match the bag metadata), 47.46 M points. Sample
`1788943637_784117573`: (20064, 4) float32, z ∈ [-0.62, 5.41] m. Each
camera frame's `meta.json` carries `lidar.nearest_stamp_ns` /
`nearest_dir` / `dt_sec` (pointer into the archive, no duplication);
`--with-lidar` additionally copies the nearest scan's payload into the
frame dir. Scans are raw (no deskew, `livox_frame`; hdr stamp is scan
start ~66 ms before log time).

## Join findings (measured)

- Colour and aligned depth share the exact header stamp on most frames;
  drops are staggered (orphan gaps to the nearest *unmatched* depth exceed
  one 33 ms camera period), so the 30 ms rescue tolerance recovers only
  22 pairs; the rest are honest orphans, reported in `join_report.json`.
- `/livox/imu` (200 Hz) and elfin telemetry are skipped by the reader;
  `join_report.json` lists every skipped topic with counts.

## Known limitations

- `d555_color_optical_frame` is absent from the recorded TF tree (driver
  TFs were not recorded), so `cargo_points.ply` stays in the optical
  frame; world-frame 3D needs a URDF-mirror bridge or hand-eye extrinsics
  (follow-up task).
- pendant_jog_compressed (CompressedImage + livox CustomMsg) is not
  supported by design this round (its lidar is CustomMsg, not PointCloud2).
- aux (joint_states/tcp_pose) joins are nearest-stamp within 50 ms;
  misses are recorded as absent files with `joint_dt_ms: null` in
  `meta.json`, never silently widened.

## Reproduce

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
pendant_bag_replay_eval.py \
  --bag ~/work/robotarm_bags/record_site_pendant_20260909_164710 \
  --bag ~/work/robotarm_bags/2026-09-09/pendant_jog_vaccum3 \
  --bag ~/work/robotarm_bags/2026-09-09/pendant_jog_vaccum4 \
  --out <out_root> --backend yolo_world --device cuda --make-video
```

Defaults are the tuned real-site set above; `--cargo-select none` and/or
`--config <sim yaml>` restore raw behaviour. Requires
`pip install mcap mcap-ros2-support` (Humble's rosbag2 cannot parse these
bags' embedded metadata; see `luggage_perception/eval/bag_mcap_source.py`).

| Run | Scope | Location |
|---|---|---|
| Full fidelity | all 4496 joined frames + full lidar archive (14 GB) | `~/work/pendant_replay_out/` (outside repo) |
| Sampled evidence | 12/13/13 frames per bag (stride 35/212/127) | `docs/status/evidence/pendant_replay/` (this dir) |
