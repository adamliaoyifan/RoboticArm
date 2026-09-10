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

## First pass (superseded, kept for the tuning evidence)

Raw conf 0.005 + 8 sim prompts + every box painted: 100% of frames had
multiple cargo boxes (rank-2 up to 0.99 at the borders), masks unusable.
Raw confidences of that pass remain in the git history of this directory.

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
  supported by design this round.
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
| Full fidelity | all 4496 joined frames, per-frame depth/mask/ply + mp4 | `~/work/pendant_replay_out/` (outside repo) |
| Sampled evidence | 12/13/13 frames per bag (stride 35/212/127) | `docs/status/evidence/pendant_replay/` (this dir) |
