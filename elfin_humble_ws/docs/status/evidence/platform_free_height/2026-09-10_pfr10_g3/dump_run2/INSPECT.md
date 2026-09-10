# How to inspect this dump

Use **dump_run2**, not dump_run1. Every trial has RGB + depth + mask + cargo
PLY. Failed trials also have `early/` and `mid/`.

This run did **not** reproduce the earlier whole-window
`DETECT_TOP_UNOBSERVABLE` miss. C1 failed because two trials took longer
than 1.4 s to reach FULL_3D (`slow_full3d`). Settled `failed` count is 0
and `top_surface_rate` is 1.0. Gazebo `tilt_rad` on the failed large box
is 0 — the suitcase is standing.

## Start here

1. `dumps/INDEX.md` — pass/fail table
2. Failed trials:
   - `dumps/trial_00_fail_slow_full3d_pickup_box_0001_carryon/`
   - `dumps/trial_05_fail_slow_full3d_pickup_box_0006_large/`
3. Passing comparison: `dumps/trial_02_ok_ok_pickup_box_0003_large/late/`

## Files in each snapshot

| file | what |
|---|---|
| `color.png` | preprocessed RGB |
| `depth.png` / `depth.npy` | grey preview / metres |
| `mask.png` / `mask_labels.npy` | cargo=red, other labels colorized / raw ids |
| `cargo_world.ply` / `cargo_world.xyz` | cargo cloud in `world` (open this in CloudCompare) |
| `cargo_workspace.ply` / `.xyz` | after XY workspace crop `(-1,0) ± 0.5 m` |
| `cargo_camera.ply` / `.xyz` | same cloud in `camera_depth_optical_frame` |
| `meta.json` | GT, detector `stream_stats`, crop/inlier counts, TF |
| `../gz_pose.json` | live Gazebo pose (tilt/yaw/xy) |
| `../scores.jsonl` | every scored DetectionFrame for the trial |

Do **not** open `early/*.ply` on the failed trials: those have 0 points
(spawn lag) and CloudCompare will say Nothing to load.

Open `late/cargo_world.xyz` or `late/cargo_world.ply` (binary, rewritten
for CloudCompare). If the PLY filter still treats it as a mesh, use the
`.xyz` file: CloudCompare File > Open, then accept X Y Z columns.

RANSAC top-surface (offline replay of the detector estimator, same
workspace crop / voxel 0.01 m / 8 mm horizontal RANSAC):

| file | what |
|---|---|
| `top_inliers.xyz` / `.ply` | points on the fitted horizontal top plane |
| `top_outliers.xyz` / `.ply` | workspace cargo that is not on that plane |
| `top_voxel.xyz` / `.ply` | voxelized input RANSAC saw |
| `top_ransac.json` | plane Z, inlier count, fitted width/depth |

Semantic overlay PNG was not written (overlay decode did not join); use
`color.png` + `mask.png` instead.

Gate-4 summary: `gate4/summary.json`. Sim residual after stop: `residual_count`.
