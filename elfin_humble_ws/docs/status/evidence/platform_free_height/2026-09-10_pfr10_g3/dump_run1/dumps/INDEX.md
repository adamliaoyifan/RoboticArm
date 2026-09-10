# Gate-4 trial dumps

Superseded for inspection by `../dump_run2` (timed snapshots + cargo PLY).
This run still has RGB/depth/mask for every trial.

Open `late/color.png`, `late/overlay.png`, and `late/cargo_camera.ply`.
Failed trials also keep `early/` and `mid/`. `meta.json` has detector
`stream_stats` (including `timing_ms.pipeline` crop/inlier counts),
segmenter/filter stats, and GT. `gz_pose.json` is the live Gazebo pose.

| trial | folder | result | box | pca_reason | n_cargo | t_valid_s | t_full3d_s |
|---|---|---|---|---|---|---|---|
| 0 | `trial_00_ok_ok_pickup_box_0001_carryon` | ok | pickup_box_0001_carryon | ok | 27646 | 0.300707204034552 | 0.9699872230412439 |
| 1 | `trial_01_ok_ok_pickup_box_0002_standard` | ok | pickup_box_0002_standard | ok | 49877 | 0.2399446739582345 | 0.2399446739582345 |
| 2 | `trial_02_ok_ok_pickup_box_0003_large` | ok | pickup_box_0003_large | ok | 86813 | 0.3162624299293384 | 1.3757237399695441 |
| 3 | `trial_03_fail_fail_pickup_box_0004_carryon` | FAIL | pickup_box_0004_carryon | ok | 32884 | 0.34942518500611186 | 2.3769137100316584 |
| 4 | `trial_04_fail_fail_pickup_box_0005_standard` | FAIL | pickup_box_0005_standard | ok | 51675 | 0.4230665300274268 | 2.009159623994492 |
| 5 | `trial_05_ok_ok_pickup_box_0006_large` | ok | pickup_box_0006_large | ok | 92530 | 0.3903804860310629 | 0.8271304630907252 |

