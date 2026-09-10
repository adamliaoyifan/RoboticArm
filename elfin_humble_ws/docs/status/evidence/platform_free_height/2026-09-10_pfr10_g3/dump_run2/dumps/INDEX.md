# Gate-4 trial dumps

Prefer this folder (`dump_run2`) over `../dump_run1`. Read `../INSPECT.md`.

Open `late/color.png`, `late/mask.png`, and `late/cargo_world.ply`.
Failed trials also keep `early/` and `mid/`. `meta.json` has detector
`stream_stats` (including `timing_ms.pipeline` crop/inlier counts),
segmenter/filter stats, and GT. `gz_pose.json` is the live Gazebo pose.

| trial | folder | result | box | pca_reason | n_cargo | t_valid_s | t_full3d_s |
|---|---|---|---|---|---|---|---|
| 0 | `trial_00_fail_slow_full3d_pickup_box_0001_carryon` | FAIL | pickup_box_0001_carryon | ok | 31834 | 0.5074126980034634 | 2.393332912004553 |
| 1 | `trial_01_ok_ok_pickup_box_0002_standard` | ok | pickup_box_0002_standard | ok | 42907 | 0.3525184349855408 | 1.124394405982457 |
| 2 | `trial_02_ok_ok_pickup_box_0003_large` | ok | pickup_box_0003_large | ok | 41005 | 0.2958138600224629 | 0.6491053570061922 |
| 3 | `trial_03_ok_ok_pickup_box_0004_carryon` | ok | pickup_box_0004_carryon | ok | 33685 | 0.39178561395965517 | 0.5776656050002202 |
| 4 | `trial_04_ok_ok_pickup_box_0005_standard` | ok | pickup_box_0005_standard | ok | 35659 | 0.3042159699834883 | 0.3042159699834883 |
| 5 | `trial_05_fail_slow_full3d_pickup_box_0006_large` | FAIL | pickup_box_0006_large | ok | 49644 | 0.4802751881070435 | 1.6552630320657045 |

