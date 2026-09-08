# Pack eval carryon n=2 + place smoke N=3

日期：2026-09-02。`ROS_DOMAIN_ID=7`。

## B5 place N=3

`docs/status/evidence/place_smoke_n3_regress/`

- 3/3 `place_ok`，descend fraction 全 1.0，载荷丢失 0
- `err_xy` 均值 7.7 mm，`err_z` −2.3 mm，3/3 `inside_inner_box`

## B4 n=2 冒烟（含停机可视化）

重跑：2026-09-02 16:18，干净单套 `sim_world`，`ROS_DOMAIN_ID=7`，`RTF=1.0`。
`docs/status/evidence/packing_eval_carryon_n2/`

| 字段 | 值 |
|---|---|
| `termination_reason` | `MAX_BOXES` |
| `capacity_claim_valid` | false |
| `boxes_packed` | 2 |
| `floor_coverage` | 0.15 |
| `cycle_sec.mean` | 38.7 s |
| `rtf_last` | 1.0 |

两箱都 `committed=true`，地板槽 world z=0.655。`reject_histogram` 含 `outside_hull=33` / `outside_aperture`（645 / 597）。

`final_layout/layout.html` 已是七面体内腔：hull **15** 边（地板 +Y 只到 `y=0.55`），黄线 aperture 4 边 `y∈[-0.928, 0.398]`、`z∈[0.597, 2.009]`。浏览器打开该 HTML。

均质 `BIN_FULL`：上一轮 `packing_eval_carryon_n50` 因双 Gazebo / RTF≈0 在 pick 阶段 `ABORT`，不是容量结论。当前这套仿真仍在、RTF=1，可直接 `--max-boxes 50`。
