# Pack eval carryon n=2 + place smoke N=3

日期：2026-09-02。`ROS_DOMAIN_ID=7`。

## B5 place N=3

`docs/status/evidence/place_smoke_n3_regress/`

- 3/3 `place_ok`，descend fraction 全 1.0，载荷丢失 0
- `err_xy` 均值 7.7 mm，`err_z` −2.3 mm，3/3 `inside_inner_box`

## B4 n=2 冒烟（含停机可视化）

`docs/status/evidence/packing_eval_carryon_n2/`

| 字段 | 值 |
|---|---|
| `termination_reason` | `MAX_BOXES` |
| `capacity_claim_valid` | false |
| `boxes_packed` | 2 |
| `floor_coverage` | 0.15 |
| `cycle_sec.mean` | 33.9 s |

停机目录 `final_layout/`：`boxes.json` 两只 carryon，`container_and_boxes.ply`（内壁+箱）、`interior_free.ply`（剩余空腔）、`layout.html`（浏览器打开）。

均质 `BIN_FULL`：`packing_eval_carryon_n50` 在重启 sim 后 RTF≈0，连续 3 次 pick 失败（`PLAN_approach` / `YOLO_NOT_READY`）`ABORT`，未形成容量结论。可视化仍写出了空箱 `final_layout/`。需要在 RTF≈1 的干净 `sim_world` 上重跑 `--max-boxes 50`。
