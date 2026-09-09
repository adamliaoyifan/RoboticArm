# HB-2 RGB-to-depth alignment — three static ranges

Parent: `D555-BRINGUP-20260908`. Arm moved by operator between stations.
No URDF/yaml edits. Raw frames under `z0p6_stool/`, `z1p0/`, `z2p0/`.

Sign: `+du` = aligned-depth contour is to the **right** of the colour edge.

## Stations

| Operator | Floor Z (m) | Edge Z (m) | signed du, dv (px) | metres (u, v) | 59 mm would be (px u) | 95 mm would be (px u) |
|---|---|---|---|---|---|---|
| ~0.6 m | 0.614 | 0.35 (stool) | +5, +1 | 5.4 mm, 1.1 mm | ~54 | ~88 |
| ~1.0 m | 1.027 | 0.715 (stool) | +1, +2 | 2.2 mm, 4.4 mm | ~27 | ~43 |
| ~2.0 m | 1.955 | 1.616 (stool band) | −3, +7 | −15 mm, 35 mm | ~12 | ~19 |

Contour method: stool/floor depth jump vs nearest colour-gradient peak
(±12 px). Whole-frame phase correlation is not used for the verdict
(grout has no depth jump; 2 m scene is cluttered).

## Comparison

At every station the aligned-depth / colour residual is **a few pixels**,
not the 12–54 px u-shift implied by treating the product as depth-native
with the 59 mm colour lever, nor the larger 95 mm baseline shift.

The 2 m `dv=+7` is the noisiest (workbench + arm on the floor in FOV);
even there `|du|` is below the 59 mm prediction.

Identity warp of **native** depth at 0.6 m and 1.0 m moved the valid-depth
mask by about `du=−32` and `−23` px, matching the 59 mm lever projected at
floor Z. That is the depth-native product. `/camera/d555/depth/color/points`
stays in `d555_depth_optical_frame` with a populated `rgb` field (not
constant) at all three stations.

## Verdict

- `/camera/d555/aligned_depth_to_color/image_raw`: **colour-aligned**
- `/camera/d555/depth/image_rect_raw` and `/camera/d555/depth/color/points`: **depth-native** geometry (points carry real RGB texture)

Canonical depth image for a colour-space contract is the aligned topic.
Do not unproject `/depth/color/points` with colour `K`.
