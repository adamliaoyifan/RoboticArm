# D555-BRINGUP-20260908 — archived results

Live Jazzy cell, 2026-09-08. Plan:
`docs/plans/d555_hardware_bringup_verification.md`.
No URDF, xacro, or yaml edits. No corrected mount transform.

Host: native ROS 2 Jazzy, `ROS_DOMAIN_ID=7`. D555 PoE `192.168.11.55`
SN `419222302385`, FW `7.56.37776.6014`, stable `640x360@15`,
`align_depth.enable=true`. Mid-360S `192.168.1.120`. Arm CPS
`192.168.0.10:10003`. Tested workspace snapshot `40ab61c`.

`/joint_states` from the Sep-2 executor was all zeros. HB-3 FK used CPS
`HRIF_ReadActACS`, not that topic.

## Lifecycle

| ID | Outcome | Closed | Thread | Evidence |
|---|---|---|---|---|
| HB-1 | pass | 2026-09-08 19:57 +08 | `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-1.md` | `20260908_1951_hb1/` |
| HB-2 | pass | 2026-09-08 20:27 +08 | `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-2.md` | `20260908_2004_hb2/` |
| HB-3 | pass | 2026-09-08 20:55 +08 | `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-3.md` | `20260908_2036_hb3/` |

Mailbox rows `Q-20260908-4/5/6` are closed.

## HB-1 — device facts

Colour K at 640×360: fx=323.18, fy=322.90, cx=317.75, cy=178.03 (not sim
337.222). Depth K: fx=fy=321.51. `depth_to_color` translation ≈
`[-0.05877, 0, 0.0008]` m (~59 mm colour lever).

| Product | frame_id | Verdict |
|---|---|---|
| colour image | `d555_color_optical_frame` | consistent |
| aligned depth | `d555_color_optical_frame` + colour K | colour-aligned, consistent |
| native depth and `/depth/color/points` | `d555_depth_optical_frame` | depth-native, consistent |

`camera_link` → `d555_link` is identity. DDS on native Jazzy. Rates ~14–15 Hz.

## HB-2 — RGB-to-depth alignment

Green stool on tiled floor. Sign: +du = aligned-depth contour right of colour.

| Operator | Floor Z | signed du, dv (px) |
|---|---|---|
| ~0.6 m | 0.614 m | +5, +1 |
| ~1.0 m | 1.027 m | +1, +2 |
| ~2.0 m | 1.955 m | −3, +7 |

Residuals are a few pixels, not the 12–54 px shift of an unaligned 59 mm
lever. Aligned depth is colour-aligned. Native depth and coloured points
stay depth-native. Do not unproject `/depth/color/points` with colour K.

## HB-3 — mount residual vs Mid-360

Joints from CPS. Planes in `elfin_base`. GUI mount still in the chain:
`eef_mount_adapter` → `camera_link` xyz `0.013 0.097 -0.021`,
rpy `0.03770 1.36345 1.57080`.

| pose | offset mm | angle deg |
|---|---|---|
| pose1 | 10.10 | 1.91 |
| pose2 | 2.42 | 2.33 |
| pose3 | 1.03 | 2.19 |

Offset varies with pose; angle stays near 2°. **Mount rotation error.**
These three floor poses do not constrain yaw and are not a full hand-eye
run. Mid-360 edge doubling is RViz Decay Time over a non-repetitive scan.

## Reported, not edited

`realsense_d435.yaml` `tune_joints` rpy (`rx=-1.363, ry=0.038, rz=1.571`)
disagrees with `fixed.rpy` / `cam_mount_rpy` (`0.038, 1.363, 1.571`).
Live URDF uses the latter.

## Next (not in this archive)

CAD-seed `eef_mount_adapter` → D555 body, then ChArUco eye-in-hand
(10–20 static poses). Do not write a 2° GUI tweak from HB-3 alone.

## Pointers

- Plan: `docs/plans/d555_hardware_bringup_verification.md`
- Eng notes: `docs/agents/eng/2026-09-08_1955_d555-hb1-device-facts.md`,
  `docs/agents/eng/2026-09-08_2026_d555-hb2-alignment.md`,
  `docs/agents/eng/2026-09-08_2054_d555-hb3-mount-residual.md`
