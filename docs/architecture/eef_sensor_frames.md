# Canonical EEF sensor frames (D555 + Mid-360S)

**This is the correct TF tree and the correct camera housing.** Do not treat
GUI-tuned mounts, D435 90 mm visuals, or an earlier table-ICP revision as
current.

Status: **canonical** as of 2026-09-14. Revision
`2026-09-14_xy_balanced_corners_edges`. `locked: true` and
`applied_to_urdf: true` in
[livox_d555_table_icp_frozen.yaml](../../src/luggage_perception/config/livox_d555_table_icp_frozen.yaml).

Hardware: wrist **D555 PoE** (not D435, not D455) and **Livox Mid-360S** on
the printed `arm_realsense_v1.3` adapter. The URDF macro is still named
`realsense_d435` for history; the body is D555.

## Chain

```text
elfin_base_link
  -- FK(q) -->
elfin_end_link                          (EOF)
  -- flange CAD, do not move -->
suction_panel
  -- solved adapter (realizes frozen Livox pose) -->
eef_mount_adapter                       (mounter)
  -- CAD square pocket -->
mid360_mount_frame
  -- handbook optical +47 mm Z -->
livox_frame

eef_mount_adapter
  -- ChArUco Layer 3, re-expressed (T_end_optical unchanged) -->
camera_link  ==identity==  d555_link
  --> d555_color_frame --> d555_color_optical_frame
```

Projection of sensor points into `elfin_base_link` is always

```text
T_base ← sensor = T_base ← EOF  ·  (EOF → suction_panel → eef_mount_adapter → sensor)
```

Do not dump cloud ICP into `mid360_mount_frame`. Do not retune
`cam_mount_*` to “seat” a wrong housing; Layer 3 is optical, not the grey box.

## Locked joints (metres, radians)

| Joint | Source | xyz | rpy |
|---|---|---|---|
| `elfin_end_link` → `suction_panel` | flange CAD | `0.000600 0 0` | `-1.56451314 0 π` |
| `suction_panel` → `eef_mount_adapter` | solved so the chain realizes frozen Livox | `0.021734 -0.033926 0.082591` | `1.51187936 -0.01424906 3.13751341` |
| `eef_mount_adapter` → `mid360_mount_frame` | CAD pocket on the adapter +Y face | `0.022 0.103 0.038` | `0 π/2 π/2` |
| `mid360_mount_frame` → `livox_frame` | Mid-360S handbook optical | `0 0 0.047` | `0 0 0` |
| `eef_mount_adapter` → `camera_link` | CC600 ChArUco Layer 3, re-expressed | `-0.023249 0.099580 -0.052059` | `0.02901151 1.32524323 1.59953586` |
| `camera_link` → `d555_link` | identity | `0 0 0` | `0 0 0` |

Frozen `elfin_base_link ← livox_frame` at bag stamp
`record_site_pendant_20260911_220406`:

- xyz `(0.464195, -0.361791, 0.317075)` m
- rpy `(3.09534192, 1.48765098, 1.53052399)`

`T_end_optical` (`elfin_end_link ← d555_color_optical_frame`) stays the
2026-09-11 CC600 ChArUco mean (TSAI / PARK / DANIILIDIS). After the adapter
moved to seat the CAD Livox pocket under that frozen pose, Layer 3 was
**re-expressed** on the mounter so this optical transform did not change.

## Correct camera size

The printed pocket was cut for **D555**, not a D435 90 mm body.

| | D435 (wrong visual, old URDF) | **D555 Datasheet v1.1 (correct)** |
|---|---|---|
| Envelope | 25.05 × 90 × 25 mm (D × L × H) | **48 × 167 × 42 mm** (D × L × H) |
| Mass | 72 g | 337 g |
| Stereo baseline | 50 mm | 95 mm |

URDF visual and collision use that D555 envelope in `camera_link`:

- box size `(0.048, 0.167, 0.042)` m
- origin `(-0.024, -0.0475, 0)` — front face at `camera_link` +X = 0, body
  extends −X into the mount; Y offset is half of the 95 mm baseline (left IR
  = `d555_link`). This Y offset is **not** an Intel CAD-to-IR datum.

A 90 × 25 × 25 mm box sits ~30 mm off the mounter. The D555 envelope sits
~1 mm from the adapter STL (sampled vertices). That gap was a drawing error,
not a ChArUco error.

Gazebo still uses the historical D435 `rgbd_camera` FOV plugin on
`camera_link`. That is a **tracked simulation deviation**. It does not
authorize a D435 housing on the real tree.

## Files

| Role | Path |
|---|---|
| Flange | `src/luggage_description/config/suction_flange_origin.xacro` |
| Adapter | `src/luggage_description/config/eef_mount_adapter_origin.xacro` |
| Livox pocket + optical | `src/luggage_description/config/mid360_origin.xacro` |
| Camera Layer 3 | `src/luggage_description/config/camera_mount_origin.xacro` |
| D555 body | `src/luggage_description/urdf/realsense_d435.urdf.xacro` |
| Solver | `src/luggage_description/luggage_description/livox_d555_align.py` (`locked_eef_livox_tree`) |
| Freeze record | `src/luggage_perception/config/livox_d555_table_icp_frozen.yaml` |
| Seating viz | `src/luggage_perception/scripts/viz_eef_tf_fit.py` |

Calib dump with this tree applied (not in git):

`/home/adamliao/work/robotarm_bags/calib_out/record_site_pendant_20260911_220406_xy_balanced_urdf/`

## Must not

- Move `elfin_end_link → suction_panel`.
- Write table ICP into `mid360_mount_xyz` / `mid360_mount_rpy`.
- Change `cam_mount_*` unless a new ChArUco `T_end_optical` is solved and
  Layer 3 is re-derived.
- Draw or collide a D435 90 × 25 × 25 mm housing for the wrist camera.
- Play calib bags onto `ROS_DOMAIN_ID=7`.
