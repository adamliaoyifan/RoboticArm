# D555 PoE hardware bring-up verification

Status: plan for read-only hardware fact-finding. Not an architecture change.
No URDF, no message contract, and no acceptance-bar edits are authorised here.

Parent task: `D555-BRINGUP-20260908`.

## Why

The installed wrist camera is a **D555 PoE** (SN `419222302385`), confirmed by
the user. The mount was designed for D555, so mechanical fit is not in
question. What is unverified is every number the online stack would consume.

A 2026-09-08 hardware visualisation showed the D555 RGBD cloud roughly coplanar
with Mid-360 floor points, which rules out a gross mount error but cannot
resolve either the RGB-to-depth alignment question or errors at the 6-9 cm scale
that a wrong depth/colour extrinsic produces on a 95 mm baseline.

Two open threads are blocked on exactly these facts:

- `Q-20260908-2` records the camera decision as needing the canonical body
  frame, the measured mount transform, and the Humble-versus-Jazzy answer.
- `Q-20260908-3` asks whether the canonical depth image is colour-aligned or
  depth-native for non-identity optics. D555 has both an origin offset and an
  FOV difference (depth 87x58, RGB 90x65), so this cannot be answered from a
  datasheet.

This plan collects those facts. It deliberately does **not** change code.

## Known state to verify against

| Item | Recorded value | Source |
|---|---|---|
| Camera | D555 PoE, `192.168.11.55`, FW `7.56.37776.6014` | `20260902_183500_eef_livox_d555/NOTES.md` |
| Host | `192.168.11.70`, MTU 9000, librealsense 2.58.4 | same |
| Stable stream | 640x360 at 15 Hz RGB + depth | same |
| Known failure | 896x504 at 30 Hz dropped the DDS device | same |
| Distro at bring-up | native ROS 2 Jazzy | same |
| Mount transform | `eef_mount_adapter` to `camera_link` = `0.013 0.097 -0.021`, rpy `0.03770 1.36345 1.57080` | `camera_mount_origin.xacro`, saved by `eof_mount_stack_tune_gui.py` |
| Body frame | `camera_link` to `d555_link` identity | `realsense_d435.urdf.xacro` |
| Datasheet | baseline 95 mm, depth FOV 87x58, RGB FOV 90x65, Min-Z ~26 cm VGA, 167x42x48 mm, 337 g | D555 Datasheet v1.1 |

Sim, for contrast: 640x480 at 30 Hz, HFOV 1.5184 rad, `fx = fy = 337.222`,
depth clip 0.105-3.0 m, `mass 0.072`, `baseline 0.05`,
`depth_to_color.translation = [0, 0.015, 0]`.

## Safety

Hardware-in-the-loop rules from `ros2_humble_mvp_and_migration_plan.md` apply:
read-only, low speed, small amplitude. HB-1 requires no arm motion at all.
HB-2 and HB-3 use static poses only, commanded one at a time, with the operator
present. Do not run a packing cycle. Do not command the suction gripper.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| HB-1 | `cursor/grok-4.6` | none | Dump every D555 device fact the online stack would consume, arm stationary | All nine HB-1 items recorded; frame-versus-K consistency verdict is explicit | Read-only topic, TF, and parameter inspection |
| HB-2 | `cursor/grok-4.6` | HB-1 | Quantify RGB-to-depth alignment with a positive test, not visual inspection | Pixel reprojection error reported at three depths; aligned-versus-native verdict stated with evidence | Target overlay at known distances |
| HB-3 | `cursor/grok-4.6` | HB-1 | Measure the mount-extrinsic residual against Mid-360 and separate translation from rotation error | Plane offset in mm and angle in deg at three arm poses, plus the translation-versus-rotation conclusion | Plane fitting on both clouds |

HB-2 and HB-3 may be reported in one evidence run if the same session covers
both. HB-1 must be separable because it needs no arm.

### HB-1 -- device fact dump

Record, with the exact command and raw output preserved under evidence:

1. Full D555 topic list with `ros2 topic info -v`: exact names, types, and QoS
   (reliability, durability, history, depth) for colour image, depth image,
   both `camera_info`, any point cloud, and IMU.
2. The driver's published frame tree (`tf2_tools view_frames` plus
   `/tf_static` echo). State the actual frame names and whether the body origin
   coincides with left IR / depth as the D400 wrapper convention implies.
3. `/extrinsics/depth_to_color`: full rotation and translation. Compare against
   the D450 nominal (about -59 mm on Y) and report the delta.
4. Depth and colour `camera_info` at the profile actually running: `width`,
   `height`, full `K`, `D`, `R`, `P`, and `distortion_model`. Do not derive
   these from the sim `fx = 337.222`; the hardware aspect ratio is 16:9 against
   the sim's 4:3, so vertical FOV differs and K is not a rescale.
5. Whether `align_depth` exists on the SafeDDS path, and whether it is on.
6. The `frame_id` carried by the depth image and by any point cloud, and
   **whether that frame is consistent with the K being used**. This is a
   correctness check with a single right answer: an aligned product must sit in
   colour optical, an unaligned product in depth optical.
7. Measured rate of every stream (`ros2 topic hz`) over at least 30 s, and
   whether 640x360 at 15 Hz is still the stable ceiling. If a higher profile is
   attempted, record whether the DDS device drops as it did at 896x504 at 30 Hz.
8. Driver, librealsense, and firmware versions, plus the **ROS distro actually
   running**. State plainly whether D555 DDS streaming worked on Humble or
   required Jazzy.
9. The RViz `Color Transformer` setting used for the RGBD display in the
   2026-09-08 screenshot. A FlatColor or Intensity setting means that image
   carries no colour information and cannot support any alignment claim.

Pass condition: all nine recorded, with item 6 stated as an explicit
consistent-or-inconsistent verdict rather than a raw dump.

### HB-2 -- RGB-to-depth alignment

Visual coplanarity is not evidence. Use a positive test.

1. Place a high-contrast planar target with a sharp colour boundary (checker
   board, or tape cross on a box face) at approximately 0.6 m, 1.0 m, and 2.0 m.
   Stay at or beyond the 26 cm Min-Z and prefer the 0.6-6 m ideal range.
2. For each distance, project depth points into the colour image using the
   colour `K` and the measured `depth_to_color`, and report the **pixel offset**
   between the colour boundary and the depth discontinuity. Report signed u and
   v offsets, not a magnitude.
3. Convert the offset to metres at that depth and compare against the 95 mm
   baseline and the ~59 mm nominal colour offset. State which extrinsic
   assumption the data supports.
4. State the verdict: is the delivered depth product colour-aligned or
   depth-native? Give the evidence, not the datasheet expectation.
5. If a coloured cloud topic exists, verify colour is actually populated and
   not a constant.

Pass condition: signed pixel offsets at three depths, the metric conversion,
and an evidence-backed aligned-or-native verdict. A null result is acceptable
if it is quantified; "looks right" is not.

### HB-3 -- mount extrinsic residual

Converts the visual coplanarity into a number, and separates the two error
types so it can seed a later hand-eye calibration.

1. At three distinct static arm poses that all see a common flat floor region,
   capture D555 depth and Mid-360 simultaneously.
2. Fit a plane to the floor points from each sensor independently, transform
   both into `elfin_base`, and report the plane-to-plane **offset in mm** and
   **angle in deg** per pose.
3. Interpret: an offset roughly constant across poses indicates a mount
   translation error; an offset that varies with pose indicates a mount
   rotation error. State which pattern the data shows.
4. Report the residual against the current GUI-tuned value so a CAD-seeded
   replacement can be judged later.
5. Note the observed Mid-360 edge doubling seen in the 2026-09-08 screenshot:
   state whether it is RViz `Decay Time` accumulation or genuine motion
   distortion from absent deskew. Do not fix deskew here.

Pass condition: nine numbers (three poses times offset, angle, and pose id),
the translation-versus-rotation conclusion, and the smearing verdict.

## Out of scope

- Any URDF, xacro, or yaml edit. In particular do **not** write a new mount
  transform; HB-3 measures the residual, it does not apply a correction.
- Any hand-eye calibration run. That is later work seeded by HB-3 and CAD.
- Any message-contract or topic-remap change. `Q-20260908-3` owns that.
- Any change to PF-R acceptance bars. The 15 Hz re-derivation is a separate
  decision in `Q-20260908-2`.
- Resolving the `realsense_d435.yaml` `tune_joints` versus `fixed.rpy`
  inconsistency. Report it if encountered; do not edit it.
- Creating a D555 sim model.

## Evidence

`docs/status/evidence/d555_bringup/<run>/` with the tested commit and
dirty-file count, per `docs/agents/FORMAT.md` section 12. Raw dumps and images
belong there, never under `docs/agents/`.

## Pointers

- `docs/agents/discuss/2026-09-08_1643_d455-sim-parity-and-min-z-conflict.md`
- `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`
- `src/luggage_description/config/backups/20260902_183500_eef_livox_d555/NOTES.md`
- `src/luggage_description/config/camera_mount_origin.xacro`
- `src/luggage_description/config/realsense_d435.yaml`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/plans/d455_replace_d435/README.md`
