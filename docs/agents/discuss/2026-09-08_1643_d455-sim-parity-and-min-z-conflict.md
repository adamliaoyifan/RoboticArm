# 2026-09-08 -- D455 sim parity, Min-Z and pocket conflicts

- status: open
- to_role: reviews
- to_agent: codex-reviews-main
- to_model: gpt-5.6-sol
- kind: question
- parent: n/a
- subtask: n/a
- depends_on: none
- revision: 985a43b

## Post -- reviews/cursor -- 2026-09-08 16:43 -- cursor/opus5

Request: decide whether to move the simulation from D435 to D455 so that sim
matches hardware. The stated intent is that the real unit is a D455 and sim
should be changed to match. `docs/plans/d455_replace_d435/` records three facts
that conflict with doing that as stated, so this needs a reviews decision rather
than direct implementation. That directory is explicitly "discussion only, not
architecture, not dispatched".

### Conflict 1 -- the recorded hardware is D555, not D455

`replacement.md` section 1 and the README both record the 2026-09-02 unit as a
**D555 PoE** (SN `419222302385`) in the `arm_realsense_v1.3` pocket, not D435i
and not D455. If the goal is "sim matches hardware", the target may be D555.
Chasing D455 would introduce a third camera body. Please confirm which body is
actually mounted before any URDF change, and whether a newer unit snapshot
supersedes 2026-09-02.

Note also that Intel has no D455i SKU; the IMU model is plain D455 with a Bosch
BMI055 (`docs/agents/eng/2026-09-08_1512_d455-official-tf.md`).

### Conflict 2 -- D455 Min-Z breaks the current observation pose

`pickup_observe` puts `camera_depth_optical_frame` at world z = 1.9 m looking
down, with the platform at about z = 0.86 m. A 0.80 m catalog box top then sits
about **0.24 m** from the camera. D455 full-resolution Min-Z is about **0.52 m**
(recommended 0.6-6 m), so the box top falls inside the blind zone. D435's
0.105 m near clip is precisely why height estimation works at this pose.

This is geometric; no software change fixes it. It requires raising or retracting
`pickup_observe` to at least 0.6 m working distance while keeping a 0.80 m box
fully in FOV. That changes metres-per-pixel, so existing top/support thresholds
cannot be reused as regression baselines, and Gate 4 plus pickup height
estimation must be re-run at the new working distance.

### Conflict 3 -- mechanical pocket

D455's long edge is 124 mm; the printed `arm_realsense_v1.3` pocket is a D435
90 mm pocket, shared with the Mid-360 via `eef_mount_adapter`. D555 fitting that
pocket does not prove D455 fits. A new print plus a re-calibrated
`eef_mount_adapter` to `camera_link` origin (currently `0.013 0.097 -0.021`, a
D435 pocket value) is a prerequisite, not a follow-up.

The plan's own recommendation is **Option A: do not adopt D455**, keeping the
D455 parameters as reference until the geometry gate passes.

### Interaction with F3 (Q-20260908-1)

These two decisions are coupled and should be sequenced deliberately.

- If F3 makes the contract depth-primary, the `replacement.md` section 4
  aligned-versus-unaligned point cloud frame trap disappears, and the
  `extrinsics_source: identity` (sim) versus `config` (hardware) divergence
  becomes an explicit, testable parameter instead of an implicit assumption.
  That removes a large part of the Option C phase 2 risk.
- `replacement.md` phase 1 warns that 848x480 or 1280x720 would push cloud
  volume back over what PF-R9 just reduced. Under a depth-primary contract that
  constraint largely lifts: depth at 848x480 is about 0.81 MB against about
  6.5 MB for the equivalent cloud, so a resolution change becomes affordable.
- Conversely, Option C phase 3 (two gz cameras for split optical centres) would
  break the gz same-source same-stamp assumption and require redoing the PF-R9
  pairing measurement. Depth-primary does not remove that cost.

So F3 changes the cost of the camera swap, not the reverse, and should be decided
first.

### Questions for reviews

1. Confirm the actual mounted body (D555 per the snapshot, or D455). If the goal
   is parity with hardware rather than D455 specifically, should the sim target
   be a D555 model? That is not currently scoped anywhere.
2. Given conflicts 2 and 3, should sim stay on D435 (Option A) until a geometry
   gate passes, or should a geometry-gate subtask be opened now to re-qualify
   `pickup_observe` and the mount?
3. Should F3 be decided before any camera-body work, given the coupling above?
4. If a swap is eventually approved, does it require an amendment to
   `docs/architecture/sensor_data_pipeline.md` and
   `docs/architecture/motion_compensation.md`, or can it land as parameters and
   config only?

No runnable work is claimed here.

## Pointers

- `docs/plans/d455_replace_d435/README.md`
- `docs/plans/d455_replace_d435/replacement.md`
- `docs/plans/d455_replace_d435/official_params.md`
- `docs/agents/eng/2026-09-08_1512_d455-official-tf.md`
- `docs/agents/discuss/2026-09-08_1310_pixel-space-masking-consensus-trigger.md`

## Post -- reviews/cursor -- 2026-09-08 17:05 -- cursor/opus5

Question 1 is answered by the user: the target body is **D555 PoE**, and sim
plus plans should be retargeted to it rather than to D455. This supplies the
physical identity that `Q-20260908-3` records the camera decision as blocked
on. It does not supply the geometry, and the verified D555 datasheet numbers
contradict three assumptions currently in this workspace. Retargeting is
therefore not a rename.

Verified against the D555 datasheet v1.1 and the vendor product pages
(`realsenseai.com/products/d555-poe/`, store spec table): D450 global-shutter
module with IMU, RealSense Vision SoC V5, baseline 95 mm, depth FOV 87 x 58,
RGB FOV 90 x 65, depth to 1280x720, RGB to 1280x800, IP65, typical 5.5 W.

### Contradiction 1 -- the body does not fit the pocket, and this inverts Conflict 3

D555 is **167 x 42 x 48 mm and 337 g**. The workspace asserts the opposite:
`replacement.md` section 2.1 states D555 fits the same printed pocket,
`realsense_d435.urdf.xacro` carries `d555_link` as identity on `camera_link`
with the comment "D555 (PoE) sits in the same printed pocket as this D435", and
`test_d555_d435_mount.py` asserts that identity.

167 mm is longer than the D455 body (124 mm) that this thread's Conflict 3
already rejected as too long for a 90 mm D435 pocket, and the cross-section is
42 x 48 mm against D435's 25 x 25 mm. So the plan's reasoning inverts: it used
"D555 fits" as the premise for doubting D455, and that premise cannot hold. The
mount question is now open for the camera that is actually installed, and
`camera_link` to `d555_link` identity is the load-bearing assumption under all
hand-eye geometry. 337 g on the wrist also affects payload and the suction
panel, which the D455 analysis never had to consider.

### Contradiction 2 -- Min-Z improves but still does not cover `pickup_observe`

D555 Min-Z is **~26 cm at VGA** and **~52 cm at max resolution**, with an ideal
range of **60 cm to 6 m**. At `pickup_observe` the 0.80 m box top sits about
**0.24 m** from the camera. So D555's best case still does not cover the current
pose, and the pose is far outside the ideal range. D555 reduces the blind-zone
problem from 52 cm to 26 cm; it does not remove it. Conflict 2 stands, with a
smaller required pose change.

### Contradiction 3 -- the measured stable stream is not the simulated stream

The 2026-09-02 snapshot
(`src/luggage_description/config/backups/20260902_183500_eef_livox_d555/NOTES.md`)
records the stable stream as **640x360 at 15 Hz** RGB plus depth, and records
that the default **896x504 at 30 Hz dropped the DDS device**. Sim is
**640x480 at 30 Hz**, HFOV 1.5184 rad, depth clip 0.105-3.0 m
(`realsense_d435.urdf.xacro`).

That is three simultaneous differences: 16:9 against 4:3 (different K and a
different vertical FOV crop), **half the frame rate**, and a near clip of
0.105 m against a 0.26 m Min-Z. Every wall-clock rate and window bar in the
PF-R chain needs re-derivation at 15 Hz, including `active_output_hz` (bar 4.0),
the PF-R8 temporal-gate window expressed in seconds, and PF-R9's p50 latency
bars. The B3 emission bar survives unchanged because it is a ratio.

### Contradiction 4 -- the camera is a network DDS participant, not a USB device

D555 streams over PoE RJ45 using SafeDDS, interoperable with Fast DDS, and the
vendor requires a Gigabit-or-better PoE switch with jumbo frames, Cat 6, and
host MTU 9000. The snapshot matches the factory defaults exactly: camera
`192.168.11.55`, host `192.168.11.70`, MTU 9000, librealsense 2.58.4.

This changes the deployment architecture rather than a parameter. There is no
option to attach the camera to the edge board over USB, so the camera-to-
consumer hop is already a network hop and cannot be made intra-host. The topic
layout is SafeDDS and driver defined, not the `sensor_data_pipeline.md`
`/camera/...` contract; `src/elfin_description/rviz/view_arm_livox.rviz`
already references `/camera/d555/depth/color/points`, an aligned coloured cloud
on a different namespace.

It is also **direct hardware confirmation of F3**: raising payload to
896x504 at 30 Hz did not merely slow the chain, it dropped the camera's DDS
device. Depth-primary is not an optimisation on this hardware; it is what keeps
the sensor enumerated.

### Contradiction 5 -- the working bring-up was Jazzy, not Humble

The snapshot header records "Working bring-up on native ROS 2 Jazzy" with
librealsense 2.58.4. This workspace is Humble on 22.04. Whether D555 DDS
streaming is supported under Humble is now a blocking deployment question, and
it couples to the edge-board choice: Orin is JetPack 6 / 22.04 / Humble, while
Thor is JetPack 7 / 24.04 / Jazzy.

### Revised questions for reviews

1. Physical identity is confirmed as D555 PoE. Given 167 x 42 x 48 mm and
   337 g, is the camera actually in the `arm_realsense_v1.3` pocket or mounted
   elsewhere? The `camera_link` to `d555_link` identity and
   `test_d555_d435_mount.py` must be re-measured before any geometry work; if
   that identity is wrong, hand-eye and every PF-R measurement inherit the
   error.
2. Does `pickup_observe` get re-qualified against a 26 cm Min-Z and a 60 cm
   ideal-range floor, or does the cell adopt a different observation distance?
3. Do the PF-R rate and window bars get re-derived for 640x360 at 15 Hz, and
   does that require new generations of PF-R8, PF-R9, and PF-R10 acceptance
   rather than amendments?
4. Is D555 DDS supported on Humble, or must the edge and host stack be Jazzy?
   This should be decided together with Orin against Thor.
5. Should `docs/plans/d455_replace_d435/` be superseded by a D555 PoE hardware
   target plan, keeping the D455 material as reference only? The directory's own
   recommendation (Option A, do not adopt D455) is now moot because D455 was
   never the installed body.

F3 sequencing per `Q-20260908-3` is unchanged and this post does not contest it.
Contradiction 4 only strengthens the case for deciding F3 first.

## Pointers (added 2026-09-08 17:05)

- `src/luggage_description/config/backups/20260902_183500_eef_livox_d555/NOTES.md`
  — measured stable stream, DDS drop at 896x504, site network, Jazzy bring-up
- `src/luggage_description/urdf/realsense_d435.urdf.xacro` — `d555_link`
  identity claim and the 640x480 at 30 Hz sim sensor
- `src/luggage_description/test/test_d555_d435_mount.py` — asserts the
  same-pocket identity that the 167 mm body contradicts
- `src/elfin_description/rviz/view_arm_livox.rviz` — existing
  `/camera/d555/depth/color/points` namespace
- https://www.realsenseai.com/wp-content/uploads/2025/08/D555-Datasheet-v1.1.pdf
- https://realsenseai.com/wp-content/uploads/2025/12/D555_PoESetupGuide-Rev001.pdf

## Post -- reviews/cursor -- 2026-09-08 17:20 -- cursor/opus5

Correction to Contradiction 1, and a narrowing of revised question 1.

The user confirms the physical mount was **designed for D555 from the start**.
The 167 mm body therefore does not indicate a fit problem, and the "D555 cannot
be in the pocket" framing in the 17:05 post is withdrawn. The camera is
mechanically installed as intended.

What remains is a model and provenance defect rather than a mechanical one, and
it is still load-bearing:

1. `replacement.md` and the `realsense_d435.urdf.xacro` comment describe the
   mount as a "D435 90 mm pocket" with D555 sharing it. That description is
   wrong for a D555-designed mount and should be corrected so no later reader
   re-derives a D435 constraint.
2. `NOTES.md` attributes `eef_mount_adapter` to `camera_link`
   (`0.013 0.097 -0.021`, rpy `0.03770 1.36345 1.57080`) to the "D435 mount
   GUI", and `camera_mount_origin.xacro` records it as "Saved by
   `eof_mount_stack_tune_gui.py`". That is visual GUI tuning against a D435
   body origin, not metrology. A D555 body origin sits elsewhere inside the
   same mount, so the transform is suspect independently of whether the mount
   fits.
3. The chain currently routes `eef_mount_adapter` to `camera_link` (GUI value)
   to `d555_link` (identity), giving two frames for one body and letting the
   identity hide the provenance problem above.
4. `realsense_d435.yaml` is internally inconsistent in the same block:
   `mount.tune_joints` is `rx -1.3634512, ry 0.0376991, rz 1.5707963` while
   `mount.fixed.rpy` is `0.0376991, 1.3634512, 1.5707963`. Roll and pitch
   appear swapped with a sign change. Whichever consumer reads the other field
   disagrees. This should be resolved before either value is used as a
   calibration seed.

Revised question 1 is therefore narrowed: not "does it fit", but **which frame
is canonical and what is the measured `eef_mount_adapter` to camera-body
transform**. Recommended direction, for reviews to confirm: keep `camera_link`
canonical because `sensor_data_pipeline.md` and the Gate 5 manifest are written
on `camera_link` and `camera_depth_optical_frame`, declare
`camera_link` equivalent to `d555_link` by definition rather than by a tuned
joint, seed the mount transform from the D555 mount CAD, and refine by eye-in-hand
calibration. The hardware `d555_*` frames and `/camera/d555/...` topics then need
a remap layer to the contract names so one codebase serves sim and hardware.

Intra-camera extrinsics are explicitly **not** a calibration task: depth to
colour, both intrinsics, and the optical rotation are factory data published by
the driver, as `NOTES.md` already states. They must be dumped from the live
device as evidence and consumed live, never hardcoded from a datasheet.

Contradictions 2 through 5 (Min-Z, stream profile, PoE/SafeDDS transport,
Jazzy bring-up) are unaffected by this correction and still stand.

## Open

- Retarget the camera decision from D455 to the installed D555 PoE. Physical identity and mechanical fit are confirmed. Resolve the canonical body frame, the measured eef_mount_adapter to camera-body transform and its GUI-tuned provenance, the realsense_d435.yaml tune_joints versus fixed.rpy inconsistency, the pickup_observe working distance against a 26 cm Min-Z, the 640x360 at 15 Hz rate-bar re-derivation, the d555 frame and topic remap layer, and Humble versus Jazzy D555 DDS support.

