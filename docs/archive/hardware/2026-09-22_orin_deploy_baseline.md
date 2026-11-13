# Orin deploy baseline (2026-09-22)

This is the frozen real-cell bringup. Later perception or planning edits
must not change the machine split, the entry scripts, or the topic names
below. Tune rates and gates inside the nodes; do not replace this flow
with another launch.

Host observed: `hkureconova` (aarch64, Orin, ROS 2 Humble), `ROS_DOMAIN_ID=7`.
Profile: D555 colour and aligned depth `640×360 @ 15`, RGB8 / Z16.
These rates are one session's measurements, not a scored campaign.

## Flow

Orin, this repo:

```bash
cd deployment_ws
./scripts/orin_perception.sh
```

That script checks Humble, CUDA, and jumbo MTU, then runs
`ros2 launch luggage_perception orin_perception.launch.py`.
It refuses `semantic_device:=cpu`.

ThinkPad (Jazzy), other machine, same domain. Do not start a second D555
or YOLO:

```bash
cd deployment_ws
source scripts/env_jazzy_real.sh
./scripts/hardware_pick.sh \
  start_d555:=false start_perception:=false \
  start_planning:=true start_executor:=true start_scene:=true \
  use_moveit:=true use_rviz:=false
ros2 run luggage_planning hardware_pick_driver.py --detect-only
```

Floor-box driver wrapper, after that graph is up:

```bash
./scripts/run_floor_box_pick.sh              # plan only
./scripts/run_floor_box_pick.sh execute      # motion after a yes
```

`OBSERVE_POSE` defaults to `current` (no extra joint move).

## What stays put

| Piece | Role |
|---|---|
| `deployment_ws/scripts/orin_perception.sh` | Orin entry. Humble only. |
| `src/luggage_perception/launch/orin_perception.launch.py` | D555 remap to `*_hw`, adapter, preprocessor, YOLO, filter, detector, Livox. No CPS, no MoveIt. |
| `deployment_ws/config/d555_orin.yaml` | Driver profile `640,360,15`. Launch forces `pointcloud.enable: false`. |
| `src/luggage_perception/config/preprocessor_d555_orin.yaml` | Mapped raw colour and aligned depth. Not the replay yaml. |
| `deployment_ws/scripts/env_jazzy_real.sh` | ThinkPad overlay. `ROS_DOMAIN_ID=7`. |
| `deployment_ws/scripts/hardware_pick.sh` | ThinkPad planning graph. Cameras off when Orin owns perception. |
| `deployment_ws/scripts/run_floor_box_pick.sh` | Detect / plan / execute wrapper. |

Overlay for debug is on by default (`/luggage/semantic/overlay`).
Pass `publish_overlay:=false` on the Orin script to drop that image.

Clock path stays: driver `image_hw` (device clock) → adapter maps to host
time → `image_raw`. Do not publish unmapped device stamps on `image_raw`.

## Measured rates (overlay on)

8 s sample, 2026-09-22, stack up about 30 s. Receive rate on this host.
A best-effort subscriber can drop samples (`sequence size exceeds remaining
buffer`); camera counts near 15 Hz were stable.

| Topic | Rate | Age when received |
|---|---|---|
| `/camera/d555/color/image_raw` | 14.9 Hz | 29 ms |
| `/camera/d555/aligned_depth_to_color/image_raw` | 14.9 Hz | 20 ms |
| `/luggage/preprocessed/camera/color/image` | 13.8 Hz | 33 ms |
| `/luggage/preprocessed/camera/depth/image` | 14.9 Hz | 36 ms |
| `/luggage/semantic/mask` | 6.1 Hz | 139 ms |
| `/luggage/semantic/yolo_detections` | 6.1 Hz | 139 ms |
| `/luggage/semantic/overlay` | 5.8 Hz | 228 ms |
| `/luggage/semantic/cargo_points` | 6.1 Hz | 202 ms |
| `/joint_states` | 100 Hz | stamp about 21 ms ahead of receipt |

Without overlay, the same YOLO path was about 8.2 Hz and the mask age was
about 180 ms. Overlay drawing is why the later sample is nearer 6 Hz.

Mapped colour and aligned depth shared stamps (90/90 in one 6 s window).
Preprocessed colour and depth shared stamps. YOLO stamps match the
preprocessed colour stamp. Image stamps sat within about ±5 ms of the
nearest joint sample. `/clock_sync/d555` was `locked`, epoch 0, no reset;
device-to-host offset about 16.6 ms.

## Where the YOLO time goes

`bbox_fill` on this Orin, 640×360, CUDA, median of 6 frames with the
segmenter otherwise idle:

| Stage | Time |
|---|---|
| `YOLOWorld.predict` | 34 ms |
| Temporal-gate `rgb_signature` | 22 ms |
| Mask rectangle fill and GPU box copy-back | ~0 ms |
| `update` total | 59 ms |

The mask image is not the slow part. A 15 Hz frame is 67 ms, so one
callback cannot emit every camera frame. The colour subscription keeps
only the latest frame. Cargo clouds whose stamp is more than 200 ms
behind the newest depth are dropped so DetectLuggage does not pair an
old cloud with an evicted depth.

Live detector plane gates used for this bringup (in-process, yaml file
unchanged): top normal 20°, suction tilt 20°, normal-deviation P95 15°,
connected-plane fraction 0.70. Failures log the candidate tilt or the
suction reject counts on `DetectLuggage`.
