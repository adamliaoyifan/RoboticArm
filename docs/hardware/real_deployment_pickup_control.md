# Real Deployment, Pickup XY, And Smooth Control

This note is the current real-cell contract. It does not add Orin-specific
scripts because the Orin environment is not fixed yet.

## Current ThinkPad All-In-One

Use one ROS 2 domain on the LAN:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source install/setup.bash
source deployment_ws/install/setup.bash
ros2 launch luggage_planning hardware_pick.launch.py
```

Default launch shape:

- `start_d555:=true`: RealSense D555 owner.
- `start_executor:=true`: one Huayan CPS owner.
- `start_scene:=true`: one scene TF owner.
- `start_perception:=true`: preprocessor, semantic segmenter, semantic point
  filter, detector.
- `start_planning:=true`: scene manager, waypoint generator, motion planner,
  vacuum controller, and MoveIt when `use_moveit:=true`.
- `execution_backend:=waypoint`: production default.

Detection and pick sequence remains:

```bash
ros2 run luggage_planning hardware_pick_driver.py --detect-only
ros2 run luggage_planning hardware_pick_driver.py --plan-only
ros2 run luggage_planning hardware_pick_driver.py
```

## Future Split Deployment

Both machines use native ROS 2 DDS on the same LAN:

```bash
export ROS_DOMAIN_ID=7
```

Orin owns perception only:

```bash
ros2 launch luggage_planning hardware_pick.launch.py \
  start_d555:=true \
  start_executor:=false \
  start_scene:=false \
  start_perception:=true \
  start_planning:=false \
  use_moveit:=false
```

Lenovo owns planning and hardware execution:

```bash
ros2 launch luggage_planning hardware_pick.launch.py \
  start_d555:=false \
  start_executor:=true \
  start_scene:=true \
  start_perception:=false \
  start_planning:=true \
  use_moveit:=true
```

There must be exactly one owner for D555, one owner for CPS, and one owner for
scene TF. The Lenovo graph consumes `/luggage/*` perception outputs published
by Orin.

## Pickup XY Contract

Do not change `luggage_msgs/DetectedLuggage.msg` for pickup XY selection.
The selected suction contact XY is encoded in both:

- `DetectedLuggage.pose.position.x/y`
- `DetectedLuggage.top_surface_pose.position.x/y`

`top_surface_pose.position.z` remains the measured contact Z. Consumers should
continue to refuse motion when `top_surface_valid=false`.

## Offline Pickup Benchmark

Use real bags from `~/robotarm_bags` plus the preserved floor-pick bag. Manual
labels are the ground truth:

```json
{
  "labels": [
    {
      "bag_path": "/home/user/robotarm_bags/site_pick_001",
      "stamp": 1790000000.123,
      "frame_id": "site_pick_001@1790000000.123000000",
      "suction_safe_lid_center_pixel": [321, 188],
      "suction_safe_lid_center_world_xy": [0.482, -0.117],
      "lid_polygon_pixel": [[280, 150], [360, 148], [370, 230], [274, 226]]
    }
  ]
}
```

Candidate extraction should compare at least:

- `pca_center`: current live output.
- `yolo_bbox_center_top_plane`: YOLO bbox center projected to the top plane.
- `lid_inlier_center`: top-surface inlier center.
- `robust_blended_center`: robust blend selected after scoring.

Score labels and extracted candidates with:

```bash
python3 -m luggage_perception.eval.pickup_xy_benchmark \
  --labels labels.json \
  --candidates candidates.json \
  --strategy robust_blended_center \
  --output summary.json
```

Acceptance for the selected strategy:

- median world-XY error <= 0.030 m.
- P95 world-XY error <= 0.060 m.
- improves over `pca_center` on at least 80% of comparable labeled frames.

Outlier frames must keep replay artifacts under `docs/status/evidence/` before
rerunning the same gate.

## ServoJ Qualification

Production pick remains `execution_backend:=waypoint`. The only smooth-control
backend added for qualification is:

- `execution_backend:=servo_j`
- CPS calls: `StartServo` once, then `PushServoJ` absolute joint degrees on a
  fixed time grid.

Do not use `servo_esj`. Do not use `MovePathJOL` for realtime smoothing.

Hardware gates before using ServoJ for any dry pick:

- S0: `StartServo` only, no motion.
- S1: push current joints for 1-2 s.
- S2: small empty-space motion with no waypoint-style 0.16-0.20 s stops.
- S3: cancel mid-motion, verify `GrpStop` and recovery.

Only after S0-S3 pass may `execution_backend:=servo_j` be considered for a dry
pick. It must not be enabled for live pickup by default.
