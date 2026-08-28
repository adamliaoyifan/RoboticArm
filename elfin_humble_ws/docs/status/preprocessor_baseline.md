# Sensor preprocessor baseline (Fortress)

Date: 2026-08-21  
Launch: `ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false`  
Pose: named `observe` (launch default), then a 0.35 rad `elfin_joint1` nudge and return.

This is the first live run of `sensor_preprocessor_node` in the Humble
Fortress world. It is evidence, not a contract.

## What published

After the arm reached `observe` and the motion gate went `stable`:

| Stream | Observed |
|---|---|
| `/luggage/preprocessed/camera/color/image` | yes, encoding `rgb8`, stamp = RGB primary |
| `/luggage/preprocessed/camera/depth/image` | yes, **16UC1 millimetres** |
| `/luggage/preprocessed/camera/depth/points` | yes, `frame_id=camera_depth_optical_frame`, **0 NaN/inf** |
| `/luggage/preprocessed/status` | `schema=luggage.preprocessed.status.v1`, transient-local |

Matched RGB/depth headers shared one primary stamp. Status reported
`data_frame=camera_link`, `lidar_ok=false`, `deskewed=false`, and camera
output continued without a lidar stream.

Callback counts on a long-lived node were essentially equal across RGB,
depth, camera_info, points, and joints (~1130). Output rate was **~4–6 Hz**
versus the raw camera **~30 Hz**: each full-resolution cloud (~271k finite
points after dropping ~35k inf far-plane hits) is decoded and transformed
in Python. Pairing stays RGB-primary; the node does not invent a second
alignment loop.

gz `ros_gz_bridge` currently **offers RELIABLE** on the D435 topics. The
preprocessor therefore subscribes RELIABLE (BEST_EFFORT subscriptions on
this RMW delivered only a trickle). Outputs stay BEST_EFFORT for the detector.

## Frame correction (M2 invariant)

World-frame z histograms on the same raw cloud:

| Transform source | z>1.7 (impossible overhead) | z∈[0.8, 1.2] (platform/box band) |
|---|---|---|
| Header optical (`camera_depth_optical_frame`) | **196419** | 24052 |
| Parameter `camera_link` | 10818 | **50832** |
| Preprocessed cloud, header optical | 10818 | **50832** |

The preprocessed cloud matches `camera_link` axes after the stamped
`camera_depth_optical_frame <- camera_link` lookup. Residual z>1.7 at the
`observe` pose is scene geometry, not the M2 label/data mismatch
(that mismatch produced ~196k impossible points).

## Motion gate

Nudge `elfin_joint1` by 0.35 rad via FollowJointTrajectory:

- While the arm was moving, status showed `motion_too_large=true`,
  `geometry_ok=false`. RGB `rgb_ok` stayed true (images still published).
- After returning to `observe` and waiting for `settle_time_sec=0.5`,
  status returned to `geometry_ok=true`, `state=stable`, and clouds resumed
  (`cloud` buffer occupancy 2, `cloud_ok=true`).

## DetectLuggage vs GetCurrentBox

`DetectLuggage` used the **perception path** (`perception estimate (conf=1.00)`),
not GT fallback. One settled sample at the launch `observe` pose:

| | width | depth | height | center xyz |
|---|---|---|---|---|
| GT (`GetCurrentBox`) | 0.729 | 0.459 | 0.288 | (-1.000, 0.000, 1.004) |
| DetectLuggage | 0.222 | 0.111 | 0.216 | (-0.683, -0.197, 0.968) |

Height is in the right band (the M2 optical-header bug produced height ~0.97
and z ~1.35). XY and the footprint are still off because **`observe` is not
`pickup_observe`**: the wrist camera does not see a clean top-down of the
pickup box. This is a viewpoint limit of the launch default, not a
regression of the preprocessor's frame correction. Re-run at
`observe_pose_name:=pickup_observe` before treating size error as a detector
regression.

Stopping a sensor was not exercised as a process kill in this run. Unit
tests cover missing/stale joints (`geometry_ok=false`, no cloud) and
invalid stamps (no silent republish). Runtime status already reports
`lidar_ok=false` without starving the camera.

## Tests

```text
colcon test --packages-select luggage_perception
# 160 passed, including test_stamp_ring_buffer and test_sensor_preprocessor
```
