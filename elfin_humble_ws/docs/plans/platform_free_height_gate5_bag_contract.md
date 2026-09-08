# Platform-free Height Gate 5 Rosbag Contract (PF-A2)

Date: 2026-09-04

Companion: `docs/plans/platform_free_height_test_plan.md` Gate 5,
`docs/plans/platform_free_height_remediation.md` PF-A2.

Implementation: `src/luggage_perception/luggage_perception/eval/gate5_bag_readiness.py`
CLI: `scripts/gate5_bag_readiness.py`

This contract is backend-neutral. It does not depend on a bag that does not
yet exist. Passing the readiness checker is necessary but **not sufficient**
for Gate 5 accuracy. The checker never sets `gate5_accuracy` to passed.

## Manifest

Recordings are described by a JSON manifest (`schema`:
`gate5_bag_manifest/v1`). A future rosbag2/mcap indexer must emit this
schema; synthetic fixtures use the same object.

Required fields:

- `topics[]`: `name`, `type`, `role`, `count`, time range (`t_start_sec` /
  `t_end_sec` and/or `stamps_sec`)
- `frame_ids`
- `tf_coverage[]`: `stamp_sec`, `parent`, `child`, `present`
- `coverage`: sizes, placements, settled count, platform heights, support
  visibility flags
- `reference`: offline label topics and `publish_to_online`

Roles:

| role | Meaning |
|---|---|
| `online` | algorithm input (sensors, preprocessed streams, TF, masks) |
| `task_state` | `/luggage/current_box` id/generation only |
| `output` | detector `DetectionFrame` for comparison |
| `reference` | offline reconstruction / eval GT; never an online input |

## Required online topics

| Topic | Type |
|---|---|
| `/luggage/preprocessed/camera/color/image` | `sensor_msgs/msg/Image` |
| `/luggage/preprocessed/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/luggage/preprocessed/camera/depth/image` | `sensor_msgs/msg/Image` |
| `/luggage/preprocessed/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/luggage/preprocessed/camera/depth/points` | `sensor_msgs/msg/PointCloud2` |
| `/luggage/preprocessed/status` | `std_msgs/msg/String` |
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/tf` | `tf2_msgs/msg/TFMessage` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` |

At least one non-empty deterministic segmentation input:

- `/luggage/semantic/mask` (`sensor_msgs/msg/Image`), or
- `/luggage/semantic/yolo_detections` (`luggage_msgs/msg/YoloDetections`)

Detector output (recorded, not an algorithm input):

- `/luggage/perception/detection_frame` (`luggage_msgs/msg/DetectionFrame`)

## Isolation

Offline references must not be published onto online algorithm topics.
`GetCurrentBox`, `/luggage/perception/size_eval/spawned`, and reconstructed
boxes are `reference`. `/luggage/current_box` is `task_state`, never `online`.

## Coverage metadata (dataset minimum)

- at least three luggage sizes
- at least three XY/yaw placements per size
- at least 30 settled observations
- at least one platform height other than 0.86 m
- visible-support and support-occluded cases

## Checker reasons

| Code | Failure |
|---|---|
| `BAG_MISSING_TOPIC` | required online topic absent |
| `BAG_WRONG_TYPE` | ROS type mismatch |
| `BAG_EMPTY_STREAM` | count is 0 |
| `BAG_DURATION_SHORT` | overlapping online window too short |
| `BAG_STAMP_NON_MONOTONIC` | stamps go backwards |
| `BAG_WINDOWS_NO_OVERLAP` | online streams do not share a time window |
| `BAG_MISSING_FRAME` | required `world` or `camera_depth_optical_frame` missing |
| `BAG_TF_GAP` | stamped TF parent/child missing at an acquisition sample |
| `BAG_SEGMENTATION_INPUT_MISSING` | no replayable mask/YOLO stream |
| `BAG_REFERENCE_LEAK` | offline labels on online topics |
| `BAG_COVERAGE_INSUFFICIENT` | dataset metadata below Gate 5 minimum |
| `BAG_GATE5_ACCURACY_NOT_CLAIMED` | caller asked to pass accuracy |
| `BAG_SCHEMA_INVALID` | not a v1 manifest |

Exit status is nonzero when any reason is present. A ready synthetic fixture
exits 0 with `gate5_accuracy: not_claimed`.
