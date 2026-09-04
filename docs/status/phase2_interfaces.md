# Phase 2 `luggage_msgs` interfaces

Date: 2026-08-14  
Workspace: `elfin_humble_ws`  
Build: `colcon build --packages-select luggage_msgs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`  
Result: **passed**

ROS 1 Catkin `luggage_msgs` was converted in place to `ament_cmake` + `rosidl_generate_interfaces`. Field names, order, and meanings are unchanged. No Action/Service servers were implemented.

## Inventory

`colcon list` is five ROS 2 packages: `elfin_control`, `elfin_description`, `elfin_moveit_config`, `elfin_mvp_bringup`, `luggage_msgs`. Other `luggage_*` packages remain `COLCON_IGNORE`.

| Kind | Count | Notes |
|---|---:|---|
| msg | 5 | same as ROS 1 |
| srv | 20 | 26 ROS 1 services minus 6 converted to actions |
| action | 6 | new |

Intra-package types in `.msg`/`.srv`/`.action` files use the unqualified name (`DetectedLuggage`, `SlotSpec`, `MotionSegment`). Humble `rosidl_adapter` rejects `pkg/msg/Type` inside interface files; external types stay `geometry_msgs/Pose` and `std_msgs/Header`. Python/C++ still import as `luggage_msgs.msg.DetectedLuggage` / `luggage_msgs/msg/detected_luggage.hpp`.

## Messages (kept)

| ROS 1 / ROS 2 | Change |
|---|---|
| `DetectedLuggage` | none |
| `SlotSpec` | none |
| `ContainerOpeningEstimate` | none (`float64[3] inner_size` kept) |
| `MotionSegment` | none |
| `LoadTaskStatus` | none |

## Services kept (20)

Short query, reset, or atomic state change. Not converted.

| Service | Role | Notes |
|---|---|---|
| `GetCurrentBox` | query current pickup box | |
| `ClearCurrentBox` | delete current pickup box | |
| `SyncPickupBox` | push box into planning scene | |
| `FinalizeCurrentBox` | commit current box | |
| `ResetCargoMap` | clear cargo voxels | |
| `GetCargoMapStats` | compact map stats | |
| `GetNextSlot` | legacy slot wrapper | |
| `VacuumCommand` | suction on/off | idempotent if `enable` repeats; not an Action |
| `AddPlacedBox` | add collision object | |
| `RemovePlacedBox` | remove collision object | |
| `SyncStaticScene` | load static scene | |
| `InspectContainer` | interior inspect | watch in Phase 5 if P95 exceeds service timeout |
| `DetectLuggage` | detect pickup box | same watch |
| `VerifyPlacedBox` | post-place measure | |
| `IntegrateCargoView` | fuse settled depth | same watch |
| `EvaluateCargoViews` | batch view scoring | same watch |
| `OrchestratorStep` | GUI step gate | packing cycle stays orchestrator-owned |
| `SpawnNextBox` | sim spawn/replace box | **sim-backend only**, not a hardware plugin; ROS 1 clears then spawns, so repeat is not idempotent |
| `ComputePlacement` | packing solver | short service; not the load-cycle Action |
| `BuildMotionSequence` | geometric waypoints | see deviations |

## Actions (6)

Goal/result fields match the ROS 1 services. Feedback, cancel, timeout, and idempotency are server policy for Phase 6/9; they are not extra message fields except feedback.

### `PlanMotion`

ROS 1 `handle_plan_motion` plans **and executes**. The name is kept.

| | |
|---|---|
| Goal | `MotionSegment segment` |
| Result | `success`, `message` |
| Feedback | `stage` (`planning` / `executing` / `settling`), `segment_name`, `fraction` |
| Cancel | cooperative stop of `FollowJointTrajectory`; **do not** auto vacuum-off |
| Timeout | 45 s (ROS 1 `_execute_timeout`) |
| Idempotent | no; a new goal is a new attempt (ROS 2 goal UUID distinguishes tries) |

### `GoToRobotPose`

| | |
|---|---|
| Goal | `pose_name` |
| Result | `success`, `already_there`, `message` |
| Feedback | `stage`, `remaining_error` |
| Cancel | stop trajectory |
| Timeout | 45 s |
| Idempotent | yes if already inside tolerance (`already_there=true`, no motion) |

### `GoToJointValues`

Same cancel / timeout / idempotency as `GoToRobotPose`. Goal: `values`, `joint_names`.

### `AimCameraAtContainer`

| | |
|---|---|
| Goal | `container_frame`, `link6_xy_tolerance`, `link6_z_tolerance`, `execute` |
| Result | `success`, `already_there`, `joint_values`, `message` |
| Feedback | `stage` (`ik` / `planning` / `executing`) |
| Cancel | stop trajectory only when `execute=true`; IK-only may drop the compute |
| Timeout | 45 s if `execute`; 5 s if IK-only |
| Idempotent | `already_there` matches ROS 1 |

### `ValidateMotionSequence`

Plans a full sequence without executing.

| | |
|---|---|
| Goal | `MotionSegment[] segments` |
| Result | `success`, `message`, `rejection_reason`, `minimum_cartesian_fraction`, `minimum_joint_margin`, `manipulability`, `retreat_feasible` |
| Feedback | `segment_index`, `segment_name`, `stage` |
| Cancel | stop remaining segment plans; result `success=false`, `rejection_reason=cancelled` |
| Timeout | 60 s |
| Idempotent | read-only; repeats allowed; results may change with the scene |

### `PlanNextCargoView`

| | |
|---|---|
| Goal | `mode`, `current_joint_values`, `joint_names`, `views_used`, `reset_session`, `preview_only` |
| Result | same fields as ROS 1 `PlanNextCargoView.srv` |
| Feedback | `stage`, `candidate_id` |
| Cancel | `preview_only` must not mutate session; otherwise do not commit the candidate |
| Timeout | 10 s |
| Idempotent | `preview_only=true` is read-only; `reset_session=true` is explicitly not |

## Deviations from the parent TODO

1. **Packing main flow is orchestrator-only.** No `ExecuteLoadTask` Action. The ROS 1 orchestrator already has 20 states; the manual GUI drives it with `OrchestratorStep`. A single cycle Action would hide step mode. Phase 9 owns the orchestrator. `ComputePlacement` stays a short solver service.

2. **`BuildMotionSequence` stays a Service.** The parent TODO listed it with the motion Actions, but `waypoint_generator_node` only runs geometric `build_sequence()` (milliseconds, no MoveIt). Cancel and timeout do not apply.

## Typesupport

```text
colcon list
elfin_control       src/elfin_control       (ros.ament_cmake)
elfin_description   src/elfin_description   (ros.ament_cmake)
elfin_moveit_config src/elfin_moveit_config (ros.ament_cmake)
elfin_mvp_bringup   src/elfin_mvp_bringup   (ros.ament_cmake)
luggage_msgs        src/luggage_msgs        (ros.ament_cmake)

ros2 interface list | grep luggage_msgs
# 5 msg, 20 srv, 6 action (see Inventory)

python3 -c "from luggage_msgs.msg import DetectedLuggage, MotionSegment; from luggage_msgs.srv import ComputePlacement, OrchestratorStep; from luggage_msgs.action import PlanMotion, PlanNextCargoView"
# python typesupport ok

# Humble C++ include root is .../include/luggage_msgs/
test -f install/luggage_msgs/include/luggage_msgs/luggage_msgs/action/plan_motion.hpp
# plan_motion.hpp ok
```

C++ include: `#include "luggage_msgs/action/plan_motion.hpp"` with include directory `install/luggage_msgs/include/luggage_msgs`.
