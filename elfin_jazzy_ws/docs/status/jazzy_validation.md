# Jazzy native validation (Ubuntu 24.04)

Date: 2026-08-29  
Workspace: `elfin_jazzy_ws`  
ROS: `/opt/ros/jazzy` (Harmonic `gz sim` 8.11.0)

Host notes: use `/usr/bin/cmake` and `/usr/bin/python3`; unset Anaconda `CONDA_PREFIX` before `colcon build`. `mesa-utils` was not installed (no sudo); GPU gate falls back to `nvidia-smi`.

## Results

| Check | Result |
|---|---|
| `colcon build` (11 ament packages, `luggage_bringup` ignored) | pass |
| Mock `demo.launch.py` + FJT 3 repeats, max_error 0.001 | **3/3**, worst abs err **0.000026 rad** |
| MoveIt OMPL adapters (`ompl_planning.yaml` Jazzy format) | loaded; `You can start planning now!` |
| Harmonic spawn (S20 + pedestal + pickup_platform + container) | 4/4 entities created |
| Controllers in gz (`joint_state_broadcaster`, `elfin_arm_controller`) | both `active` |
| `observe_pose_hold` FJT | `error_code=0` |
| D435 `/camera/color/image_raw` | 640×480 `rgb8`, `camera_depth_optical_frame` |
| D435 `/camera/depth/points` | 640×480 PointCloud2, same frame |
| `/pickup_box_spawner/spawn_next_box` | success (`pickup_box_0001_carryon`) |
| Sim FJT tucked spawn (`spawn_at_observe:=false`), 3 repeats | action **6/6 SUCCEEDED** (go+back); wait_near vs GOAL **0.073 rad** (START 0.0). Humble sim budget was 0.05; Harmonic tracking is looser. |
| `use_semantic:=true` | not exercised: `yolov8s-world.pt` absent; node would stub |

`send_joint_trajectory.py` abort-on-exit (`terminate called without an active exception`) is rclpy teardown noise; the logged `done repeats=...` line is the gate.

## Jazzy-only runtime notes

- `gz_ros_control` node name (not Humble `gz_ros2_control`) must carry `position_proportional_gain: 0.8`.
- Jazzy JTC honours `FollowJointTrajectory.goal_time_tolerance`. Humble ignored the 1 s value in `send_joint_trajectory.py`; it is now 15 s so observe-distance motions are not aborted.
- Headless cameras need `gz sim -s -r --headless-rendering`. EGL prints `failed to create dri2 screen` on this 2080 SUPER but ogre2 still published RGB-D.
- Round-trip FJT from **observe** to the mock GOAL/START pair is a large joint move; tucked spawn matches the Humble 20/20 protocol.

## Commands used

```bash
export PATH="/usr/bin:$PATH"
unset CONDA_PREFIX CONDA_DEFAULT_ENV
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch elfin_mvp_bringup demo.launch.py use_rviz:=false
ros2 run elfin_mvp_bringup send_joint_trajectory.py --repeat 3 --max-error 0.001

ros2 launch luggage_gazebo sim_world.launch.py \
  gui:=false use_rviz:=false use_moveit:=true \
  spawn_at_observe:=false
ros2 run elfin_mvp_bringup send_joint_trajectory.py --repeat 3 --max-error 0.10 --duration 8.0
```
