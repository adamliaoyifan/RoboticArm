# Airport Luggage Loading Simulation — Phase 0 Skeleton

Modular ROS Noetic stack for Elfin S20 airport luggage loading into a side-open container.

## Architecture

```
luggage_bringup (orchestrator)
    ├── luggage_perception   → detect_luggage
    ├── luggage_packing      → get_next_slot
    └── luggage_planning     → sync_static_scene, build_motion_sequence,
                               go_to_robot_pose, plan_motion, vacuum_command, add_placed_box

luggage_msgs                 → shared msg/srv contracts
luggage_description          → YAML + URDF stubs
luggage_gazebo               → Gazebo world (wraps elfin_s20_empty_world)
```

Modules communicate **only via ROS services** defined in `luggage_msgs`. The orchestrator contains the state machine; it does not implement geometry or MoveIt calls.

## Packages

| Package | Role |
|---------|------|
| `luggage_msgs` | Messages and services |
| `luggage_description` | `vacuum_gripper.yaml.example`, `robot_poses.yaml.example`, container configs |
| `luggage_gazebo` | Simulation world launch |
| `luggage_perception` | `luggage_detector_node` |
| `luggage_packing` | `bin_packer_node` |
| `luggage_planning` | scene, waypoint, motion, vacuum nodes |
| `luggage_bringup` | `orchestrator_node` + launches |

## Build

Inside the Noetic Docker container (or native Noetic):

```bash
cd /catkin_ws   # or ~/RobotArm/elfin_noetic_ws mounted as /catkin_ws/src/...
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run skeleton (no Gazebo)

Runs all stub nodes and one orchestrator cycle. `skip_reset` defaults to `true` so MoveIt is not required:

```bash
roslaunch luggage_bringup sim_skeleton.launch
```

Expected log flow: `SyncScene → Detect → PlanPick → ExecPick → PlanPlace → ExecPlace → UpdateScene → Idle`.

## Observe reset (first real motion)

Gazebo spawns the arm flat (joint zeros). Before detection, move to the **observe** pose so the wrist camera looks down at ground luggage.

Pose joints are configured in `luggage_description/config/robot_poses.yaml.example` (copy to `robot_poses.yaml` to tune).

### Standalone reset (Gazebo + MoveIt + auto move)

```bash
roslaunch luggage_bringup reset_observe.launch
```

This starts `sim_world`, MoveIt with the camera URDF (`moveit_with_camera.launch`), `motion_planner`, and a one-shot client that calls `/motion_planner/go_to_robot_pose`.

Verify camera view after reset:

```bash
roslaunch luggage_bringup camera_view.launch
```

Call again while the stack is running — the service should return `already_there=true` without moving.

### Tune observe pose (recommended: MoveIt RViz)

MoveIt respects joint limits and reachability — drag the end-effector in RViz, execute, then read joint angles.

```bash
roslaunch luggage_bringup pose_tune_moveit.launch
```

This starts Gazebo, MoveIt, the RGB tune GUI, and **RViz with Motion Planning**.

**Workflow:**

1. In **RViz** → **MotionPlanning** panel (left):
   - Planning Group: `elfin_arm`
   - **Query Goal State** enabled (orange interactive marker on the arm)
   - Drag the marker to aim the camera at the luggage / ground
   - Click **Plan**, then **Execute** (MoveIt checks collisions and joint limits)
2. In the **Observe Pose Tune** window (RGB preview):
   - Click **Read Joints (after MoveIt Execute)** — loads `/joint_states` into sliders
   - Click **Copy** or **Save** → `~/observe_pose_tuned.yaml`
3. Send the 6 joint values (rad) back so `robot_poses.yaml` can be updated.

CLI alternative (no GUI sliders):

```bash
rosrun luggage_bringup export_observe_joints.py
```

### Tune observe pose (sliders only)

For quick experiments without MoveIt IK (may reach invalid poses):

```bash
roslaunch luggage_bringup pose_tune.launch
```

- Enable **Live preview in Gazebo** to move the arm while dragging sliders
- **Execute Move (MoveIt)** for a collision-checked move after tuning

**Tuning tips:**

- Adjust joint2 and joint4 first (shoulder/elbow — largest effect on camera pitch)
- Then fine-tune joint3 and joint5
- joint1 rotates the view horizontally; joint6 has minor effect
- Goal: ground level in RGB, luggage visible and not heavily tilted

After you find a good pose, send the 6 radian values back so `robot_poses.yaml` can be updated.

### Tune camera mount on elfin_link6 (side mount)

The RealSense D435 is **side-mounted on `elfin_link6`**. **Body frame** (`camera_link`) follows Intel `realsense2_description`: **+X = lens/forward**, **+Y = 90 mm long axis / baseline**, **+Z = height**; optical **+Z = camera +X**.

**Observe acceptance** (world frame, rigid mount): optical axis ≈ **`[0,0,-1]`**; long edge **+Y** parallel to ground (horizontal).

**Production** URDF: fixed `camera_mount_joint` from [`config/camera_mount_origin.xacro`](elfin_noetic_ws/src/luggage_description/config/camera_mount_origin.xacro). **Tune mode**: six actuated mount joints.

```bash
roslaunch luggage_bringup camera_mount_tune.launch
rosrun luggage_bringup verify_camera_mount_config.py   # FK check at observe joints
```

- **`camera_mount_tune.launch`** → tune GUI + observe spawn/init (调 mount 用)
- **`sim_full.launch`** → 无 GUI，生产仿真 + observe + 固定 mount
- 修改 `luggage_bringup/scripts/*.py` 后需 **`catkin_make && source devel/setup.bash`**

- **`camera_mount_tune_init`** snaps arm to **observe** at startup (no MoveIt wait)
- Drag **tx/ty/tz** and **roll/pitch/yaw** sliders — RGB preview updates live
- **Save** writes **tune_joints** (sliders) plus **fixed** URDF mount to `camera_mount_origin.xacro` and mount YAML — **restart `sim_full`** to apply production camera pose
- Verify tune vs production mount: `rosrun luggage_bringup verify_camera_mount_config.py`
- Resync from YAML without GUI: `rosrun luggage_bringup sync_camera_mount_config.py`
- **Restart** production sim (`sim_world.launch` / `reset_observe.launch`) to pick up saved URDF

Optional MoveIt (collision-aware observe move): `roslaunch luggage_bringup camera_mount_tune.launch use_moveit:=true`

Skip observe snap: `move_to_observe_on_start:=false`

Typical order: tune **arm observe joints** first (`pose_tune_moveit.launch`), then tune **camera mount** if RGB aim is still off.

### Orchestrator with reset

Full sim without orchestrator auto-start. **`sim_full.launch` spawns the arm at the observe pose** and uses the **fixed** mount from `camera_mount_origin.xacro` (derived from tune `tune_joints` in mount YAML). After changing mount in tune GUI, click **Save** and restart `sim_full`.

```bash
roslaunch luggage_bringup sim_full.launch
```

MoveIt (camera URDF aligned with Gazebo):

```bash
roslaunch luggage_bringup moveit_with_camera.launch
```

This loads `S20_with_camera.srdf` so MoveIt matches the RealSense-mounted URDF used in Gazebo.

Start orchestrator with reset enabled:

```bash
rosrun luggage_bringup orchestrator_node.py _skip_reset:=false
```

Expected log flow: `ResetObserve → SyncScene → Detect → ...`.

### Container aim + interior inspection

Aim `camera_depth_optical_frame` at the container opening (MoveIt path constraints on `elfin_link6` XY), then inspect free space inside the container:

```bash
roslaunch luggage_bringup inspect_container.launch
```

With orchestrator cycle (requires MoveIt + Gazebo):

```bash
roslaunch luggage_bringup inspect_container.launch start_orchestrator:=true skip_reset:=true
```

Services:

- `/motion_planner/aim_camera_at_container` — IK + constrained plan to face opening
- `/container_inspector/inspect_container` — Gazebo GT occupancy + free slots (mode `gazebo_gt` or `depth`)

Orchestrator flow with container aim enabled (`skip_container_aim:=false`):

`ResetObserve → SyncScene → AimContainer → InspectContainer → Detect → ...`

Static TF: `elfin_base_link → container_link → container_opening_frame` from [`container.yaml.example`](elfin_noetic_ws/src/luggage_description/config/container.yaml.example).

## Run with Gazebo (legacy multi-terminal)

Terminal 1:

```bash
roslaunch luggage_bringup sim_full.launch
```

Terminal 2 (MoveIt with camera URDF):

```bash
roslaunch luggage_bringup moveit_with_camera.launch
```

Terminal 3 (orchestrator; enable reset when MoveIt is running):

```bash
rosrun luggage_bringup orchestrator_node.py _skip_reset:=false
```

## Service map

| Service | Node | Behaviour |
|---------|------|-----------|
| `/luggage_detector/detect_luggage` | perception | Returns one fake suitcase |
| `/bin_packer/get_next_slot` | packing | Returns fixed slot |
| `/scene_manager/sync_static_scene` | planning | Log TODO |
| `/scene_manager/add_placed_box` | planning | Log TODO |
| `/waypoint_generator/build_motion_sequence` | planning | Named empty segments |
| `/motion_planner/go_to_robot_pose` | planning | MoveIt joint target to named pose |
| `/motion_planner/go_to_joint_values` | planning | MoveIt joint target (arbitrary values, for tune GUI) |
| `/motion_planner/plan_motion` | planning | Log TODO, returns success |
| `/vacuum_simulator/vacuum_command` | planning | Log TODO |

## Config files

Copy examples and edit:

```bash
cp $(rospack find luggage_description)/config/vacuum_gripper.yaml.example \
   $(rospack find luggage_description)/config/vacuum_gripper.yaml

cp $(rospack find luggage_description)/config/robot_poses.yaml.example \
   $(rospack find luggage_description)/config/robot_poses.yaml
```

## Orchestrator states

`LoadTaskStatus.state` may be: `Idle`, `ResetObserve`, `SyncScene`, `Detect`, `PlanPick`, `ExecPick`, `PlanPlace`, `ExecPlace`, `UpdateScene`.

## Phase 1+ TODO

- `fill-gazebo` — side-open container SDF, suitcase models
- `fill-ee` — xacro reads `vacuum_gripper.yaml`, mount on `elfin_end_link`
- `fill-scene` — MoveIt collision objects
- `fill-plan` — MoveIt / Pilz motion execution for pick/place segments
- `fill-pack` — grid/layer bin packer
- `fill-perception` — Gazebo ground truth or depth detection
