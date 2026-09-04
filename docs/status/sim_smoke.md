# Simulation backend smoke

Date: 2026-08-14  
Host GPU: NVIDIA GeForce RTX 5090, driver 580.105.08  
OpenGL: `NVIDIA GeForce RTX 5090/PCIe/SSE2` (not `llvmpipe`)  
Related: [gpu_runtime.md](gpu_runtime.md), [sim_backends_ros2.md](../plans/sim_backends_ros2.md)

## Decision

| Backend | This batch | Role |
|---|---|---|
| Gazebo Fortress 6.18.0 | Installed, GUI smoke passed | Product closed-loop sim (Phase 7A) |
| MuJoCo `mujoco_ros2_control` 0.0.3 | Installed, official demo controllers active | Control / contact contrast (Phase 7B) |
| Isaac Sim 5.1 (`~/isaacsim`) | Present on host; Kit **not** started | Photoreal RGB-D / later RL (Phase 7C) |

Do not install `ros-humble-gazebo-ros` (Classic) next to Fortress. Do not start Isaac Kit in this batch.

## Fortress

Humble pairs with Fortress (`libignition-gazebo6` 6.18.0). On this host the CLI is **`ign gazebo`**, not `gz sim` (`gz` is Garden+ / Harmonic tools).

```text
ign gazebo --versions          # 6.18.0
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix gz_ros2_control
```

Packages: `ros-humble-ros-gz-sim` 0.244.25, `ros-humble-gz-ros2-control` 0.7.20.

GUI smoke (killed after 12 s, exit 124 = timeout):

```bash
timeout 12 ign gazebo -r /usr/share/ignition/ignition-gazebo6/worlds/shapes.sdf -v 3
```

Observed: `Ignition Gazebo GUI v6.18.0`, `Ignition Gazebo Server v6.18.0`, plugin `ignition-rendering-ogre2`. Qt/EGL printed `libEGL warning: egl: failed to create dri2 screen` once; the scene still loaded Ogre2. `glxinfo` renderer remains NVIDIA, not llvmpipe.

S20 + container SDF is **not** migrated in this batch.

## MuJoCo

```text
ros2 pkg prefix mujoco_ros2_control
ros2 pkg list | grep mujoco
# mujoco_ros2_control, mujoco_ros2_control_demos, mujoco_ros2_control_msgs,
# mujoco_ros2_control_plugins, mujoco_vendor
```

Official demo (headless physics; window rendering is the GPU hard gate, not required for this smoke):

```bash
ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py headless:=true
```

```text
position_controller      ForwardCommandController   active
joint_state_broadcaster  JointStateBroadcaster      active
gripper_controller       ForwardCommandController   active
```

Log line `Failed to find actuator for joint : gripper_right_finger_joint` is expected (passive finger in the demo MJCF). Demo uses this package's `ros2_control_node`, not `controller_manager`'s.

S20 MJCF is **not** authored in this batch.

## Isaac Sim / Lab

The host already has Isaac Sim **5.1.0-rc.19** at `/home/adamliao/isaacsim`, including `exts/isaacsim.ros2.bridge` with bundled Humble Python 3.11 libs. This batch did **not** start Kit, did not import USD, and did not run a ROS 2 bridge smoke.

Host `/usr/bin/python3` (3.10) cannot `import isaacsim`; that is expected. Start Sim from a terminal that has **not** sourced `/opt/ros/humble`. Phase 7C: two-process split, then one conservative joint goal. Isaac Lab was not exercised.
