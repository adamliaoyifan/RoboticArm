# Elfin S20 – EOF Camera Mount

Interactive 3D visualization of the Elfin S20 robot arm and a 3D-printable
camera mount bracket for the End-of-Flange (EOF).

## Files

| File | Description |
|------|-------------|
| `visualize_robot.py` | Forward-kinematics visualizer (Plotly → interactive HTML) |
| `camera_mount.scad` | OpenSCAD parametric camera mount for the EOF |
| `elfin_s20_visualization.html` | Pre-generated interactive 3D view |

---

## Visualization

```bash
pip install numpy plotly
python3 visualize_robot.py          # home position (default)
python3 visualize_robot.py reach    # extended pose
python3 visualize_robot.py folded   # compact pose
```

Opens / saves `elfin_s20_visualization.html` — open in any browser for a
fully interactive 3D view with joint info and camera mount.

---

## Camera Mount (`camera_mount.scad`)

### Design overview

```
                       ┌─────────────────────┐
                       │   Camera (D435)      │
                       │  ◉    ◉    ◉  lens  │
                       └────────┬────────────┘
                    ╔═══════════╧══════════╗
                    ║     Camera Cradle     ║
                    ╚═══════════╤══════════╝
                                │  Bracket arm
                    ╔═══════════╧══════════╗
                    ║     Flange Plate      ║
                    ║  ○       ○       ○   ║  4× M6 on Ø63 PCD
                    ╚══════════════════════╝
                          [EOF flange]
```

Three parts (assembled as one print or printed separately):

1. **Flange Plate** – bolts directly to the S20 EOF flange  
   - 4× M6 bolt holes on Ø63 mm PCD  
   - Central Ø16 mm pilot bore  
   - Anti-rotation notch  

2. **Bracket Arm** – 55 mm standoff with cable routing slot  

3. **Camera Cradle** – friction-fit + 2× M2 retention screws  
   - Fits Intel RealSense D435 (90 × 25 × 25 mm)  
   - Easily adapted to other cameras via `CAM_W/H/D` parameters  

### Print settings

| Setting | Value |
|---------|-------|
| Material | PETG or ABS (not PLA) |
| Layer height | 0.2 mm |
| Infill | 40 % |
| Supports | Yes (cradle overhang) |

### Key parameters (top of `.scad`)

```scad
FLANGE_PCD   = 63;   // mm  bolt-circle diameter – adjust to your exact flange
ARM_LENGTH   = 55;   // mm  standoff length
CAM_W        = 90;   // mm  camera body width (RealSense D435)
CAM_H        = 25;   // mm  camera body height
CAM_D        = 25;   // mm  camera body depth
```

### Render / export

```bash
openscad camera_mount.scad               # interactive view
openscad -o camera_mount.stl camera_mount.scad   # export STL
```

---

## Robot specs (Elfin S20)

| Property | Value |
|----------|-------|
| DOF | 6 |
| Payload | 20 kg |
| Reach | ~1400 mm |
| Joint 1 range | ±360° |
| Joint 2 range | −190° / +10° |
| Joint 3 range | ±168° |
| Joints 4-6 range | ±360° |
| Manufacturer | HuaYan Robotics |
| URDF source | [GitHub](https://github.com/huayan-robotics/elfin_s_robot/blob/main/elfin_description/urdf/S20.urdf.xacro) |

---

## ROS Noetic simulation (Docker on Ubuntu 22.04)

Run the official [elfin_s_robot noetic stack](https://github.com/huayan-robotics/elfin_s_robot/tree/noetic) in an Ubuntu 20.04 container — Gazebo, MoveIt, and RViz — without downgrading the host OS.

**Prerequisites:** Docker, X11 (`xhost +local:docker`).

```bash
# Workspace lives in this repo (or set ELFIN_WS)
# git clone is already under elfin_noetic_ws/src/elfin_s_robot

# Build image (~15–30 min first time)
./docker/noetic/run.sh build

# Start container
./docker/noetic/run.sh start

# Option A — two-step (recommended for trajectory execution)
./docker/noetic/run.sh gazebo    # wait until Gazebo is fully up
./docker/noetic/run.sh moveit
./docker/noetic/run.sh api

# Option B — single launch (S20)
./docker/noetic/run.sh sim-all

./docker/noetic/run.sh stop
```

**Important:** `moveit_planning_execution.launch` expects `/robot_description` from Gazebo unless you pass `load_robot_description:=true`. The `run.sh moveit` command auto-loads the URDF when Gazebo is not running, but you still need Gazebo for simulated motion execution.

If RViz/Gazebo show a black window or NVIDIA libGL errors, try:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./docker/noetic/run.sh moveit
# or enable GPU passthrough:
USE_GPU=1 ./docker/noetic/run.sh start
```

Inside the container, source the workspace: `source /catkin_ws/devel/setup.bash`

| Command | Purpose |
|---------|---------|
| `./docker/noetic/run.sh sim` | Interactive shell (simulation) |
| `./docker/noetic/run.sh hw` | Hardware profile (privileged + realtime caps) |
| `./docker/noetic/run.sh gazebo` | Gazebo + S20 model |
| `./docker/noetic/run.sh moveit` | MoveIt + RViz |
| `./docker/noetic/run.sh sim-all` | Gazebo + MoveIt + RViz (S20, one launch) |
| `./docker/noetic/run.sh api` | Elfin Control Panel |

Set `ELFIN_MODEL=s05|s10|s30` to change robot variant (default `s20`). See [`docker/noetic/HARDWARE.md`](docker/noetic/HARDWARE.md) for real EtherCAT hardware.

### Airport luggage loading (simulation)

Catkin packages under [`elfin_noetic_ws/src/luggage_*`](elfin_noetic_ws/src/). Build inside the Noetic container, then:

```bash
source /catkin_ws/devel/setup.bash
roslaunch luggage_bringup inspect_container.launch   # Gazebo + MoveIt + container aim/inspect
roslaunch luggage_bringup active_loading.launch      # Runtime pickup box + active Cargo loading loop
roslaunch luggage_bringup camera_view.launch         # RViz debug: robot + Cargo + OctoMap (needs active_loading)
# roslaunch luggage_bringup sim_skeleton.launch    # Phase 0 stub only
```

See [`PROGRESS.md`](PROGRESS.md) for the full **development timeline** and milestone summary, [`CHANGELOG.md`](CHANGELOG.md) for release notes, and [`elfin_noetic_ws/src/luggage_bringup/README.md`](elfin_noetic_ws/src/luggage_bringup/README.md) for launch details.

---

## ROS2 trajectory executor (Docker)

The ROS2 Humble executor for deployment on Ubuntu 20.04 hosts lives at
[`ros2_ws/src/elfin_trajectory_executor/Dockerfile`](ros2_ws/src/elfin_trajectory_executor/Dockerfile).
It is separate from the Noetic simulation stack above.
