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

## ROS Noetic simulation (Docker)

Noetic Docker, `elfin_noetic_ws`, and the Noetic TCP executor live on the
[`main`](https://github.com/adamliaoyifan/RoboticArm/tree/main) branch:

```bash
git fetch origin
git checkout main
./docker/noetic/run.sh build
```

See `docker/noetic/HARDWARE.md` on `main` for EtherCAT and Huayan TCP.

---

## ROS 2 Humble workspace

The ROS 2 Humble migration of the luggage loading stack lives at
[`elfin_humble_ws/`](elfin_humble_ws/). It is a colcon workspace (MoveIt 2,
Gazebo / `ros_gz`, luggage perception/planning/packing packages).

YOLO / CLIP weights (`*.pt`) and raw point-cloud scans (`*.las`) are not in
git. See [`elfin_humble_ws/docs/README.md`](elfin_humble_ws/docs/README.md)
for migration plans and status.

Draft image: [`elfin_humble_ws/Dockerfile.humble`](elfin_humble_ws/Dockerfile.humble).

---

## ROS2 trajectory executor (Docker)

Real-robot TCP runtime lives in [`deployment_ws/`](deployment_ws/) (formerly `ros2_ws/`):

- Humble / Jazzy: [`deployment_ws/src/elfin_trajectory_executor`](deployment_ws/src/elfin_trajectory_executor)
- Noetic: [`deployment_ws/noetic/elfin_cps_executor` on `main`](https://github.com/adamliaoyifan/RoboticArm/tree/main/deployment_ws/noetic/elfin_cps_executor)

Huayan SDK sources used by those nodes: [`SDK_sample/`](SDK_sample/), [`third_party/`](third_party/).

See [`deployment_ws/README.md`](deployment_ws/README.md) and
[`deployment_ws/src/elfin_trajectory_executor/Dockerfile`](deployment_ws/src/elfin_trajectory_executor/Dockerfile).
It is separate from the Noetic simulation stack and from `elfin_humble_ws`.
