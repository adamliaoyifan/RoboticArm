# GPU runtime hard gate

Date: 2026-08-14  
Host: native Ubuntu 22.04 + `/opt/ros/humble`  
Result: **passed**

Interactive Gazebo Fortress, MuJoCo Simulate, RViz2, and Isaac Sim/Lab must use the NVIDIA GPU. `llvmpipe` (CPU software rendering) is a hard failure: it steals cores from OMPL/MoveIt and is not a valid sim runtime.

## Measured

```text
GPU:     NVIDIA GeForce RTX 5090
Driver:  580.105.08
VRAM:    32607 MiB
OpenGL:  NVIDIA GeForce RTX 5090/PCIe/SSE2  (vendor NVIDIA Corporation, 4.6.0)
DISPLAY: :1
```

Commands:

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used --format=csv
DISPLAY=${DISPLAY:-:1} glxinfo -B | grep -E 'OpenGL renderer|OpenGL vendor'
```

Pass: renderer contains `NVIDIA` and does not contain `llvmpipe`. Helper: `scripts/check_gpu_renderer.sh`.

Fortress interactive GUI on this host: `ign gazebo` (not `gz sim`). See [sim_smoke.md](sim_smoke.md).

## Rules

- Never run interactive sim or RViz2 under `LIBGL_ALWAYS_SOFTWARE=1`.
- Do not collect planning/control timing while software rendering is active.
- Do not run Isaac Lab batched envs and a Fortress GUI on the same GPU at full load.
- MuJoCo physics stays on CPU; this gate covers the Simulate/OpenGL window only.
- Gazebo Classic (`ros-humble-gazebo-ros`) must not be installed next to Fortress.

## Humble Docker (draft)

Use GPU passthrough. Image sketch: `Dockerfile.humble`. Helper: `docker/run.sh` (`USE_GPU=1` is required; the script refuses to start without it).

```bash
./docker/run.sh build
./docker/run.sh start
```

Equivalent flags:

```bash
docker run --rm -it --gpus all --net=host \
  --env DISPLAY="${DISPLAY:-:1}" \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env QT_X11_NO_MITSHM=1 \
  --env USE_GPU=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  elfin-humble:latest
```

Inside the container, run `scripts/check_gpu_renderer.sh` before `ign gazebo`, MuJoCo, or RViz2. `USE_GPU=1` is the required default for any interactive session.

The existing `Dockerfile` is still Noetic and must not be used for ROS 2.
