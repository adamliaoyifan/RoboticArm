# Real-robot runtime nodes (`deployment_ws`)

TCP execution for the real Elfin is kept here, separate from the Noetic
simulation / luggage stack in `elfin_noetic_ws` and from the closed-loop
stack in `elfin_humble_ws`.

| Path | ROS | Build | Role |
|---|---|---|---|
| `src/elfin_trajectory_executor` | Humble or Jazzy | `colcon` | `FollowJointTrajectory` via Huayan CPS |
| `noetic/elfin_cps_executor` | Noetic | catkin in Docker | Same TCP path, plus `/execute_trajectory` |

The Noetic package is not under `src/` so `colcon build` does not touch it.
The Noetic container bind-mounts it to `/catkin_ws/src/elfin_cps_executor`.

This host currently has **Jazzy only** (`/opt/ros/jazzy`). Build and run
the executor against Jazzy. Do not source Humble (or a Humble colcon
overlay) in the same shell.

Site worksheet: copy [`config/site_vars.yaml.example`](config/site_vars.yaml.example)
to `config/site_vars.yaml` and fill measured IPs / geometry.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7
# FollowJointTrajectory lives here; required on a Jazzy-only host:
#   sudo apt install ros-jazzy-control-msgs
cd deployment_ws
python3 scripts/check_site.py --init
python3 scripts/check_site.py          # Gate 0: NIC + ping
eval "$(python3 scripts/check_site.py --export)"
python3 scripts/check_gate1.py         # CPS import + TCP port
bash scripts/run_gate1_sim_handshake.sh  # 2° FJT + READY_FOR_NEXT, no Gazebo

colcon build --packages-select elfin_trajectory_executor
source install/setup.bash

# Real arm: no Gazebo. Only after ping succeeds and a person is on e-stop.
ros2 launch elfin_trajectory_executor jazzy_real.launch.py

# Other terminal, same ROS_DOMAIN_ID. Blocks until the arm finishes.
ros2 run elfin_trajectory_executor send_joint_trajectory --delta-deg 2 --and-back
```

Hardware TF (after Gate 1 `/joint_states` is live):

```bash
ros2 launch luggage_description scene_hardware.launch.py \
  scene_tf_config:=/abs/path/to/measured/scene_tf.yaml
```

Mid-360:

```bash
./scripts/setup_livox_driver.sh
source livox_ws/env.sh   # overlay + Livox-SDK2 rpath
python3 scripts/check_gate2.py
ros2 launch luggage_description mid360.launch.py \
  user_config_path:=/abs/path/to/MID360s_config.json
```

D435 + weights (Gate 3):

```bash
bash ../elfin_humble_ws/src/luggage_perception/scripts/download_yolo_world.sh
bash ../elfin_humble_ws/src/luggage_perception/scripts/setup_clip_vendor.sh
python3 scripts/check_gate3.py
ros2 launch luggage_perception realsense_d435.launch.py
ros2 run luggage_perception dump_camera_frames.py --out ~/robotarm_site_frames
```

`real.launch.py` is the same graph as `jazzy_real.launch.py`.

## Jazzy vs Humble

They conflict if mixed in one process or one DDS graph:

- This machine has no `/opt/ros/humble`. `elfin_humble_ws` was written for
  Humble (Python 3.10, Gazebo Fortress). Jazzy is Python 3.12 and Harmonic.
- Do not `source /opt/ros/humble` then overlay Jazzy, or the reverse.
- Do not run `luggage_gazebo/sim_world.launch.py` on Jazzy expecting the
  Humble Fortress launch to work.
- Give Humble Docker / Noetic and this Jazzy executor **different
  `ROS_DOMAIN_ID`s** if both are up on the same LAN.
- The executor package itself is distro-light (`rclpy` + `control_msgs`)
  and is the Jazzy real-robot target. Full luggage closed-loop on Jazzy
  lives on `origin/ros2_jazzy` (`elfin_jazzy_ws`), not this launch.

## Execution handshake

The planner/test client **must wait for the FollowJointTrajectory result**.
That result is the only "you may start the next step" signal.

```text
client                         executor
  |-- send_goal -------------->|
  |<-- accepted ---------------|
  |<-- feedback (joints) ------|   (motion in progress)
  |<-- result SUCCEEDED -------|   ready_for_next
  |-- next goal -------------->|
```

| Channel | Type | Use |
|---|---|---|
| `/elfin_arm_controller/follow_joint_trajectory` | action | **Sequencing.** Next step only if status is SUCCEEDED and `error_code` is SUCCESSFUL. |
| `/trajectory_executor/events` | `std_msgs/String` JSON, transient-local | Observers. `ready_for_next` is true only on `"event":"succeeded"`. |
| `/trajectory_executor/status` | `idle` / `executing` / `error` | Debug. Do **not** sequence on `idle`. |
| `/joint_states` | 100 Hz | FK / RViz. Not a completion signal. |

`send_joint_trajectory` prints `READY_FOR_NEXT` only after a successful
result. Cancel, abort, timeout, or reject must not continue the pipeline.

Do not start Gazebo, `zero_joint_state_publisher`, or EtherCAT together
with this executor: they publish competing `/joint_states`.

## Site blockers on this host (2026-09-02)

`scripts/check_site.py` last report: `enp0s31f6` is DOWN, ICMP to
`192.168.0.10` fails, no D435 on USB, no NVIDIA GPU. TCP `192.168.0.10:10003`
answered once — treat that as untrusted until wired ping works. Do not send
the hardware 2° FJT until ping is green and someone is on the e-stop.

Livox needs a second NIC on `192.168.1.0/24` and the sensor serial suffix in
`MID360s_config.json`. YOLO/CLIP weights are on disk; semantic overlay uses
`device: cpu` until `nvidia-smi` works.
