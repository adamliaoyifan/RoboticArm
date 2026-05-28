# Elfin-S20 Real Hardware via Docker

Hardware control uses EtherCAT (SOEM) and requires setup **outside** the container.

## 1. PREEMPT_RT kernel (host)

Install and boot a PREEMPT_RT kernel on Ubuntu 22.04 before running `elfin_ros_control.launch`.
Follow the vendor tutorial linked from the
[elfin_s_robot README](https://github.com/huayan-robotics/elfin_s_robot/blob/noetic/README.md).

PREEMPT_RT often conflicts with proprietary NVIDIA drivers. Use integrated graphics or Nouveau
for the desktop if needed.

Verify after reboot:

```bash
uname -a | grep PREEMPT
```

## 2. Vendor driver config

Copy the vendor-provided `elfin_drivers.yaml` to:

```
~/elfin_noetic_ws/src/elfin_s_robot/elfin_robot_bringup/config/elfin_drivers.yaml
```

Set the Ethernet interface name to match your NIC (not necessarily `eth0`):

```yaml
elfin_ethernet_name: enp3s0   # example — use ip link show
```

A placeholder template is at `elfin_drivers.yaml.example` in this directory.

## 3. Start the hardware container

```bash
./docker/noetic/run.sh hw
```

Inside the container (four terminals via `run.sh exec` or `docker exec`):

```bash
source /catkin_ws/devel/setup.bash

# Terminal 1
roslaunch elfin_robot_bringup elfin_s20_bringup.launch

# Terminal 2 — realtime priority shell (vendor recommendation)
chrt 10 bash
roslaunch elfin_robot_bringup elfin_ros_control.launch

# Terminal 3
roslaunch elfin_s20_moveit_config moveit_planning_execution.launch

# Terminal 4
roslaunch elfin_basic_api elfin_basic_api.launch
```

Use the Elfin Control Panel: **Clear Fault** (if needed) → **Servo On** before motion;
**Servo Off** before shutdown.

## Risk note

EtherCAT over Docker is not vendor-tested. If you see slave dropouts or timing faults,
use a native Ubuntu 20.04 install for production.
