# Elfin-S20 Real Hardware via Docker

This repo supports two real-hardware paths from the Noetic Docker environment:

- **EtherCAT / ros_control** via the vendor Noetic stack.
- **TCP / Huayan SDK** via `deployment_ws/noetic/elfin_cps_executor` (Noetic catkin
  package, mounted into the Docker catkin workspace), using either the Python
  `CPS.py` SDK or the C++ `libHR_Pro.so` SDK.

Only run one hardware path at a time.

## TCP / Huayan SDK path

This path talks to the controller on TCP port `10003` and does not use
`elfin_ros_control.launch`.

### SDK locations in the container

The Docker image and `run.sh` runtime mounts expose:

```bash
PYTHONPATH=/opt/huayan
HUAYAN_CPP_SDK=/opt/huayan/cpp_sdk
LD_LIBRARY_PATH=/opt/huayan/cpp_sdk/lib:/opt/huayan/cpp_sdk/HRCPS
```

Verify inside the container:

```bash
python3 -c "from CPS import CPSClient; print('CPS OK')"
ldd /opt/huayan/cpp_sdk/lib/libHR_Pro.so
```

### Rebuild after SDK integration changes

```bash
./docker/noetic/run.sh stop
./docker/noetic/run.sh build
./docker/noetic/run.sh start
./docker/noetic/run.sh exec
```

Inside the container:

```bash
cd /catkin_ws
catkin_make --pkg elfin_cps_executor
source devel/setup.bash
```

### Connect to the real robot

Use the controller IP configured on the teach pendant or controller network.
The default used here is `192.168.0.10`.

```bash
# Host: check basic network reachability.
ping -c1 192.168.0.10

# Python SDK backend.
./docker/noetic/run.sh cps robot_ip:=192.168.0.10

# Or C++ SDK backend.
./docker/noetic/run.sh cpp-cps robot_ip:=192.168.0.10
```

To start MoveIt and one SDK executor together:

```bash
# Python backend.
roslaunch elfin_cps_executor moveit_cps_real.launch robot_ip:=192.168.0.10

# C++ backend.
roslaunch elfin_cps_executor moveit_cps_real.launch robot_ip:=192.168.0.10 use_cpp:=true
```

Expected ROS interfaces:

```bash
rostopic echo /joint_states -n1
rosaction list | grep follow_joint_trajectory
# /elfin_arm_controller/follow_joint_trajectory
```

Do not start Gazebo or `elfin_ros_control.launch` with the SDK executor. They
will conflict on `/joint_states` and the `elfin_arm_controller` trajectory
action.

## EtherCAT / ros_control path

EtherCAT hardware control uses SOEM and requires setup **outside** the container.

### 1. PREEMPT_RT kernel (host)

Install and boot a PREEMPT_RT kernel on Ubuntu 22.04 before running `elfin_ros_control.launch`.
Follow the vendor tutorial linked from the
[elfin_s_robot README](https://github.com/huayan-robotics/elfin_s_robot/blob/noetic/README.md).

PREEMPT_RT often conflicts with proprietary NVIDIA drivers. Use integrated graphics or Nouveau
for the desktop if needed.

Verify after reboot:

```bash
uname -a | grep PREEMPT
```

### 2. Vendor driver config

Copy the vendor-provided `elfin_drivers.yaml` to:

```
~/elfin_noetic_ws/src/elfin_s_robot/elfin_robot_bringup/config/elfin_drivers.yaml
```

Set the Ethernet interface name to match your NIC (not necessarily `eth0`):

```yaml
elfin_ethernet_name: enp3s0   # example — use ip link show
```

A placeholder template is at `elfin_drivers.yaml.example` in this directory.

### 3. Start the hardware container

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
