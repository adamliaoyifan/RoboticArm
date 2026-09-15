# Noetic TCP executor (`deployment_ws/noetic`)

**Noetic / Docker only.** This tree is the ROS 1 Huayan TCP trajectory executor
(`elfin_cps_executor`). Build it inside the Noetic image, not with colcon.

```bash
./docker/noetic/run.sh build
./docker/noetic/run.sh start
./docker/noetic/run.sh cps robot_ip:=192.168.0.10
```

The package is not under a colcon `src/`, so `colcon build` does not touch it.
`docker/noetic/run.sh` bind-mounts it to `/catkin_ws/src/elfin_cps_executor`.

Hardware paths (EtherCAT vs TCP) are in [`docker/noetic/HARDWARE.md`](../docker/noetic/HARDWARE.md).

The ROS 2 executor for the live cell is on `ros2_humble` at
`deployment_ws/src/elfin_trajectory_executor`. This branch still has the older
ROS 2 copy at [`ros2_ws/`](../ros2_ws/).
