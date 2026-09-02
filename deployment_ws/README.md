# Real-robot runtime nodes (`deployment_ws`)

TCP execution for the real Elfin is kept here, separate from the Noetic
simulation / luggage stack in `elfin_noetic_ws`.

| Path | ROS | Build | Role |
|---|---|---|---|
| `src/elfin_trajectory_executor` | Humble | `colcon` | `FollowJointTrajectory` via Huayan CPS |
| `noetic/elfin_cps_executor` | Noetic | catkin in Docker | Same TCP path, plus `/execute_trajectory` |

The Noetic package is not under `src/` so `colcon build` does not touch it.
The Noetic container bind-mounts it to `/catkin_ws/src/elfin_cps_executor`.

```bash
./docker/noetic/run.sh hw-exec robot_ip:=192.168.0.10
ros2 launch elfin_trajectory_executor real.launch.py robot_ip:=192.168.0.10
```
