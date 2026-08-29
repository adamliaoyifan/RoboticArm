# Todo 3 — pick / retreat 的 ROS 2 壳

对应 plan todo `pick-retreat-ros2`。把已有的 `build_sequence` 几何库接进 Humble 图，
再补一个**薄**执行器，让四段 pick 动作能真的跑起来。

## 范围

`waypoint_generator_node`（服务）+ `motion_planner_node`（action）+
`sim_world` 里起 `move_group`。

**不做**：吸盘 / `VacuumCommand` / ACM attach、place 段、多箱、
把 3203 行的 rospy `motion_planner_node.py` 整文件搬过来。

## 交付物

| 文件 | 性质 |
|---|---|
| `luggage_planning/luggage_planning/motion_executor.py` | 新，无 ROS 的 MoveIt 客户端封装 |
| `luggage_planning/scripts/ros1_reference/waypoint_generator_node.py` | 现有 rospy 版挪到这里 |
| `luggage_planning/scripts/ros1_reference/motion_planner_node.py` | 现有 rospy 版挪到这里（约 3203 行，只搬家不改逻辑） |
| `luggage_planning/scripts/waypoint_generator_node.py` | 新，Humble 壳，**沿用原名** |
| `luggage_planning/scripts/motion_planner_node.py` | 新，Humble 壳，**沿用原名**（不要改叫 `motion_executor_node.py`） |
| `luggage_planning/CMakeLists.txt` | 改：加 `install(PROGRAMS ...)`，只装 Humble 节点 |
| `luggage_planning/package.xml` | 改：加 `rclpy` / `luggage_msgs` / `moveit_msgs` / `tf2_ros` |
| `luggage_gazebo/launch/sim_world.launch.py` | 改：起 `move_group` + 两个新节点 |

### ROS 1 参考文件（已确认）

各包 `scripts/` 下的 rospy `.py` **先挪到** `scripts/ros1_reference/`（与
`luggage_perception` 相同），再在原路径写 Humble 节点。参考实现不删、不改逻辑。

- Humble 节点继续用原来的名字：`waypoint_generator_node.py`、
  `motion_planner_node.py`。`motion_executor.py` 仍是无 ROS 的客户端库。
- 这是全仓库的做法，不限于本 todo：`luggage_packing`、`luggage_bringup` 等
  同样有 rospy 脚本时，**轮到改那个包再搬**，不要现在整仓搬家。
- `ros1_reference/` 不进 `install(PROGRAMS ...)`。

---

## 1. 关键决策：MoveIt 2 怎么用

**已确认的环境事实**（`ros2 pkg list` / `python3 -c import`）：

- `moveit_commander` **不存在**（那是 ROS 1）
- `moveit_py` **不存在**（Humble 上没装）
- `moveit_msgs` 齐全：`action/MoveGroup`、`action/ExecuteTrajectory`、
  `srv/GetCartesianPath`、`srv/GetPositionIK`、`srv/GetPositionFK`、
  `srv/GetStateValidity`
- `elfin_moveit_config` 存在：`config/S20.srdf` 里 group `elfin_arm`、
  end effector `elfin_ee`；`config/moveit_controllers.yaml` 已指向
  `elfin_arm_controller`，`action_ns: follow_joint_trajectory`
- `elfin_mvp_bringup/launch/demo.launch.py` 已经会用
  `MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")` 起 `move_group`
- **`sim_world.launch.py` 默认起 `move_group`**（`use_moveit:=true`，
  `use_sim_time: True`）。关掉：`use_moveit:=false`
- Humble apt **没有** `ros-humble-moveit-py`（那是 Iron/Rolling）。
  Python 客户端装 **`ros-humble-pymoveit2`**（底层仍是 `moveit_msgs`
  action/service）。`moveit-runtime` 补 `moveit_ros_perception`。

**结论**：规划/IK 走 `move_group`；Python 侧用 `moveit_msgs` 裸客户端或
`pymoveit2`（同一套接口的薄封装）。纯 FJT 方案不可行——`pre_grasp`
是 `pose_target`，没有 IK 就没法从位姿得到关节角，而 IK 也只有 `move_group`
提供（`/compute_ik`）。

`sim_world` 传给 `move_group` 的 `robot_description` 必须是带吸盘/相机的
Gazebo URDF（与 RSP 同一份），不能用 `MoveItConfigsBuilder` 默认的裸
`S20.urdf.xacro`。URDF `<robot name>` 必须是 **`S20`**，与 SRDF 一致。

| 段类型 | 接口 |
|---|---|
| `pose_target` | `moveit_msgs/action/MoveGroup`（OMPL） |
| `cartesian` | `moveit_msgs/srv/GetCartesianPath` → `moveit_msgs/action/ExecuteTrajectory` |
| 命名关节位姿 | 直接 FJT `/elfin_arm_controller/follow_joint_trajectory`，绕开 MoveIt 更稳 |

`GoToRobotPose` 走最后一行：`robot_poses.yaml` 里就是关节角，没必要过规划器。
`observe_pose_hold.py` 已经是这个做法，可以直接抄。

---

## 2. `build_sequence` 的既有行为（不要改）

```python
build_sequence(pick, place_slot, phase, pick_clearances=None,
               place_clearance_z=None, perception_info=None, opening_info=None)
```

`phase="pick"` 产出 4 段，顺序固定：

| # | name | type | 其他字段 |
|---|---|---|---|
| 1 | `pre_grasp` | `pose_target` | 默认 |
| 2 | `approach` | `cartesian` | `allow_ompl_fallback=True` |
| 3 | `attach` | `cartesian` | `allow_ompl_fallback=True` |
| 4 | `pick_retreat` | `cartesian` | `keep_tool_down=True`, `allow_ompl_fallback=True` |

Z 的算法：`top_z = pick.pose.position.z + max(0, pick.height) * 0.5`，
每段 `z = top_z + clearance[name]`。默认 clearance 是模块常量
`DEFAULT_PICK_CLEARANCES`（`pre_grasp` 0.30 / `approach` 0.25 / `attach` 0.0 /
`pick_retreat` 0.35），**不读 YAML**；ROS 1 节点通过 `~pick_*_clearance` 参数覆盖。

位姿在 **world 系**，姿态是 tool-down + `yaw`。

Pick 抬离段名、clearance 键、`segment_names_for_phase("pick")` 一律是
**`pick_retreat`**。Place 抬离仍叫 **`retreat`**，不要混用。

### `perception_info` 与自适应 clearance

`build_sequence` 接受 `perception_info={"box_top_z":..., "suction_z":...}`，
非 None 时走 `_perception_clearances()`。ROS 1 节点里 `suction_z` 来自 TF 查
`suction_contact_frame` 在 world 系的 Z（`~use_perception_approach` 默认 True）。

本轮建议：**先关掉**（`use_perception_approach: false`），用固定 clearance。
自适应逻辑多一个 TF 依赖和一条分支，先让固定路径跑通再说。

---

## 3. `waypoint_generator_node`

薄。服务 `BuildMotionSequence`：

```
DetectedLuggage pick
SlotSpec place_slot
string phase
---
MotionSegment[] segments
bool success
string message
```

节点做的事：

1. `luggage_msgs/DetectedLuggage` → `luggage_planning.pose` 的输入形态
2. 读 clearance 参数（默认取 `DEFAULT_PICK_CLEARANCES`）
3. 调 `build_sequence(..., phase="pick")`
4. `luggage_planning.pose.MotionSegment` → `luggage_msgs/MotionSegment`
5. 空列表 → `success=false` + message

**msg 与 dataclass 字段一一对应**，转换是机械的：

| dataclass (`pose.py`) | msg (`MotionSegment.msg`) |
|---|---|
| `name: str` | `string name` |
| `type: str` | `string type` |
| `target_pose: Pose` | `geometry_msgs/Pose target_pose` |
| `waypoints: List[Pose]` | `geometry_msgs/Pose[] waypoints` |
| `keep_tool_down` | `bool keep_tool_down` |
| `keep_camera_down` | `bool keep_camera_down` |
| `lock_wrist` | `bool lock_wrist` |
| `allow_ompl_fallback` | `bool allow_ompl_fallback` |

注意 `luggage_planning.pose.Pose` 是自己的 dataclass，不是
`geometry_msgs.msg.Pose`，别混。转换函数放节点侧或一个
`luggage_planning/ros_message_adapters.py`——**不要**放进
`waypoint_generator.py`，那是算法模块，禁止 import 消息类型（连惰性 import 都不行）。

`place_slot` 本轮传空/占位，`phase="pick"`。可视化 marker 后补。

---

## 4. `MotionExecutor` + 执行节点

### 算法侧 vs 节点侧的切分

`MotionExecutor` 想做成"无 ROS 算法类"是**做不到的**：它的本质就是调 MoveIt
的 action/service，必然 import `moveit_msgs`。按
[perception_architecture.md](../architecture/perception_architecture.md) 的分类，
它属于**节点层**（和 `ros_message_adapters.py` 同类），放在
`luggage_planning/luggage_planning/motion_executor.py` 但顶层 import 消息是合规的，
算法模块不许 import 它。

真正无 ROS 的部分是几何/判据：settle 判定（`settle_criterion.py` 已有）、
downward 约束检查（`downward_constraint_utils.py` 已有）。这些复用，别重写。

### 接口

```
PlanMotion.action:
  MotionSegment segment
  ---
  bool success / string message
  ---
  string stage        # planning | executing | settling
  string segment_name
  float64 fraction
```

**一次一段**。驱动自己 for 循环四段，不做 `PlanSequence`。

### 执行细节

**`pose_target`**：构造 `moveit_msgs/action/MoveGroup` goal，
`group_name="elfin_arm"`，`PositionConstraint` + `OrientationConstraint` 到
`target_pose`，规划器 OMPL。目标末端 link 是 **`suction_contact_frame`**
（ROS 1 的 `~pick_pose_target_link` 默认值），不是 `elfin_end_link`——搞错的话
整条轨迹会差一个吸盘长度。

**`cartesian`**：`GetCartesianPath` 请求，`waypoints=[target_pose]`，
`max_step` ~0.01、`jump_threshold=0.0`。返回 `fraction`：

- `fraction >= cartesian_min_fraction`（ROS 1 默认 0.95）→ `ExecuteTrajectory`
- 否则若 `allow_ompl_fallback` → 退化成 `pose_target` 走 OMPL
- 否则 `success=false`，message 带上 fraction

`fraction` 要通过 feedback 上报，这是排查失败最有用的一个数。

**约束**：`keep_tool_down` / `keep_camera_down` / `lock_wrist` 三个 flag 本轮
可以**先只实现 `keep_tool_down`**，另两个记 TODO 并在 message 里说明未生效。
不要假装实现了。ROS 1 的 `_build_segment_constraints`（L1898+）是参考。

**settling**：每段执行后等机械臂稳定再返回，否则下一段的 IK 种子和 TF 都是
运动中的值。复用 `settle_criterion.SettleTracker`。这一步同时也让预处理的
`geometry_ok` 恢复，检测才能用。

### 时序与超时

- `execute_timeout` 默认 45 s（ROS 1 值），仿真里可以短一些
- `move_group` 启动慢，节点要等 action server 可用再宣告 ready
- gz 里 controller update 周期 10 ms 而物理 1 ms，日志里那条
  `Desired controller update period (0.01 s) is slower than the gazebo
  simulation period (0.001 s)` 是已知的、无害的

---

## 5. launch 与打包

`sim_world.launch.py` 已加 `move_group`：抄
`elfin_mvp_bringup/launch/demo.launch.py` 的
`MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")`，
**`use_sim_time: True`**，并用仿真那份 `robot_description` 覆盖裸臂 URDF。
`use_moveit` 默认 **`true`**。感知-only 调试再 `use_moveit:=false`。

`luggage_gazebo/package.xml` 已有 `elfin_moveit_config`、`moveit_ros_move_group`、
`moveit_configs_utils`。闭环执行节点起来后再加 `luggage_planning`。

`luggage_planning/CMakeLists.txt` 现在**只装 python 包和 `data/`**，
没有 `install(PROGRAMS ...)`。要加，否则 launch 找不到可执行文件。

`luggage_planning/package.xml` 现有 exec_depend 只有
`ament_index_python` / `luggage_description` / `python3-numpy` / `python3-yaml`。
缺 `rclpy`、`luggage_msgs`、`moveit_msgs`、`tf2_ros`、`geometry_msgs`，都要补。

---

## 6. 测试

**ROS-free**（`luggage_planning/test/`）：

- `build_sequence(phase="pick")` 返回 4 段，名字与顺序符合预期
- `top_z` 计算：`pose.z + height/2`
- clearance 覆盖生效
- `pick_retreat` 的 `keep_tool_down=True`
- `segment_names_for_phase("pick")` 与 `build_sequence` 第四段都是 `pick_retreat`；
  place 最后一段仍是 `retreat`

**需要 ROS 但不需要 Gazebo**：dataclass ↔ msg 转换往返测试，
用 `pytest.importorskip("luggage_msgs")` 守住。

**需要 Gazebo**：不写单测，进 todo 4 的驱动。

## 7. 验收

- `BuildMotionSequence(phase=pick)` 对一个真实 `DetectLuggage` 结果返回 4 段，
  Z 递减符合 clearance
- 四段依次 `PlanMotion` 全部 `success=true`
- feedback 里能看到 `stage` 变化和 cartesian 的 `fraction`
- retreat 后末端 Z 明显高于 attach（记录数值）
- 中途取消 goal 能干净停下，机械臂不飞车
- `move_group` 不在时节点不崩，报清楚的"等不到 action server"
