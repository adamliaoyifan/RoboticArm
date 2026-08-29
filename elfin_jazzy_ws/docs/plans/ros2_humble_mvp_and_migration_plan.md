# Elfin ROS 2 Humble MVP 与完整迁移计划

文档状态：Phase 2 `luggage_msgs` implemented (Phase 1 Gates 0–5 still hold)  
更新时间：2026-08-14  
工作区：`elfin_humble_ws`  
目标平台：Ubuntu 22.04 + ROS 2 Humble  
勾选清单：[ros2_migration_todo.md](ros2_migration_todo.md)  
仿真后端：[sim_backends_ros2.md](sim_backends_ros2.md)  
验收：[mvp_gates.md](../status/mvp_gates.md)、[gpu_runtime.md](../status/gpu_runtime.md)、[sim_smoke.md](../status/sim_smoke.md)、[phase2_interfaces.md](../status/phase2_interfaces.md)

## 1. 结论

该工程可以迁移到 ROS 2。迁移不应从批量替换 `rospy` 开始，而应先建立一条稳定、标准化、可替换后端的控制链：

```text
Elfin S20 URDF/Xacro
        |
robot_state_publisher
        |
ros2_control controller_manager
        |
joint_trajectory_controller
        |
FollowJointTrajectory action
        |
MoveIt 2 MoveGroupInterface
        |
固定关节目标的规划与执行
```

MVP 使用 `mock_components/GenericSystem`，不依赖 Huayan SDK、不依赖真机、不依赖物理仿真。MVP 验收后，保持上层接口不变，将 mock 后端分别替换为 `gz_ros2_control`（Gazebo Fortress）、`mujoco_ros2_control` 和 Huayan SDK 硬件插件。Isaac Sim 通过薄适配器接入，不替换 MoveIt 业务层。

交互式仿真和 RViz2 必须使用 NVIDIA GPU。`llvmpipe` 视为失败。见 [gpu_runtime.md](../status/gpu_runtime.md)。

## 2. 当前基线

当前 `elfin_humble_ws` 仍以 ROS 1 Noetic 业务包为主（`COLCON_IGNORE`），并已加上 Phase 1 ROS 2 MVP 包和 Phase 2 `luggage_msgs`：

- 6 个 Catkin 包（`luggage_msgs` 已转为 ROS 2）。
- 61 个有效 ROS Python 文件。
- 1 个 ROS 1 C++ 点云节点。
- ROS 2 `luggage_msgs`：5 个 msg、20 个 srv、6 个 action。
- 23 个有效 ROS 1 launch 文件。
- 79 处 `rospy.set_param`，包含大量运行时共享状态。
- `src/CMakeLists.txt` Noetic symlink 已删除（Gate 1）。
- Dockerfile 使用 `osrf/ros:noetic-desktop-full` 和 `catkin_make`。
- MoveIt launch 仍引用已删除的 `elfin_s20_moveit_config`。
- Gazebo 包仍引用已删除的 `elfin_gazebo`。
- `luggage_description` 仍通过 `$(find elfin_description)` 引用机器人描述。
- `src/pointcloud/elfin-noetic.tar` 是约 5.1 GB 的 Noetic OCI 镜像归档，不参与源码构建。

已经保留的可复用资产位于：

```text
robot_assets/elfin_description/
  meshes/S05
  meshes/S10
  meshes/S20
  meshes/S30
  urdf/S05.urdf.xacro
  urdf/S10.urdf.xacro
  urdf/S20.urdf.xacro
  urdf/S30.urdf.xacro
  urdf/materials.xacro
```

本机已有 `/opt/ros/humble`。Gate 0 已安装 MoveIt 2、`ros2_control`、`ros2_controllers`；Gate 0.5 已安装 Gazebo Fortress（`ros-humble-ros-gz`、`gz-ros2-control`）和 `mujoco-ros2-control`。环境记录见 `docs/status/gate0_environment.md` 与 `docs/status/gpu_runtime.md`。Humble 2.54 上 `mock_components/GenericSystem` 由 `hardware_interface` 提供，不是独立 ROS 包。不安装 Gazebo Classic，本批不安装 Isaac。

## 3. MVP 定义

### 3.1 MVP 目标

在完全不启动 ROS 1 的条件下，实现以下闭环：

1. ROS 2 能展开和加载 Elfin S20 Xacro。
2. RViz2 能显示完整 S20 模型和 TF。
3. `controller_manager` 能加载并激活关节状态广播器和轨迹控制器。
4. `/joint_states` 连续发布六个关节状态。
5. `FollowJointTrajectory` 能执行一个固定六关节目标。
6. MoveIt 2 能为同一目标规划并执行轨迹。
7. 整个 MVP 使用 `colcon build` 构建，运行时不依赖 Catkin、`roscore`、`rospy` 或 ROS 1 bridge。

### 3.2 MVP 非目标

以下内容明确不属于 MVP：

- Huayan SDK 和真机通信。
- EtherCAT、SOEM 和实时内核配置。
- Gazebo 世界、相机、行李箱和吸盘仿真。
- RGB-D、点云过滤、OctoMap 和语义分割。
- 装箱算法、orchestrator 和 GUI。
- S05、S10、S30 多机型支持。
- ROS 1/ROS 2 bridge。
- 对 ROS 1 全业务性能的最终等价证明。

这些内容不进入 MVP，是为了先验证所有后续模块共同依赖的 ROS 2 描述、规划和控制接口。

### 3.3 MVP 包结构

建议在 `src/` 下创建以下 ROS 2 包：

```text
src/
  elfin_description/       # S20 Xacro、mesh、ros2_control 描述、RViz 配置
  elfin_control/           # 控制器 YAML 和 mock hardware 配置
  elfin_moveit_config/     # SRDF、运动学、OMPL、MoveIt 控制器映射
  elfin_mvp_bringup/       # 启动文件和确定性演示节点
```

包职责必须保持分离：

| 包 | 构建类型 | MVP 职责 |
|---|---|---|
| `elfin_description` | `ament_cmake` | 安装模型资源，生成 `robot_description`，声明六关节 `ros2_control` 接口 |
| `elfin_control` | `ament_cmake` | 配置 `mock_components/GenericSystem`、`joint_state_broadcaster`、`joint_trajectory_controller` |
| `elfin_moveit_config` | `ament_cmake` | 定义 `elfin_arm` planning group、关节限制、OMPL 和 MoveIt controller mapping |
| `elfin_mvp_bringup` | `ament_cmake` | 组合启动控制器、MoveIt 2、RViz2，并提供固定目标演示程序 |

MVP 不应把全部内容塞进一个 bringup 包。描述、控制和 MoveIt 配置将分别被真机、仿真和业务流程复用。

## 4. MVP 实施步骤

### Gate 0：环境和依赖

目标：明确环境问题与代码问题的边界。

工作项：

- 固定 Ubuntu 22.04、ROS 2 Humble、`amd64` 或目标 `arm64` 架构。
- 安装或通过 ROS 2 Docker 镜像提供：
  - `ros-humble-desktop`
  - `ros-humble-moveit`
  - `ros-humble-ros2-control`
  - `ros-humble-ros2-controllers`
  - `ros-humble-controller-manager`
  - `ros-humble-xacro`
  - `ros-humble-rviz2`
- 使用 `rosdep` 解析工作区依赖。
- 记录 RMW 实现、内核版本、CPU 型号和构建类型。
- 默认使用 `Release` 构建进行性能测量。

通过条件：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix controller_manager
ros2 pkg prefix joint_trajectory_controller
ros2 pkg prefix hardware_interface
test -f /opt/ros/humble/share/hardware_interface/mock_components_plugin_description.xml
```

前四条能找到包。最后一条确认 `mock_components/GenericSystem` 插件存在。Humble 2.54 没有独立的 `mock_components` ROS 包。

### Gate 0.5：Fortress、MuJoCo 与 GPU 硬门

目标：产品仿真和对照仿真的系统包可用，且交互渲染走 NVIDIA GPU。

工作项：

- 安装 Humble 官方 Fortress 栈：`ros-humble-ros-gz`、`ros-humble-ros-gz-sim`、`ros-humble-gz-ros2-control`。不要装 Harmonic 官方冲突源，不要装 `ros-humble-gazebo-ros`。
- 安装 `ros-humble-mujoco-ros2-control` 及其 demos/msgs。
- 交互式 Gazebo / MuJoCo Simulate / RViz2 / Isaac 禁止 `llvmpipe`。
- 记录 `nvidia-smi` 与 `glxinfo` 到 `docs/status/gpu_runtime.md`。
- 本批不安装 Isaac Sim / Isaac Lab。

通过条件：

```bash
source /opt/ros/humble/setup.bash
gz sim --versions
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix gz_ros2_control
ros2 pkg prefix mujoco_ros2_control
glxinfo -B | grep -i renderer
```

`gz sim --versions` / `ign gazebo --versions` 为 Fortress（本机 6.18.0）。renderer 含 `NVIDIA`，不含 `llvmpipe`。本机 CLI 为 `ign gazebo`。

### Gate 1：隔离 ROS 1 源码

目标：使 `colcon` 只处理新 ROS 2 包，同时保留旧实现作为迁移参考。

工作项：

- 移除顶层指向 Noetic 的 `src/CMakeLists.txt` 符号链接。
- 在尚未迁移的 `src/luggage_*` 包中放置 `COLCON_IGNORE`。
- `src/pointcloud` 不是 ROS 包，保持原位；5.1 GB Noetic 归档不进入构建上下文。
- 不删除 ROS 1 包，直到对应 ROS 2 阶段通过验收。

通过条件：

```bash
colcon list
```

输出只能包含已经创建的 ROS 2 包，不包含 Catkin 包。

### Gate 2：S20 ROS 2 描述包

目标：在 ROS 2 中正确加载物理模型。

工作项：

- 从 `robot_assets/elfin_description` 复制 S20 mesh、材料和 Xacro 到 `src/elfin_description`。
- 保留关节原点、轴、质量、惯量、限制和 mesh URI。
- 将 `$(find package)` 改为 ROS 2 支持的资源解析方式。
- 给 Xacro 增加硬件插件参数，例如 `hardware_plugin`。
- 增加六个关节的 position command interface 和 position/velocity state interface。
- MVP 默认插件设为 `mock_components/GenericSystem`。
- 使用 `robot_state_publisher` 发布 TF。
- 添加 RViz2 配置，仅显示 RobotModel 和 TF。

通过条件：

- Xacro 展开无错误。
- URDF 解析无错误、无断裂 link、无重复 joint。
- RViz2 中 mesh、关节方向和初始姿态正确。
- 关节名称严格保持：`elfin_joint1` 至 `elfin_joint6`。

### Gate 3：ros2_control 控制链

目标：建立未来仿真和真机共用的控制接口。

工作项：

- 启动 `ros2_control_node`。
- 配置并激活：
  - `joint_state_broadcaster`
  - `elfin_arm_controller/JointTrajectoryController`
- 控制器使用六关节 position command interface。
- MVP 控制器更新率暂定 100 Hz；该数值不是最终真机频率。
- 状态发布率设为 50 Hz。
- 配置 goal/path tolerance 和非零超时，禁止依赖 ROS 1 仿真中的宽松容差。
- 使用 controller spawner 的就绪状态替代固定 `sleep`。

通过条件：

```bash
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
ros2 topic hz /joint_states
```

- 两个控制器均为 `active`。
- Action 存在。
- `/joint_states` 至少包含全部六关节，发布频率稳定在配置值附近。

### Gate 4：直接轨迹执行

目标：在引入 MoveIt 前独立验证控制层。

工作项：

- 使用 `FollowJointTrajectory` 发送一个位于全部 joint limit 内的保守目标。
- 演示目标不得靠近 joint limit，不允许使用全零作为唯一测试。
- 记录目标、反馈、结果、总耗时和最终误差。
- 执行目标后再发送返回初始姿态的轨迹。

通过条件：

- Action 返回成功。
- 六关节均达到目标。
- mock 后端最终最大绝对关节误差不大于 `0.001 rad`。
- 连续执行 20 次，无失败、无 controller lifecycle 异常。

### Gate 5：MoveIt 2 规划与执行

目标：证明业务规划层可以使用标准 ROS 2 控制接口。

工作项：

- 创建 S20 SRDF，planning group 名称保持 `elfin_arm`。
- 定义 end effector link 和必要的 disabled collision pairs。
- 创建 `joint_limits.yaml`、`kinematics.yaml`、`ompl_planning.yaml`。
- 配置 MoveIt Simple Controller Manager 指向同一个 `FollowJointTrajectory` Action。
- 编写最小 C++ `MoveGroupInterface` 演示节点。
- 演示节点只执行确定性的关节空间目标，不包含业务逻辑。

选择 C++ 而不是 Python 的原因：Humble 上 C++ `MoveGroupInterface` 是稳定、完整的主路径，同时该节点以后可以承载性能敏感的运动执行和约束逻辑。

通过条件：

- `move_group` 正常启动并识别 `elfin_arm`。
- 规划结果成功且轨迹包含全部六关节。
- 执行 Action 成功。
- 最终最大绝对关节误差不大于 `0.01 rad`。
- 连续规划执行 20 次，成功率 100%。
- 全程没有 ROS 1 进程、ROS 1 环境变量或 bridge。

## 5. MVP 启动与验收接口

计划提供两个 launch 入口：

```bash
# 描述和控制器，不启动 MoveIt/RViz
ros2 launch elfin_mvp_bringup control.launch.py use_rviz:=false

# 完整 MVP：mock hardware + controller + MoveIt 2 + RViz2
ros2 launch elfin_mvp_bringup demo.launch.py use_rviz:=true
```

计划提供一个确定性演示命令：

```bash
ros2 run elfin_mvp_bringup move_to_joint_goal
```

建议的全量 MVP 验收命令：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch elfin_mvp_bringup demo.launch.py use_rviz:=false
ros2 run elfin_mvp_bringup move_to_joint_goal
```

MVP Definition of Done：

- [x] 只使用 ROS 2 Humble 构建和运行。
- [x] S20 模型和 TF 正确。
- [x] 两个 `ros2_control` 控制器处于 active。
- [x] 直接轨迹执行 20/20 成功。
- [x] MoveIt 2 规划执行 20/20 成功。
- [x] 最大最终关节误差满足验收值。
- [x] 新增 ROS 2 smoke/integration tests 通过。
- [x] 启动、运行、停止过程无残留进程。
- [x] 验收结果记录到 `docs/status/`。

## 6. MVP 后的目标架构

```text
                         +----------------------+
                         | luggage_bringup ROS 2|
                         | orchestrator/actions |
                         +----------+-----------+
                                    |
              +---------------------+---------------------+
              |                                           |
    +---------v----------+                      +---------v----------+
    | luggage_planning   |                      | luggage_perception |
    | MoveIt 2 / C++ API |                      | C++ components     |
    +---------+----------+                      +---------+----------+
              |                                           |
     MoveGroup / PlanningScene                  PointCloud2 / Image
              |                                           |
    +---------v-------------------------------------------v----------+
    | ROS 2 topics, services, actions, TF2, explicit QoS             |
    +-------------------------------+--------------------------------+
                                    |
                        FollowJointTrajectory
                                    |
                  +-----------------v-----------------+
                  | ros2_control controller_manager  |
                  | joint_trajectory_controller      |
                  +-----------------+-----------------+
                                    |
             +----------------------+----------------------+
             |                                             |
             +----------------------+----------------------+
             |                      |                      |
   +---------v-----------+  +-------v--------+  +---------v---------+
   | gz_ros2_control     |  | mujoco_ros2_   |  | elfin_hardware    |
   | Gazebo Fortress GPU |  | control        |  | Huayan SDK C++    |
   +---------------------+  +-------+--------+  +-------------------+
                                    |
                            Isaac Sim bridge
                            (thin adapter, later)
```

## 7. 完整迁移阶段

### Phase 1：MVP 基础控制链

范围：本文第 3 至第 5 节。

交付物：

- `elfin_description`
- `elfin_control`
- `elfin_moveit_config`
- `elfin_mvp_bringup`
- MVP 验收报告

退出条件：MVP Definition of Done 全部满足。

### Phase 2：自定义接口迁移

范围：迁移 `luggage_msgs`。

工作项：

- 使用 `ament_cmake` 和 `rosidl_generate_interfaces`。
- 保留 5 个 `.msg` 和 26 个 `.srv` 的字段语义。
- 调整 ROS 2 类型引用和 Python/C++ 导入路径。
- 对长时间操作重新分类：
  - 运动规划和执行改为 Action。
  - 探索视点执行改为 Action。
  - 装箱主流程改为 Action 或由 orchestrator 状态机内部管理。
  - 快速查询、复位和原子状态变更保留 Service。
- 给新 Action 定义取消、反馈、超时和幂等语义。

退出条件：接口包独立构建；Python/C++ typesupport 可加载；接口语义评审完成。

### Phase 3：纯算法模块 ROS 2 打包

范围：不依赖 ROS 的 Python 模块。

优先迁移：

- `luggage_packing` 中的 EMS、placement solver、评分和 free-space 模型。
- `luggage_perception` 中的体素、几何、检测后处理和深度过滤算法。
- `luggage_planning` 中的视点生成、约束计算、atlas 和 settle criterion。
- `luggage_description` 中的 YAML、几何和资源解析工具。

工作项：

- 使用标准 Python package 结构和 `setup.py/setup.cfg`，停止通过 `sys.path` 和 `rospkg.RosPack()` 导入。
- 通过 `ament_index_python` 定位安装后的 share 资源。
- 保持算法函数输入输出与 ROS 消息解耦。
- 为新 ROS 2 包创建聚焦的单元测试；不恢复已删除的 ROS 1 测试。

退出条件：纯算法测试独立于 ROS graph 运行，结果与当前实现的固定数据集一致。

### Phase 4：TF、描述和场景配置

范围：`luggage_description` 和场景静态 TF。

工作项：

- 全部 `tf` API 统一为 `tf2_ros`。
- 静态变换使用 `StaticTransformBroadcaster` 和 transient-local QoS。
- `robot_description`、SRDF、joint limits 作为所属节点参数，不模拟 ROS 1 全局参数服务器。
- 配置 YAML 按 ROS 2 节点名和 `ros__parameters` 组织。
- 取消 `description_params_node` 向其他节点命名空间批量写参数的模式。

退出条件：TF tree、场景坐标和 ROS 1 基线在数值上等价。

### Phase 5：感知高吞吐链

范围：RGB-D、PointCloud2、语义过滤、Cargo map、world scene map。

工作项：

- 首先迁移 C++ `task_cloud_filter_node` 到 `rclcpp`。
- 将连续点云处理节点实现为 `rclcpp_components`，支持同进程组合。
- 采用 `SensorDataQoS`，点云/Image 默认 `best_effort + keep_last(1)`。
- 对必须成对处理的 Image/PointCloud2 使用 ROS 2 `message_filters`。
- 缓存 TF，按传感器时间戳查询；继续保持 fail-closed 行为。
- 能保留 NumPy/Torch 的推理逻辑，但 `rclpy` 回调只负责搬运和调度。
- 将每帧 `set_param` 诊断改为 typed diagnostics topic。
- 状态快照使用 topic；原子读写使用 service；大数据禁止进入参数服务。

退出条件：

- 输出语义与 ROS 1 固定 bag/dataset 对齐。
- 点云处理 P95 延迟、吞吐和 CPU 不劣于基线目标。
- 队列不随运行时间持续增长。

### Phase 6：MoveIt 2 规划与场景管理

范围：`luggage_planning`。

工作项：

- 将 3200 行 `motion_planner_node.py` 拆分为：
  - MoveIt 2 C++ adapter。
  - 轨迹执行 Action server。
  - 约束与验证算法库。
  - settle/hold 安全逻辑。
- 使用 C++ `MoveGroupInterface` 或更底层 Planning Component。
- 迁移 PlanningScene、AttachedCollisionObject、AllowedCollisionMatrix。
- 保持 `elfin_arm`、link 名称、joint 名称及业务约束语义。
- 使用 `rclcpp_action` 调用 MoveIt 和 `FollowJointTrajectory`。
- 对 IK、state validity、FK 等批量请求评估直接库调用，减少大量同步 service round-trip。
- 用 callback groups 和 MultiThreadedExecutor 消除同步等待死锁。

退出条件：固定场景的规划成功率、轨迹约束、碰撞结果和执行结果达到 ROS 1 基线。

### Phase 7A：Gazebo Fortress 产品仿真（GPU 硬门）

范围：`luggage_gazebo` 和仿真专用吸盘逻辑。细节见 [sim_backends_ros2.md](sim_backends_ros2.md) 与 [ros2_migration_todo.md](ros2_migration_todo.md)。

工作项：

- 使用 ROS 2 Humble 官方配套的 Gazebo Fortress。
- 使用 `ros_gz_sim`、`ros_gz_bridge` 和 `gz_ros2_control`。
- 将 Gazebo Classic world/model/plugin 迁移到 Fortress 支持的 SDF。
- 替换 `/gazebo/spawn_sdf_model`、`delete_model`、`get_model_state` 等 Classic service。
- 通过 bridge 只桥接必要的相机、时钟和模型状态。
- launch 启动前检查 GPU renderer；`llvmpipe` 直接失败。
- 吸盘 attach/follow 实现为 Gazebo system plugin 或仿真节点，ROS 真空 service 与真机同名。

退出条件：同一个 MoveIt 2 与 controller 配置可以在 mock 和 Fortress 之间切换；单箱 pick-place 可重复；renderer 为 NVIDIA。

### Phase 7B：MuJoCo 对照后端

工作项：

- 从已展开的 S20 URDF 生成并手修 MJCF，关节名不变。
- 容器/台座 geom 位姿由 `scene_tf.yaml` 生成。
- 使用 `mujoco_ros2_control` 自带 `ros2_control_node` 与 `MujocoSystemInterface`。
- 同一固定关节目标，误差门槛与 mock 相同。
- 不把感知主路径切到 MuJoCo，除非相机质量达标。

退出条件：窗口 GPU 渲染；`FollowJointTrajectory` 对照通过。

### Phase 7C：Isaac Sim / Isaac Lab（不阻塞装箱）

工作项：

- 本阶段再安装 Isaac Sim；Gate 0.5 不装。
- URDF/STL → USD；`scene_tf.yaml` → Xform。
- 双进程：Sim（内部 Humble 库，未 source `/opt/ros/humble`）+ `elfin_humble_ws`。
- 先只读 `/clock`、图像、`joint_states`，再做 `FollowJointTrajectory` 薄适配器。
- 同一 USD 在 Isaac Lab 加载 1 个 env smoke。Lab 不跑 MoveIt。

退出条件：Humble 侧能看到图和关节；一次保守关节目标成功。

### Phase 8：Huayan SDK 真机后端

范围：新增 `elfin_hardware_vendor` 和 `elfin_hardware`。

Gate 8A：SDK 可用性验证：

- 检查 SDK zip 的 header、`.so/.a`、Python binding、示例和许可证。
- 确认支持的 CPU 架构、Ubuntu、glibc、GCC 和 C++ ABI。
- 确认 SDK 是同步还是异步接口、线程模型、控制频率和超时语义。
- 确认关节单位、顺序、零位、方向、限位和错误码。
- 在 ROS 之外先完成 connect/read state/enable/disable/stop 的 smoke test。
- 不允许在未确认急停和失联行为前发送运动命令。

Gate 8B：`ros2_control` 插件：

- `elfin_hardware` 继承 `hardware_interface::SystemInterface`。
- 实现 `on_init`、configure/activate/deactivate、`read`、`write`。
- SDK 阻塞 I/O 使用专用线程和预分配缓冲区，不阻塞 controller manager 更新循环。
- `read/write` 路径禁止 Python、文件 I/O、动态日志洪泛和无界等待。
- 生命周期状态对应 SDK connect/enable/disable/disconnect。
- 通信超时、protective stop、SDK error 必须使插件进入可识别的错误状态。
- 状态/命令接口至少覆盖 position；是否暴露 velocity/effort 由 SDK 实际能力决定。

Gate 8C：真机渐进验证：

1. 只读关节状态。
2. 上电但不运动。
3. 单关节、小幅度、低速度运动。
4. 六关节保守目标。
5. `FollowJointTrajectory`。
6. MoveIt 2 规划执行。
7. 业务轨迹。

退出条件：所有安全检查通过，且控制周期、抖动、丢包、轨迹误差不劣于规定基线。

若 SDK 二进制不兼容 Ubuntu 22.04，优先级如下：

1. 获取 Huayan 官方 Jammy/目标架构 SDK。
2. 获取源码并在目标环境重编译。
3. 将 SDK 运行在厂商支持的独立进程/主机，通过有界、可监控 IPC 接入。

不得把 ABI 不兼容的 `.so` 强行链接进 `controller_manager`。

### Phase 9：业务节点和 orchestrator

范围：`luggage_bringup`、`luggage_packing` 节点壳和全流程状态机。

工作项：

- `rospy` 迁移为 `rclpy`，性能敏感节点使用 `rclcpp`。
- orchestrator 使用明确状态机和 Action，不依赖全局参数传递状态。
- 使用 lifecycle node 或就绪 service 管理启动顺序，删除 launch 中的 `sleep 5/20/25`。
- GUI 与执行状态通过 Action feedback 和状态 topic 通信。
- 失败、取消和重试必须是显式状态，避免阻塞式 `wait_for_service` 链。

退出条件：mock/Gazebo 下完整装箱流程成功，状态转移和错误恢复可重复。

### Phase 10：工具、数据记录和清理

工作项：

- `rosbag` 改为 rosbag2 或 `rosbag2_py`。
- ROS 1 CLI 改为 `ros2` CLI。
- RViz 配置迁移到 RViz2。
- Dockerfile 改为 Ubuntu 22.04/ROS 2 Humble，并使用 `colcon build`。
- 删除已经有 ROS 2 等价实现且通过验收的 ROS 1 包。
- 清理 `COLCON_IGNORE`、Noetic 顶层链接、Catkin 元数据和 Noetic 依赖。
- 5.1 GB Noetic OCI 归档由用户确认保留、移出工作区或删除，不能无确认执行删除。

退出条件：工作区扫描不到活动的 `catkin`、`rospy`、`roscpp`、`actionlib`、`roslaunch`、`rosbag` 或 Noetic 构建依赖。

## 8. ROS 1 到 ROS 2 映射规则

| ROS 1 | ROS 2 | 迁移约束 |
|---|---|---|
| Catkin | Ament + colcon | C++ 用 `ament_cmake`，Python 用 `ament_python` 或混合包 |
| `rospy` | `rclpy` | 避免在高频数据链使用 Python 多次复制大消息 |
| `roscpp` | `rclcpp` | 点云、控制、MoveIt adapter 优先 C++ |
| `actionlib` | `rclcpp_action` / `rclpy.action` | 必须实现取消、feedback、超时 |
| `tf` | `tf2_ros` | 禁止继续使用 tf1 API |
| latch | transient-local QoS | 只用于静态或低频状态 |
| queue size | QoS history/depth | 传感器流默认 depth 1，控制和服务可靠传输 |
| global param | node-owned parameter | 运行状态改 topic/service/action |
| `rospkg` | `ament_index_cpp/python` | 所有资源必须安装到 share 目录 |
| `rosbag` | rosbag2 | 存储格式、QoS override 和回放时间需要重新验证 |
| `ros_control` | `ros2_control` | 硬件通过 `SystemInterface` 插件接入 |
| MoveIt Commander | MoveIt 2 C++ API | 核心规划执行使用 `MoveGroupInterface` |
| Gazebo Classic | Gazebo Fortress | 使用 `ros_gz` 和 `gz_ros2_control`；交互必须 GPU |
| （无） | MuJoCo | `mujoco_ros2_control` 同一 `FollowJointTrajectory` |
| （无） | Isaac Sim / Lab | `isaacsim.ros2.bridge` + 薄适配器；Lab 不跑 MoveIt |

## 9. 参数服务器重构规则

现有 79 处 `rospy.set_param` 按以下规则分类，不允许机械替换：

| 数据类型 | ROS 2 设计 |
|---|---|
| 启动配置、阈值、文件路径 | 所属节点的声明参数 |
| 当前箱体、吸盘状态、规划摘要 | typed 状态 topic |
| 最新检测、点云统计、诊断 | diagnostics/statistics topic |
| 原子命令和状态变更 | service |
| 长时间运动和探索任务 | action |
| orchestrator 内部状态 | 状态机私有内存，必要时发布只读快照 |
| 需要重启恢复的数据 | 明确的 JSON/YAML/数据库持久层 |

ROS 2 参数只用于配置，不作为高频共享数据库。

## 10. QoS 和执行器策略

| 数据路径 | Reliability | Depth | Durability | 执行建议 |
|---|---:|---:|---:|---|
| RGB、Depth、PointCloud2 | best effort | 1-2 | volatile | C++ component，同进程组合 |
| `/joint_states` | best effort 或 reliable，按驱动验证 | 1-5 | volatile | 控制状态快速覆盖旧样本 |
| TF | ROS 2 默认 TF QoS | 默认 | 按 tf2 | 单独 callback group |
| 静态 TF、静态场景摘要 | reliable | 1 | transient local | 低频发布 |
| 轨迹 Action | reliable | 默认 | volatile | 独立 callback group |
| 服务 | reliable | 默认 | volatile | 回调必须有有界执行时间 |
| 诊断/状态 | reliable | 5-10 | 可选 transient local | 不阻塞主数据链 |

执行器规则：

- 控制插件由 controller manager 的实时/高优先级循环驱动。
- 点云 C++ 组件使用 MultiThreadedExecutor，但同一状态对象必须明确互斥策略。
- Python 节点不能在 callback 中同步等待同一 executor 承载的 service/action。
- MoveIt adapter、场景更新和轨迹反馈使用不同 callback group。

## 11. 性能等价计划

“ROS 2 可运行”不等于“性能等价”。必须先在 ROS 1 保存基线，再比较 ROS 2。

### 11.1 基线指标

控制链：

- SDK/控制器 update rate。
- update loop P50/P95/P99 周期和最大抖动。
- `FollowJointTrajectory` goal 接收到首个 command 的延迟。
- 六关节 RMS/最大跟踪误差。
- 轨迹成功率、timeout、protective stop 次数。

规划链：

- 相同 start/goal/scene 下规划成功率。
- 规划耗时 P50/P95/P99。
- 路径长度、最小 joint margin、Cartesian fraction。
- 轨迹点数量和时间参数化结果。

感知链：

- 相机输入 FPS、输出 FPS、丢帧率。
- 每阶段 P50/P95 延迟。
- PointCloud2 消息大小和进程间复制次数。
- CPU、RSS、GPU 使用率。
- 固定数据集上的体素/检测/尺寸结果差异。

业务链：

- 单箱和多箱完整周期时间。
- 检测、规划、抓取、放置成功率。
- 失败恢复时间。
- 运行 1 小时后的内存和队列增长。

### 11.2 等价判定

在未获得真实 ROS 1 测量前，不人为声明最终频率。建议初始验收规则：

- 安全相关结果和成功率不得下降。
- 控制周期 P99 不超过规定周期，最大抖动必须在 Huayan SDK 容许范围内。
- 规划 P95 不超过 ROS 1 基线的 110%。
- 感知输出 FPS 不低于 ROS 1 的 95%，P95 延迟不高于基线的 110%。
- 业务完整周期 P95 不高于基线的 110%。
- CPU/RSS 出现超过 20% 的回退必须定位和解释。

这些百分比是首轮工程阈值，真实硬件要求应覆盖它们。

## 12. 测试策略

测试仅针对 ROS 2 新实现：

- 单元测试：纯算法、消息转换、关节映射、单位转换、错误码映射。
- 组件测试：Xacro/URDF、pluginlib 加载、controller lifecycle。
- Launch test：MVP 启动、Action 可用、MoveIt 规划执行。
- Replay test：用固定 rosbag2/dataset 验证感知输出。
- Hardware-in-the-loop：只读、低速、小幅度、完整轨迹分级验证。
- Soak test：mock/Gazebo/真机长时间运行，检查内存、线程、队列和断线恢复。

不复用已删除的 ROS 1 rostest；迁移后的行为需要新的 ROS 2 `ament`/launch testing。

## 13. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|---|---|---|
| Huayan SDK 不支持 Jammy 或 ABI 不兼容 | 真机后端阻塞 | MVP 先用标准 mock 接口；尽早完成 Gate 8A |
| Humble 于 2027-05 EOL | 长期维护窗口短 | MVP 固定 Humble；接口和包边界保持可迁移到 Jazzy/Lyrical |
| `moveit_commander` 无法等价替换 | 运动规划重写量大 | 使用官方 C++ `MoveGroupInterface`，保留纯算法 |
| 全局参数状态无法直接迁移 | 节点间状态不一致 | 按第 9 节分类为 topic/service/action/私有状态 |
| Python 点云复制和 GIL | 吞吐下降 | C++ component、intra-process、depth 1 QoS |
| QoS 不兼容导致无数据 | 隐蔽运行故障 | 为每条接口固定 QoS 合同并添加启动诊断 |
| 固定 sleep 导致启动竞争 | 启动偶发失败 | lifecycle、controller spawner、就绪服务和事件驱动启动 |
| Gazebo Classic 行为不一致 | 仿真结果漂移 | Fortress 独立校准；控制接口和业务验收与仿真后端解耦 |
| 旧 ROS 1 代码同时参与构建 | 构建污染 | MVP 阶段使用 `COLCON_IGNORE`，按阶段删除 |
| 物理 mesh 碰撞模型过重 | MoveIt/Gazebo 性能下降 | 保留 visual mesh，后续生成简化 collision mesh 并验证几何误差 |

## 14. 回退与提交边界

- 所有修改仅发生在 `elfin_humble_ws`。
- 每个 Phase 独立提交，禁止把多个未验收阶段合并为一个大提交。
- 新 ROS 2 包先并行存在；对应 Gate 通过后才删除 humble 工作区中的 ROS 1 版本。
- `elfin_noetic_ws` 始终保持只读参考，不修改、不删除。
- Huayan SDK 真机阶段必须保留 mock backend，以便区分上层回归和硬件问题。
- Gazebo backend 与真实硬件 backend 通过相同 `ros2_control` interface 切换，不在业务代码中加仿真分支。

推荐提交顺序：

1. `ros2: add S20 description package`
2. `ros2: add mock ros2_control stack`
3. `ros2: add S20 MoveIt 2 config`
4. `ros2: add deterministic MVP demo and launch test`
5. `ros2: port luggage interfaces`
6. 后续按 Phase 独立提交

## 15. 决策记录

已确定：

- ROS 2 MVP 目标发行版为 Humble。
- MVP 机型仅 S20。
- MVP 使用 mock hardware，不等待 Huayan SDK。
- 控制协议统一为 `FollowJointTrajectory`。
- 真实硬件使用 C++ `ros2_control SystemInterface`。
- MoveIt 2 核心 adapter 使用 C++。
- Gazebo 不进入 MVP，后续产品仿真使用 Fortress + GPU。
- MuJoCo 作为同接口对照后端；Isaac Sim/Lab 不替代 Phase 1–6。
- 交互式仿真禁止 `llvmpipe`。
- 旧 ROS 1 包在等价实现验收前隔离但不删除。

进入 Huayan SDK 阶段前必须确认：

- SDK zip 的准确路径和版本。
- 目标控制计算机架构。
- SDK 支持的操作系统、glibc/GCC ABI。
- 厂商规定的控制周期和线程模型。
- 急停、使能、清错、断线和重连 API。
- 关节单位、方向、顺序和零位定义。

## 16. 官方参考

- [ROS 2 Humble 支持平台和发行信息](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
- [ROS 2 发行版 EOL 列表](https://docs.ros.org/en/humble/Releases.html)
- [ROS 2 QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS 2 参数模型](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html)
- [ROS 2 高效进程内通信](https://docs.ros.org/en/humble/Tutorials/Demos/Intra-Process-Communication.html)
- [MoveIt 2 Humble MoveGroupInterface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [ros2_control 硬件组件](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- [ros2_control 六轴机器人示例](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)
- [JointTrajectoryController 参数](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)
- [Gazebo 与 ROS 版本兼容矩阵](https://gazebosim.org/docs/jetty/ros_installation/)
- [Gazebo Classic ROS 2 包迁移到 Fortress](https://gazebosim.org/docs/fortress/migrating_gazebo_classic_ros2_packages/)
- [gz_ros2_control Humble](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)

