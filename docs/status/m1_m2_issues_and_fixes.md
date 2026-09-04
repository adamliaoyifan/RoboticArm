# M1/M2 验收实验问题与修复记录（evidence log）

日期：2026-08-19 ~ 08-20
范围：仿真闭环移植 M1（Fortress 后端切换，已验收：
[phase7a_gz_backend.md](phase7a_gz_backend.md)）与 M2（感知最小链，进行中：
[m2_perception_occlusion_problem.md](m2_perception_occlusion_problem.md)）。
原始数据：`evidence/m1_acceptance/`、`evidence/m2_occlusion/`。

本文记录验收过程中实际踩到的问题、现象、根因与修复，供后续阶段（M3/M4、Phase 5/6/9）
避免重蹈。

## 一、M1：Fortress 后端（均已修复，验收通过）

### 1. `model://sun` / `model://ground_plane` 解析失败 -> gz server 直接退出

- 现象：`Error Code 13: Unable to find uri [model://sun]`，Server 退出，
  `on_exit_shutdown` 连锁杀掉整个 launch。
- 根因：自建 world 用 `model://` include 依赖 gz 示例模型路径。
- 修复：sun/ground_plane 内联进 `airport_loading.sdf`（directional light + plane）。

### 2. `package://` mesh URI 在 gz 内全部解析失败

- 现象：`Could not resolve file [model://luggage_gazebo/models/suction_panel/...STL]`。
- 根因：libsdformat 在 URDF->SDF 转换时把 `package://<pkg>/...` 改写为
  `model://<pkg>/...`；gz 按 `GZ_SIM_RESOURCE_PATH` 搜 `<path>/<pkg>/...`。
- 修复：launch 里把每个包的 `install/<pkg>/share` 加入 `GZ_SIM_RESOURCE_PATH`
  （`sim_world.launch.py::_resource_path`），包 env hook 亦导出。

### 3. gz Ros2Control 系统插件加载失败

- 现象：`Failed to get info for [ignition::gazebo::systems::Ros2Control] ...
  could not instantiate from library [ign_ros2_control-system]`。
- 根因：本机 `ros-humble-gz-ros2-control`（Fortress 版）注册的插件类名是
  `ign_ros2_control::IgnitionROS2ControlPlugin`；README 常见的
  `ignition::gazebo::systems::Ros2Control` 类名在本构建中不存在。
- 修复：xacro 里 `filename="ign_ros2_control-system"
  name="ign_ros2_control::IgnitionROS2ControlPlugin"`（类名从
  `strings libign_ros2_control-system.so` 确认）。

### 4. `/world/<w>/create` ROS service 生命周期

- 现象：自定义 rclpy spawner 等待 `/world/airport_loading/create` 服务超时。
- 根因：`ros_gz_sim create` 可执行文件自带临时 service bridge，进程退出后服务消失。
- 修复：场景模型改为每模型一个 `ros_gz_sim create` 节点；M2 起改为常驻
  `parameter_bridge` 服务桥（create/remove/set_pose），供 pickup_box_spawner 和
  M4 真空跟随使用。

### 5. `Future.result(timeout=...)` 非 Humble rclpy API

- 现象：`TypeError: Future.result() got an unexpected keyword argument 'timeout'`
  （MVP 验收时未暴露的潜伏 bug）。
- 修复：`send_joint_trajectory.py` 改为轮询 `future.done()`（`_wait_future`）。

### 6. `pkill -f "ign gazebo"` 自杀

- 现象：shell exit 144，后续命令全部没跑。
- 根因：pkill -f 匹配到包含该字符串的当前 shell 命令行。
- 修复：括号技巧 `pkill -f "ign gazeb[o]"`。

### 7. 工作区搬迁导致 colcon 缓存失效

- 现象：`CMakeCache.txt directory ... is different than ...`（路径含
  `RobotArm/elfin_humble_ws` 旧前缀）。
- 修复：清 build/install/log 全量重建。工作区**不是 git 仓库**，计划的
  "每 Phase 独立提交"目前无法执行（待用户决定是否 `git init`）。

M1 验收结果（`evidence/m1_acceptance/fjt_20repeat_fortress.txt`）：
Fortress FJT 20/20，worst_abs_err 0.002709 rad；mock 回归 3/3 @ 0.001 rad；
luggage_description 单测 77 passed。

## 二、M2：感知最小链（除最后一条外均已修复）

### 8. `sensor_msgs.point_cloud2` 在 ROS2 不存在

- 修复：改 `from sensor_msgs_py import point_cloud2 as pc2`。

### 9. `AttributeError: __enter__`（Node 构造崩溃）

- 现象：`super().__init__(...)` 内 `with self.handle:` 抛错，最小复现排除了
  双 rclpy 安装、依赖 import 等假设。
- 根因：我在 Node 子类里把 service handler 方法命名为 `handle`，
  遮蔽（shadow）了 `Node.handle` property。
- 修复：改名 `handle_detect`。教训：**rclpy Node 子类不要定义 `handle` 成员**。

### 10. 服务名解析：ROS2 相对名按 namespace 而非节点名解析

- 现象：`create_service(SpawnNextBox, "spawn_next_box")` 注册成 `/spawn_next_box`，
  客户端调 `/pickup_box_spawner/spawn_next_box` 永远等不到。
- 根因：ROS1 的 `~name` 语义（节点私有）在 ROS2 不存在；相对名挂在 namespace 上。
- 修复：所有跨节点服务用**绝对名** `/pickup_box_spawner/...`、`/luggage_detector/...`。

### 11. `Time` 类型混用

- 现象：`unsupported operand type(s) for -: 'Time' and 'Time'`。
- 根因：header stamp 是 `builtin_interfaces.msg.Time`，节点时钟是 `rclpy.time.Time`。
- 修复：`rclpy.time.Time.from_msg(stamp)` 后再做差/TF 查询。

### 12. sensor_msgs_py 返回结构化 numpy record 数组

- 现象：`Cannot cast array data from dtype({'names': ['x','y','z']...})`。
- 修复：`np.stack([arr['x'],arr['y'],arr['z']], axis=1)`。

### 13. 点云 frame 约定错位 + inf 未过滤（**当前主问题，已定位未修复**）

- 详见 [m2_perception_occlusion_problem.md](m2_perception_occlusion_problem.md)。
- 根因：gz `rgbd_camera` 按 sensor（camera_link，+X 前向）约定发点云，
  `gz_frame_id` 标签写的 optical frame 与数据实际约定不符；且远平面未命中产生 inf，
  `skip_nans` 不滤 inf。
- 教训：**gz 传感器消息的 frame_id 标签不保证与数据坐标约定一致，必须数值验证**
  （本次用"直下相机不应看到 z>相机高度 的点"这一几何不变量定位）。

### 14. QoS 不匹配导致收不到桥接传感器数据

- 现象：订阅 `/camera/depth/points` 无数据。
- 根因：parameter_bridge 发布端 best_effort，reliable 订阅不匹配。
- 修复：传感器流一律 `QoSProfile(depth=1, reliability=BEST_EFFORT)`（计划第 10 节契约）。

### 15. 诊断脚本挂死（4 个叠加原因）

- TransformListener(spin_thread=True) 与手动 spin_once 同节点死锁；
  `lookup_transform` 不带 timeout 无限阻塞；非 daemon 线程阻断退出；
  同上 QoS 不匹配。修复分别为：二选一 / 传 `timeout=Duration(...)` / `os._exit(0)` /
  BEST_EFFORT。

### 16. cv_bridge 与 NumPy 2.x 不兼容

- 现象：import cv_bridge 报 numpy 版本冲突。
- 修复：图像落盘用纯 python PNG 编码（struct+zlib）。后续如需 cv_bridge，
  需统一 NumPy 版本或用 `image_transport`/`ros_gz` 自带转换。

### 17. rclpy busy-loop CPU（**未根治，仅缓解**）

- 现象：全栈 launch 下 spawner ~27%、detector ~48% CPU；单独运行同节点 0%，
  use_sim_time/服务调用等因素逐一排除均未复现。
- 缓解：spawner 去掉 `use_sim_time`（注释：1 kHz /clock 订阅在 rclpy 节点里烧 CPU）。
- 遗留：未定位；服务响应正常，不阻塞 M2 验收，但 Phase 5 高吞吐链前必须复查
  （候选方向：rclpy spin 空转、/clock 高频订阅、executor 线程数）。

## 三、给 M3/M4 的注意事项

- 跨节点服务/action 一律绝对名；Node 子类避免 `handle` 命名。
- 消息时间戳参与运算前先 `Time.from_msg`；传感器流 BEST_EFFORT depth 1。
- 任何"标签 frame"先做几何不变量数值验证再用。
- inf/NaN 过滤是点云入口的固定步骤（noetic Classic 插件天然裁剪远平面，gz 不会）。
