# Todo 4 — 闭环评测驱动

对应 plan todo `eval-driver`。唯一允许做编排的节点：spawn → observe → detect →
比 GT → pick/retreat，重复 N 次，出报表。

## 范围

一个 rclpy 节点 + 一份报表。只通过统一 ROS 接口调用别人，
**禁止 import YOLO / RANSAC / MoveIt**。

**不做**：吸盘、place、多箱、orchestrator。

## 交付物

| 文件 | 性质 |
|---|---|
| `luggage_gazebo/scripts/pick_retreat_eval_driver.py` | 新 |
| `luggage_gazebo/luggage_gazebo/eval_metrics.py` | 新，无 ROS 的聚合逻辑 |
| `luggage_gazebo/test/test_eval_metrics.py` | 新 |
| `docs/status/closed_loop_eval.md` | 新，报表 |

放在 `luggage_gazebo` 的理由：它只在仿真里有意义，且要清理 gz 相关的残留进程。
`luggage_bringup` 是 ROS 1 且被 `COLCON_IGNORE`，不能用。若以后要在真机跑，
再抽出独立包。

---

## 1. 状态机

每轮（`trial`）：

```
1. ensure_clean          残留检查（见第 2 节）
2. SpawnNextBox          -> 记录 GT 尺寸
3. GoToRobotPose(pickup_observe)
4. wait_geometry_ok      订 /luggage/preprocessed/status
5. DetectLuggage         -> measured
6. GetCurrentBox         -> gt
7. DetectionAccuracy.compare
   不过门 -> 记 fail，skip 8-10，跳到 11
8. BuildMotionSequence(phase=pick)
9. for segment in segments: PlanMotion(segment)
10. verify_retreat       末端离开 attach 高度
11. ClearCurrentBox
```

每一步都要有独立超时和独立失败码，聚合时才能归因。**不要**用一个大 try/except
把整轮包住——那样只能得到"这轮失败了"，没有诊断价值。

### 第 4 步：等 `geometry_ok`

订 `/luggage/preprocessed/status`，QoS 必须是 **RELIABLE + TRANSIENT_LOCAL**
（预处理就是这么发的，QoS 不匹配会一条都收不到）。

判据：`flags.geometry_ok == true` 且 `motion_gate.state == "stable"`。

跳过这步测到的是运动模糊，不是检测精度。手臂到位后运动门控需要
`settle_time_sec`（默认 0.5 s）才会放行几何。

### 第 10 步：retreat 校验

`PlanMotion` 返回 success 只说明规划执行没报错。要额外确认末端真的抬起来了：
查 `suction_contact_frame` 在 world 系的 Z，和 attach 段的目标 Z 比。
差值应接近 `retreat_clearance - attach_clearance`（默认 0.35 - 0.0）。

---

## 2. 残留清理 —— 实测教训

上一轮验收在这里翻过车，**这是本 todo 最容易低估的一块**。

`ros2 launch` 被 SIGTERM 之后，下列进程会**存活**：

- `ign gazebo`（launch 通过 `/bin/sh -c ruby /usr/bin/ign gazebo ...` 起的，
  杀 launch 杀不到孙子进程）
- `parameter_bridge` ×3（clock / camera / world service）
- `robot_state_publisher`

后果：下次启动出现同名双节点，`ros2 node list` 报
`nodes in the graph that share an exact name`，两套 camera bridge 同时喂
`/camera/depth/points`，深度数据互相污染，检测结果变得不可复现。

### 可靠做法

**首选：给评测跑一个独立 `ROS_DOMAIN_ID`**（上一轮用 42）。比逐个杀进程稳得多，
也不会误伤用户手上正在看的那套仿真。驱动启动时读环境变量并在报表里记下来。

**兜底清理**（真要杀时）：

1. SIGTERM launch 的 python 进程（`/opt/ros/humble/bin/ros2 launch ...`）
2. 等 3–4 s
3. 按进程名兜底清 `ign gazebo` / `parameter_bridge` / `robot_state_publisher`
4. 再等约 8 s 让 DDS 幽灵节点过期，然后才判断 `ros2 node list` 是否干净

**`pkill -f "ign gazebo"` 会匹配到自己的 shell 命令行导致自杀。**
用 `ign gazeb[o]` 或按 PID。

### 和「自己手动停仿真」的关系

三条不要混：

1. **残留会污染深度——手动停也成立。** `Ctrl+C` / 给 `ros2 launch` 发 SIGTERM
   只保证 launch 那个 Python 退出。`ign gazebo`、`parameter_bridge`、
   `robot_state_publisher` 常留下。停掉后若还要再开，先确认旧进程没了，
   不要默认「终端停了世界就干净了」。
2. **独立 `ROS_DOMAIN_ID` 不是日常停仿真的义务。** 那是评测驱动和正在看的
   那套仿真隔离用的（评测用 42，交互用默认域）。只开一套自己玩，继续默认域。
3. **`pkill -f "ign gazebo"` 自杀只在用这条命令清场时。** 日常 Ctrl+C、关终端、
   按 PID 杀不会触发。

手动停的实用顺序：launch 终端 Ctrl+C → 等几秒 → `ros2 node list` /
`pgrep -af 'ign gazebo|parameter_bridge|robot_state_publisher'` → 有残留再按
名字或 PID 清 → 再等几秒让 DDS 幽灵过期，然后才重新 launch。

---

## 3. 时序预算

| 环节 | 实测/预期 | 超时建议 |
|---|---|---|
| 预处理输出帧率 | **4–6 Hz**（raw 相机 30 Hz） | — |
| 等 `geometry_ok` | 手臂停稳 + 0.5 s settle | 10 s |
| YOLO 推理（CPU） | 远慢于 4 Hz | mask 等待 15 s |
| `DetectLuggage` | 服务内同步算 | 10 s |
| 单段 `PlanMotion` | OMPL 规划 + 执行 | 60 s |
| `move_group` 启动 | 慢 | 就绪等待 60 s |

预处理只有 4–6 Hz 的原因是全分辨率点云在 Python 里解码 + 变换（每帧约 27 万
有效点，另丢约 3.5 万 inf）。**别按 30 Hz 假设"下一帧马上到"**。

一轮完整 trial 乐观估计 30–60 s，N=20 要留出 20 分钟以上。驱动要能中途
Ctrl-C 并把已完成的 trial 写出去，不要全部丢失。

---

## 4. 实现细节

### 解析点云用现成适配器

需要读点云时（例如统计 cargo 点数）用
`luggage_perception.ros_message_adapters.cloud_points_from_msg`，
**不要在驱动里再手写一遍 `PointCloud2` 解包**。字段 offset、`point_step` 内的
padding、字节序这些坑已经在那边处理过并有单测覆盖。

这条会引入 `luggage_gazebo` → `luggage_perception` 的依赖，`package.xml` 要加
（`luggage_gazebo` 已经因为 preprocessor 加过这个 exec_depend）。

### 服务与 action 名字

全部用**绝对名**。ROS 2 没有 ROS 1 的 `~name` 私有语义，相对名挂在 namespace 上，
这个坑在 `m1_m2_issues_and_fixes.md` 里已经踩过一次：

- `/pickup_box_spawner/spawn_next_box`
- `/pickup_box_spawner/get_current_box`
- `/pickup_box_spawner/clear_current_box`
- `/luggage_detector/detect_luggage`
- `/waypoint_generator/build_motion_sequence`（新，注意和实现对齐）
- `/motion_planner/plan_motion`、`/motion_planner/go_to_robot_pose`（新）

### rclpy 细节

- 服务/action 客户端用 `ReentrantCallbackGroup` + `MultiThreadedExecutor`，
  否则在服务回调里等另一个服务会死锁
- `TransformListener(spin_thread=True)` 与手动 `spin_once` 同节点会死锁
  （`m1_m2_issues_and_fixes.md` 第 15 条）。二选一。
- `Future.result(timeout=...)` **不是 Humble 的 API**，轮询 `future.done()`
- header stamp 是 `builtin_interfaces.msg.Time`，节点时钟是 `rclpy.time.Time`，
  相减前先 `rclpy.time.Time.from_msg()`
- `use_sim_time: True`

### 别定义 `handle` 成员

rclpy `Node` 有 `handle` property，子类里定义同名方法会让 `super().__init__()`
里的 `with self.handle:` 抛 `AttributeError: __enter__`，报错信息完全指不到真因。
已经踩过一次。

---

## 5. 指标聚合（`eval_metrics.py`，无 ROS）

驱动只负责调接口和收数据，统计逻辑放无 ROS 的类里，可单测。

```python
@dataclass(frozen=True)
class TrialRecord:
    index: int
    gt: BoxObservation | None
    measured: BoxObservation | None
    accuracy: AccuracyResult | None
    detect_failure: str          # 检测器 _last_failure_reason
    segments_planned: int
    segments_succeeded: int
    segment_failures: tuple      # (name, message) 列表
    retreat_delta_z: float | None
    wall_time_sec: float
```

三个**独立**指标，不要混：

1. **检测过门率** = `accuracy.ok` 的比例
2. **规划成功率** = 过门的 trial 里四段全 success 的比例
3. **执行正确率** = retreat 高度校验通过的比例

混在一起会得到一个既不能归因也不能对比的数。

`summarize()` 输出：三个率、检测误差 P50/P95（复用
`DetectionAccuracy.summarize`）、失败码直方图、per-segment 失败次数。

---

## 6. 报表

写 `docs/status/closed_loop_eval.md`。必须包含：

- 日期、commit / 工作区状态、`ROS_DOMAIN_ID`
- 配置：`use_semantic`、backend 实际值（`yolo_world:...` 还是
  `stub(fallback:...)`）、位姿名、N、各门槛数值（todo 2 初值，或
  `docs/status/detection_gt_gate.md` 里 P95 回填后的值，报表里写明是哪套）
- 三个率 + 检测误差分位数
- 失败码分布与 per-segment 失败
- 每轮原始数据（表格或附 CSV）
- 已知偏差与未覆盖项

**backend 实际值必须记进报表。** 如果 ultralytics 缺失导致静默退回 stub，
整份报表就毫无意义——todo 1 里的 `require_backend` 参数就是为了在启动时
拦住这种情况，报表里再记一次做双保险。

报表写完把索引挂到 `docs/status/README.md`。

---

## 7. 测试

`test_eval_metrics.py`（纯 python）：

- 全成功 → 三个率都是 1.0
- 检测不过门的 trial 不计入规划成功率的分母
- 失败码直方图统计正确
- 空记录列表不除零
- 中断（部分 trial）也能 summarize

驱动本身不写单测，它就是测试工具。可以加一个 `--dry-run`：走完状态机但不真的
调服务，用来验证超时和报表格式。

## 8. 验收

- N=20 跑完不中断，报表生成
- 中途 Ctrl-C 能写出已完成部分
- 连续跑两次，检测误差 P50 差异在噪声范围内（可复现）
- 故意不起 segmenter：驱动干净失败并在报表里标 `DETECT_NO_CLOUD`，
  不是挂死或静默通过
- 故意留一套残留 bridge：驱动能检测到并拒绝开始，而不是产出污染数据
