# 同运行认证—提交离线追踪（2026-10-20）

范围：基于 `bb238bb7`，零仿真、零重放、零产品行为改动。本报告取代
`HOLD_DIAG_STAGE2C.md` 中跨运行重定时、feedback 回声及成功后 reference
爬行的归因。只比较同运行、同腿；r10 与 2c 分栏，不互补缺失字段。

## 结论与证据等级

1. **直接观察，r10**：同运行 cert_request_1/2 首腿（同 edge、同 route
   digest、相同 JointTrajectory 字段）与 driver 保存的 transit 均为 231 点。
   全部 q/v 逐值相等；全部时间差 ≤4.98e-7s，属于 driver 六位小数舍入。
   认证末点为 120.815863787s，driver 为 120.815864s，执行窗口为
   120.933s。未观察到这两边界之间的重定时。
2. **直接观察，2c**：driver 自己保存了 257 点 transit，span
   111.137935s、末点六关节速度全零，执行窗口 111.239s。其认证输入未保存，
   所以不能验证该 run 的认证—提交对应关系；与 r10 的 120.82s 无可比性。
   保存的是 driver 传入 PlanMotionV2 的段，**不是实际 FJT goal**。
3. **直接观察，2c**：driver 的 controller_state 使用消息 header stamp。
   窗口 284.89–286.55s，PlanMotionV2 返回 stamp=284.971s；返回后的
   79 个样本 reference 只有一个值，reference==output，离 driver 末点
   最大误差 3.82e-7 rad（记录六位小数）。该窗口内 reference 已冻结。
4. **watcher 时钟混用，2c**：其 `_on_cs` 未保存消息 header，`sim` 是
   另一个回调更新的缓存 `/clock`，`t` 是处理回调时的墙钟。最后一条
   feedback 六关节值与 driver joint ring 的 stamp=284.23s 样本**逐位相等**，
   却标为 sim=287.833s；另一样本标为 284.944s，逐位匹配 281.17s。
   两例表观错位分别 3.603/3.774s。结合源码和已观察到的回调饥饿，支持
   watcher 把较旧数据标上较新缓存时刻的解释；不是精确传输延迟测量。
   **撤回“成功后 reference 爬行”和“feedback 是 open-loop 内部回声”。**
5. **保留的物理观察**：`/joint_states` 的 j1/j2 在腿末减速段开始异常，
   早于 handoff；j3–j6 停下。指令、执行上下文、底层接口机制仍未分离。
   本轮未重新估计漂移起点，也未证明控制器获得了与认证相同的全部导数字段。

`result.json` 给出逐点比较、输入哈希、末点、样本数和源码对照。

## 同运行边界表

| 边界 | r10 首条生产 transit | 2c 生产 transit | 可核对字段与缺口 |
|---|---|---|---|
| 认证输入 | cert_request_1/2，edge `e-portal_transit-4ffb03af-98b56568`；route digest `4d28d93a3d509c4bb83697acc3be762c706872b54cfba7e59d95d79096e660f1` | 缺失 | r10 CDR 含关节、q/v/a、整数 ns、空 frame/零 header stamp；认证响应同 index 均 all_valid=true |
| Select 返回腿 | 完整响应未捕获；请求已捕获 | 请求/响应未捕获 | r10 请求为 `place-v2-2`；首点与 start_joints 相同。不能用请求代替响应 |
| driver → PlanMotionV2 | 231 点，q/v 与认证逐值相同，t 仅舍入 | 257 点，首末零速、末点111.137935s | 顺序均 elfin_joint1…6；driver dump **不保存 a/effort/header**，t 精度为1µs。源码“lossless submitted”注释夸大捕获范围 |
| executor → ExecuteTrajectory | 实际 goal 缺失；有条件离线重建 | 实际 goal 缺失 | `timed joint path` 返回消息支持分支识别；不能凭结果消息补齐实际提交字段 |
| MoveGroup 执行分发 | 实际输入/输出未落盘 | 同左 | 源码解释可能转换；运行时参数未全捕获 |
| FJT goal / result / cancel | 精确 goal/关联事件未捕获 | 精确 goal/关联事件未捕获 | 不能签署字段相等，不能排除其他 goal/cancel/切换 |

r10 的 cert_request_0 是另一候选（85 点、8.846179981s），不得误认作
实际生产 transit。cert_request.json 是“最后一次请求”的覆盖别名；本轮
按带 index 的文件逐个匹配，不按文件名猜对应关系。

## 源码机制与转换位置

- **认证之前**：`moveit_edge_adapter.py:1260` 的 `_retime` 可按
  `ST09_JOINT_KNOT_DENSIFY`（默认启用）加密几何 knots，再经
  `trajectory_timing.py:55` 做 rest-to-rest 计时和样条导数界统一时间拉伸。
  `t *= scale, v /= scale, a /= scale²` 保留相同插值路径的时间伸缩关系；
  该边界会改变时间/导数，也可能增加几何采样点，不能把规划器原始输出
  当认证输入。`route_producer.py:1685` 认证这些 edge 的最终轨迹，成功后
  `:1817` 才生成返回腿；失败重建的 edge 重新认证。此次不从缺失的规划
  中间层反推每个 knot 的构造来源，也未证明独立的认证算法正确性。
- **认证 → Select**：`route_producer._leg_record:3249` 将 edge 的 names、
  q/v/a、整数 ns 逐项写入 typed leg，effort=[]；metadata stamp 取 Select
  start_tcp 的 stamp。`place_service._route_motion_messages:216` 编码，
  `motion_frame_contracts:222` 会按关节名重排所有逐关节数组到 canonical
  顺序，不重采样/重计时。typed metadata stamp 与 ROS trajectory header
  不是一回事；`ros_frame_bindings/joints.py` 保持后者默认零值。
- **Select → driver → executor**：`place_smoke_driver.py:1539,3445` 直接
  保留 motion_segments 的对象列表，按顺序/名字配对；没有重新拼接轨迹。
  `typed_driver_adapter.py:716` 直接赋 `goal.segment`。driver 手中的 v1
  carry_segments 是记账视图，typed 执行取 `_frame_motion_segments`。
- **executor timed 分支**：`motion_executor.py:781` 直接进入
  `_execute_timed_joint_v2:640`；`timed_trajectory` 复制 q/v/a/effort/整数 ns。
  可能按提交时 live joints 对每关节整体减去整数个2π，保留导数/时间；
  随后做 continuity/start-state、配置存在时做 occupancy FK sweep。
  它没有执行一次完整的 post-shift 认证；旋转等价本身不能代替有界关节的
  限位证明。本轮未证明 r10/2c 实际触发平移，也未发现由此引发的缺陷。
  `validate_joint_path` 的返回加密点未用于替换 `points`。新建 RobotTrajectory
  的 header 为零/空；不调用 `:1196` 的 legacy `_retime_robot_trajectory`。
  legacy 分支确有重定时，但不能仅凭函数存在认为生产 typed 腿走了它。
- **MoveGroup → FJT**：执行的是 `/execute_trajectory` action，无新
  MoveGroup 规划请求。MoveIt 2.5.9 的 TEM `distributeTrajectory` 按控制器
  拆分/重排关节；保留点数、q、a、时间及 header；v 乘
  `execution_velocity_scaling_`（默认1.0，运行时值未保存）。FJT handle
  把分发后的 joint_trajectory 放入 goal，并带控制器容差配置。这里没有
  观察到实际输出，不宣称完整链路恒等。若 v scaling 非1而时间/a 未相应
  变化，插值导数/路径可能改变；未发现这一层触发项目完整再认证的代码，
  但尚无本次实际启用该转换的证据。

转换存在不等于缺陷。已实测的 r10 认证—driver q/v/t 没有变化；未捕获的
a/header/下游实际输出无法判定。不能依据这些缺口签署路径或导数约束被破坏。

## reference / output / feedback 来源

当前安装包：JTC 2.53.3，control_msgs 4.9.0，MoveIt simple controller
manager 2.5.9。仓库配置为 position command、position/velocity state、
open_loop_control=true、state_publish_rate=50Hz、controller update=100Hz、
末端非零速允许、stopped_velocity_tolerance=0。配置文件与 r10 记录 revision
中的文件相同；**不等于捕获了当时运行时所有参数/二进制哈希**。

JTC 对应 tag 源码：每次 update 从硬件 state interface 读取 current；
reference 取采样后的 desired；feedback 取 current；output 从 command
interface 读取。open_loop 选择新轨迹的起始插值状态为 last_commanded，
不会因此把发布的 feedback 替换成内部 reference。这些通道均不提供
Gazebo 关节力矩证据。driver 保存 header stamp，watcher 丢掉了它。

原始版本源码及 URL 在 `upstream/`；这些是当前安装版本的上游语义说明，
不是 r10 当时二进制/有效参数的完整复原。

## 有条件重建与重放准入

`analyze.py` 使用 r10 同运行认证 CDR + Select 请求的 identity，实际调用
`_leg_record → _route_motion_messages → record_from_message → execute_segment_v2`，
以离线 `_timed_submit` hook 截取 RobotTrajectory；不建立 ROS node/客户端。
假设 live 起点=第一 knot（零整周平移），省略缺失的 occupancy 执行上下文。
所得 JointTrajectory 含全部 q/v/a/effort/ns/header，与认证轨迹全部字段 **完全相等**。
额外检查一关节整体+2π条件：每个位置点平移2π，v/a/t保持不变。保存
`r10_conditional_reconstruction.{json,cdr}`，明确为条件重建，非捕获实际 goal。

缺少提交时精确 live joints、完整有效运行参数、Select 响应及两层实际 goal；
r10 manifest 为 `5a80e9b4`、dirty_lines=1，未保存那一项 dirty 的完整内容。
相关当前源码与该 revision 的受检文件相同（见 result），仍不能消除运行时
部署/dirty provenance 缺口。因此**尚不满足唯一重建当时实际提交/FJT轨迹的
准入条件**。本轮不冻结第2步。优先补同运行边界捕获，再讨论候选与预算。

需补的最小边界内容：同一 run/box/leg/request/route/cert/action goal UUID；
认证输入与 verdict；Select 响应；PlanMotionV2 输入；executor wrap 前后轨迹、
live joints/shift向量；ExecuteTrajectory/FJT 完整 goal/result/cancel；有效参数、
源码与安装文件哈希、dirty diff。所有轨迹保留 names、q/v/a/effort、整数 ns、
header，生成完整字段摘要；观测保存消息源 stamp 与本地接收 mono，不能用
缓存 /clock 给其他消息定时。此为缺口清单，不是仿真/重放执行授权。

未来即使隔离复现，也只证明所测轨迹+条件足以触发；未复现只证明所测条件
未触发，均不能自动签署转换层缺陷或上下文根因。

## 复核命令与限制

```bash
source /opt/ros/humble/setup.bash
source /home/adamliao/work/elfin_humble_ws/install/setup.bash
source /tmp/elfin-master-pack-baseline-g3/install/setup.bash
/usr/bin/python3 docs/status/evidence/occ_owner_b1/gate3_aprime/offline_trace_20261020/analyze.py
```

结果：逐点比较及真实构造/执行分支的离线断言通过。未运行控制器、物理仿真、
碰撞再认证或端到端成功率测试。2c 必要失败输入已压缩保存在 `inputs_2c/`，
含原路径/哈希清单；r10 引用主工作区既有证据，未从其他 run 填补缺项。

上游链接：
[JTC 2.53.3](https://github.com/ros-controls/ros2_controllers/blob/2.53.3/joint_trajectory_controller/src/joint_trajectory_controller.cpp)，
[TEM 2.5.9](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp)，
[FJT handle 2.5.9](https://github.com/moveit/moveit2/blob/2.5.9/moveit_plugins/moveit_simple_controller_manager/src/follow_joint_trajectory_controller_handle.cpp)。

后续输入完整性复核：见 [2c重放草案](REPLAY_DRAFT.md) 与
`2c_field_audit.json`。257点全部缺少加速度，当前未通过完整输入准入门。
