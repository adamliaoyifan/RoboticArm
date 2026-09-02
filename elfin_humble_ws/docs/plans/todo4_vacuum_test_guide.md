# Todo 4 真抓取测试指南（用户自测）

日期：2026-09-01。实现已完成、算法层单测 189/189 过，**未做仿真验收**——按约定
由你执行本指南并记录。结果模板在文末。

## 交付物速览

| 组件 | 位置 | 职责 |
|---|---|---|
| VacuumGate | `luggage_planning/luggage_planning/vacuum_gate.py` | 吸附合法性：接触/倾角/保持裕度（复用 vacuum_attach_utils + vacuum_retention + downward_constraint_utils） |
| VacuumBackend | `luggage_planning/luggage_planning/vacuum_backend.py` | ABC + SimBackend（gz 跟随+场景绑定）+ StubBackend；HardwareBackend 留接口 |
| PlanningSceneClient | `luggage_planning/luggage_planning/planning_scene_client.py` | 箱子三档：add（碰撞体）/ attach（ACM 绑定）/ detach+remove |
| vacuum_controller_node | `luggage_planning/scripts/vacuum_controller_node.py` | `/vacuum/command` 服务 + `/vacuum/state`（transient-local）+ `/vacuum/events_json` |
| VacuumState.msg | `luggage_msgs/msg/VacuumState.msg` | attached / vacuum_on / fail_reason / contact_distance / retention_margin / tilt_deg |
| launch | `sim_world.launch.py` 新参 `use_vacuum`（默认 false） | 起 vacuum_controller（backend 参数 sim/stub） |

## 启动

```bash
cd ~/work/elfin_humble_ws && source install/setup.bash
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true \
  visual_kind:=mesh sequence_ids:=standard observe_pose_name:=pickup_observe
```

就绪标志：日志出现 `vacuum_controller ready (backend=sim, panel=suction_contact_frame)`，
且 move_group 打出 "You can start planning now!"。

> T2–T5 需要臂先到 attach 位置。最省事的办法是跑一次评测驱动
> `python3 src/luggage_gazebo/scripts/todo3_pick_driver.py`（四段执行后臂停在
> retreat 高度），或用 `scripts/goto_joints.py` 手动。到 attach 的关节角可从
> driver 日志取。

## T1 箱子入 PlanningScene

前提：需要先调一次 add（当前由 eval driver / 手动触发）。手动做法：

```bash
# 终端 2
ros2 service call /pickup_box_spawner/spawn_next_box luggage_msgs/srv/SpawnNextBox
ros2 service call /luggage_detector/detect_luggage luggage_msgs/srv/DetectLuggage
# 手动加碰撞体（用检测输出的 pose/size 填 xyz / dims）：
# 目前 add_pickup_box 在 PlanningSceneClient 内，无独立服务面；
# 验收走 eval driver 时自动触发。手动验证用 RViz: PlanningScene 显示
ros2 topic echo /monitored_planning_scene --once | grep -c pickup_box
```

指标：
- `pickup_box` CollisionObject 存在、尺寸与 GT 差 ≤ 1cm
- approach 段 GetCartesianPath fraction 不因箱子降为 0（避障可绕行）

## T2 attach 吸附（信号契约）

臂到 attach 位后：

```bash
ros2 service call /vacuum/command luggage_msgs/srv/VacuumCommand "{enable: true}"
ros2 topic echo /vacuum/state --once
```

指标（全部满足才算过）：
- response `success=true`
- state：`attached=true`、`vacuum_on=true`、`fail_reason=""`
- `contact_distance` ≤ 箱半对角+0.08（约 0.47）
- `retention_margin` ≥ 2.0
- `tilt_deg` ≤ 5

失败码语义（message 前缀）：`VACUUM_NO_BOX` / `VACUUM_NOT_IN_CONTACT` /
`VACUUM_TILT_EXCEEDED` / `VACUUM_RETENTION_MARGIN` / `VACUUM_BACKEND_ERROR`。

## T3 箱子跟着走（核心证据）

attach 成功后执行 retreat 段（driver 或 FJT 到 retreat 高度），前后各读一次：

```bash
timeout 5 ign topic -e -t /world/airport_loading/pose/info --num 1 \
  | grep -A6 pickup_box
```

指标：
- 箱子 ΔZ = 0.35 ± 0.05 m（retreat clearance）
- 箱子 XY 漂移 < 0.05 m
- 跟随期间 `/vacuum/state.fail_reason` 恒为空
- RViz 中 box 随 panel 一起动（PlanningScene attach 可视）

## T4 释放（物理接管）

```bash
ros2 service call /vacuum/command luggage_msgs/srv/VacuumCommand "{enable: false}"
sleep 2
timeout 5 ign topic -e -t /world/airport_loading/pose/info --num 1 | grep -A6 pickup_box
```

指标：箱子 Z 回落到 0.86 + height/2 ± 0.02（落在平台上），之后手动动臂箱子不动；
PlanningScene 中 pickup_box 已移除（`echo /monitored_planning_scene` 查无）。

## T5 误吸附防护

臂在 observe（远离箱子）：

```bash
ros2 service call /vacuum/command luggage_msgs/srv/VacuumCommand "{enable: true}"
```

指标：`success=false`，message 前缀 `VACUUM_NOT_IN_CONTACT`，
`/vacuum/state.attached` 保持 false，gz 中箱子未被 set_pose（位置不变）。

## T6 端到端率对比（可选）

```bash
python3 src/luggage_gazebo/scripts/pick_retreat_eval_driver.py --trials 10
```

指标：端到端 ≥ 55%（8/28 基线，detection_gt_gate.md）；若驱动已接 vacuum
步骤，新增 vac_attach / vac_follow 列全绿。

## 记录模板（写 docs/status/todo4_vacuum_pick.md）

```
# Todo 4 真抓取验收
日期 / commit / ROS_DOMAIN_ID / 栈命令
T1 …（命令输出摘录 + 判定 ✓/✗）
T2 …
T3 …（ΔZ 实测值）
T4 …
T5 …
T6 三率表（如跑）
已知偏差
```

## 已知实现限制（测试时留意）

- follow 用 TF(suction_contact_frame) 30Hz → gz set_pose；箱子动力学被
  kinematic 覆盖（这是设计：运动学跟随，同 ROS 1 gazebo_follow 模式）
- `/luggage/current_box` 由 spawner 发布，ClearCurrentBox 后 vacuum 的 box
  引用不清空——detach 后 enable 会报 VACUUM_NOT_IN_CONTACT（可接受）
- PlanningScene attach 用固定 object id `pickup_box`；同一时刻只支持一只箱
- `keep_camera_down` / `lock_wrist` 仍未实现（Todo 3 遗留）
