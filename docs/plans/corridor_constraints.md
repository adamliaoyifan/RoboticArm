# 走廊约束架构（多箱 place 的路径与约束分层）

状态：**规范**（normative）。来自 2026-09-02 的设计决定，覆盖 Todo 5 切片 B–D
及后续。与 [perception_architecture.md](../architecture/perception_architecture.md)
同级别：与它冲突的改动是缺陷，不是风格偏好。

## 核心不变量

**进和出是同一条走廊。** 箱内不绕行、不穿堆。放不进走廊的槽位在
ComputePlacement 就被否掉，而不是让手臂在行李堆里搜三维路径。

- 走廊 = 沿开口轴（本场景 `-X`）的 AABB，从门洞扫到槽位近端面
- `insert` / `descend` 只在槽位正上方柱内下降；`retreat` / `extract`
  只在不低于 retreat 高度的平面上沿开口法向水平出箱
- 空载退出不另建约束：能沿走廊进去，同一走廊、不低于进箱高度就能出来。
  若退出高度被邻箱挡住，说明**进箱时走廊高度就该更高**，或这个槽不该选
- 笛卡尔 fraction < 0.95 → **换下一个 SlotSpec**，禁止 OMPL 回退在箱间钻

## 三层约束

| 问题 | 约束在哪 | 失败时 |
|---|---|---|
| 放哪 | 槽位选择（ComputePlacement） | BIN_FULL / 下一候选 |
| 怎么进、怎么出 | 开口轴走廊 + 沿程 max 高度 + 载荷膨胀 | 换槽，不绕行 |
| 臂会不会撞 | MoveIt 场景核对（placed_* + mesh + FCL） | 换槽 |

### 第 1 层：槽位（ComputePlacement，已有雏形）

在 2.5D `surface_2d` 上滑足迹，全部通过才算可行：

1. 足迹不与已 commit 的 AABB 重叠
2. 支撑判定：未知柱子不能当堆叠面（`unknown_above_floor`，placement_solver L159）
3. 头顶到箱顶间隙（`clearance_top` ≥ margin，L143/167）
4. 足迹落在开口投影内（`outside_aperture`）——箱子过不了门的位置直接否
5. 开口到该槽的走廊不被封死（`insertion_corridor.corridor_blocked`，已有；
   当前 P2 简化只检单箱全宽墙，多箱墙后补）
6. 这一放不能把深处空间堵死（`blocks_deep_space`，已有）

被否候选标 infeasible 原因，换下一个。**不搜三维绕行。**

### 第 2 层：进箱/退出路径（几何，不必 OMPL）

路点模板不变：`portal → traverse → insert → descend → retreat → extract → observe`。
多箱时只改两个量：

- **高度** = `max(沿走廊表面高度) + 载荷半高 + 余量`。
  不能只用本槽上方 `place_clearance_z`（现 0.15 m）——旁边有更高的箱子时会擦顶。
  **这是当前 waypoint 模板相对 packing 缺的一刀**（见"缺口"）。
- **走廊横截面** = 当前箱足迹 + 臂/吸盘膨胀，扫占用栅格。
  `CargoVolumeMapper.corridor_free_confidence`（L318）已是这条查询；
  occupied → 槽作废，unknown 按 fail-closed 处理。

insert/descend 只在槽柱内下降；支撑面接触用 ACM 放开（对齐 ROS 1
`_set_place_support_touch`，scene_manager 已实现
`set_place_support_touch`）。笛卡尔采样就是走廊上的直线（必要时先水平再抬），
`GetCartesianPath` 即可。

### 第 3 层：机器人（MoveIt 只做核对）

PlanningScene 里有集装箱 mesh、所有 `placed_*`、手上的箱子。对 transit 接触位
做碰撞 IK（`placement_motion_filter` 已有）；笛卡尔 `avoid_collisions=True`。
它否决"几何走廊看着行、连杆会撞"的假候选，**不负责发明新路径**。

HOME 同理：沿开口法向笛卡尔退到 portal，再回 observe。不从箱内 FJT/OMPL
（切片 A 已实证箱内 FJT 会 `GOAL_TOLERANCE_VIOLATED`）。

## 候选 → 硬过滤 → IK 接口（学习后接的位置）

```
候选生成（便宜，可换）      约束过滤（确定性，算法自持）        求关节
几何折线模板 / 以后换网络 → 开口投影 + 走廊占用 + 高度 + FCL → compute_ik
                                                           GetCartesianPath
```

- 候选先表示为**末端位姿折线**（world，`suction_contact_frame`），不出关节角。
  参数只有：走廊高度 `h`、开口平面内侧偏 `y`、载荷 `yaw`、先抬还是先出
- 学习以后只换候选生成：先学 `(h, y, yaw)` 或少数 SE(3) 路点，
  **不学 6 轴时间序列**；碰撞/走廊/IK 永远走硬过滤（Motion Planning
  Diffusion / traj-CVAE 同构，但本流形近 1D，几何提案先行）
- CHOMP/TrajOpt 可作中间层：几何折线当初值 + 碰撞代价平滑，仍非 OMPL

## 当前代码缺口（对照本架构）

| # | 缺口 | 现状 | 归属 |
|---|---|---|---|
| G1 | traverse/extract 高度用固定 `place_clearance_z=0.15`，不看沿程表面 max | `waypoint_generator.py:12` | 切片 B/C |
| G2 | `outside_aperture` 未实现（placement_solver 无此 reason） | solver 只有 unknown/clearance/collision | 切片 C |
| G3 | `corridor_blocked` 只检单箱全宽墙（P2 简化） | `insertion_corridor.py:52-54` 注释明示 | 切片 C 后补 |
| G4 | 走廊占用栅格查询未接进 place 路点生成（`corridor_free_confidence` 存在但没人调） | mapper 有、waypoint 不用 | 切片 B/C |
| G5 | HOME 从箱内直接 FJT（切片 A 空载已踩）→ 已改为先 `place_exit` 退 portal | 已修 | — |
| G6 | 集装箱 collision 8000 面进 FCL，规划慢 | `scene_manager mesh_max_faces:=2000` 已支持 | 部署项 |

## 仿真时钟掉速 ≠ 真机延迟（部署备忘）

- RTF 掉速（`/clock` 慢于墙钟、FJT 墙钟超时、`ign model --pose` 卡物理）
  **不会在真机复现**——真机无 `/clock`，轨迹按真实时间执行
- **会带到真机的**：MoveIt 规划耗时（墙钟，FCL 对 8000 面 mesh）、
  速度/加速度缩放 0.3 的执行时长、箱内直线 FJT 撞箱（路径问题非时钟问题）
- 现场超时按「轨迹时长 × 余量」设，两端同墙钟；不按仿真 RTF 放大
- 仿真验证超时合理性时看**仿真时间**内是否完成，不拿墙钟除 RTF
- 加快现场节拍 = 降 collision mesh 面数（省规划延迟）+ 提缩放/更短笛卡尔抽出
  （省执行时长）；网格只影响前者
