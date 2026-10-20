# r10/r11 transit 爆炸与 settle 失效的量化归因（2026-10-20 源码+证据复核）

按合并范围审计（docs/agents/reviews/2026-10-20_1026_prior-g3-merge-scope-audit.md）
指引完成。**修正此前错误表述**："route_producer 全按 G3"不成立——公共建路、
调速、执行等待三处均被合并改变（G3 ce361117 → HEAD 2eef0835：
route_producer 372 行、trajectory_timing 26 行、motion_executor 11 行）。

## 1. transit 10s → 121–166s：两个独立乘子，已在真实数据上复现

在 r11 cert_request（b64 反序列化的真实 knot 序列）上重放新旧两个时间参数化：

| leg | class | n_knots | G3 计时 | 合并版计时 | time_scale |
|---|---|---|---|---|---|
| 0 | PORTAL_TRANSIT (carrying) | 323 | 35.22s | **165.88s** | **4.709** |
| 1 | PORTAL_TRANSIT (carrying) | 197 | 20.73s | 88.12s | 4.251 |
| 2 | TRAVERSE | 19 | 6.64s | 6.79s | 1.023 |
| 8 | RETURN | 220 | 32.54s | 121.41s | 3.731 |

165.88s 与执行侧分毫不差（joint_ring 166.1s、plan_sec 171）。乘子分解：

1. **几何绕行 ~2.4–4.8×**：合并版 route_producer 引入 staging 候选
   （`staging_positions` 多位姿 × carrying/unloaded）＋目标梯级＋走廊高度，
   r11 出现**连续两段 PORTAL_TRANSIT**（staging 走廊腿；r5 为 1 段），
   leg0 关节路径 15.96 rad vs r5 同段 3.30 rad。本次走廊高度未激活
   （corridor_audit verdict=empty_map，首箱前地图为空）。
2. **样条加速度界整体时间拉伸 ×3.7–4.7**：合并版
   `time_parameterize_cartesian` 追加 `required_time_scale`
   （spline_timing_bounds，prior 分支 c0853505，硬件安全动机：五次
   Hermite 内部导数界）。绑定项是**内部加速度**：G3 计时下该界超限
   ~22×（多跳 lattice 路径方向折点），拉伸后 a 恰好=限值 0.349、速度
   仅用限额 20–23%。巡航从 0.472（=1.57×0.3，r5 实测满速）降到
   0.108 rad/s，全程恒速无驻留。Cartesian 直线腿（traverse/retreat）
   scale≈1.02 不受影响。

速度比例常量未变（MOVEIT_VELOCITY_SCALING=0.3 两版一致）；变慢与
prior adopted 无关（adopted=false 也走同一路径）。

## 2. HANDOFF_SETTLE_TIMEOUT：staging 末端姿态载荷下不可保持（候选细化）

r10/r11 同签名：目标（staging 姿态）joint2≈-2.39 rad；下垂集中在承力
关节 1/2（末端误差 0.075/0.148 与 0.100/0.143 rad），**单调增长**（6s 窗
口内无收敛趋势）；控制器输出冻结在目标值。r5 同段末端为另一姿态
（joint2≈-1.73），settle 0.37s、误差 2.7e-05。

判定（**2026-10-20 晚被进一步修正**，见 SETTLE_CONTROL_EVIDENCE.md）：
初版"6.9kg 载荷重力下垂不可保持"归因**撤回**——carry 是 set_pose 运动学
跟随（vacuum_controller_node.py:8），箱重不产生关节力矩；URDF 静力计算
与观测模式矛盾（j1 轴竖直 τ_g≈0 却漂移 0.075；j3 扛 53 N·m 却零误差；
r5 姿态 j2 重力矩更大 82 vs 68 N·m 却保持 2.7e-05）。观测=恒速线性漂移
（r10/r11 速率一致 0.0165–0.0169/0.0346–0.0355 rad/s），窗口内未见收敛
也未见反转；"任何等待都无效"为过度推断，撤回。当前只签：**j1+j2 恒速
持续超限、机制未定**（持续外力矩候选未排除，接触流该两 run 未捕获）。

## 3. 含义（待用户裁定，未做任何修复）

- transit 慢是"硬件保守界 × lattice 拼接折点"的组合；拉伸因子恰=加速度
  违界比率（4.71/4.25/3.73），r5 自己的腿违界比率 0.99（旧计时本就满足
  限值）。10 腿认证轨迹全长 **399.8s**（G3 计时 112.6s；此前"375s"为
  三腿部分和，全周期还需加规划、交接与其余腿）。
- settle 失效与 prior 排序无关；与"加 spin"类猜测无关。
- 可能的修复方向（均未实施）：staging 姿态可保持性约束/选择、
  lattice 边 knot 平滑（降低拉伸因子）、或对仿真插值模型的界放宽
  （需确认真实控制器插值语义）。

## 复现命令要点

r11 cert_request.json `request_b64` → `ValidateRouteV2.Request` 反序列化；
`time_parameterize_rest_to_rest` vs `time_parameterize_cartesian`（0.3/0.3）
对比 duration/time_scale；速度廓线由 joint_ring 与 cert 样本
（sample_times_ns × sample_positions_flat）测得。
