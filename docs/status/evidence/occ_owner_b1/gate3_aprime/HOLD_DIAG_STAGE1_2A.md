# 单段保持诊断 stage 1 / 2a：负复现结果与机制排除链（2026-10-20）

用户授权的有界诊断（"先验证接触与控制输出采集确实有数据，再执行一条
冻结腿；不跑完整两箱"）。全部为诊断性单腿运行，非计分、非产品流程。
工具：scripts/g3/hold_diag_boot.sh（run.sh 同款场景/清扫/A′ 容器/seed 0）、
hold_diag.py（stage 1）、hold_diag2a.py（stage 2a）。数据 /tmp/hold_diag/
（run1、run2b 为有效运行）。

## 复现物

冻结轨迹 = r11 cert_request leg0 逐字（323 knots、165.876s、巡航
0.092 rad/s、rest-to-rest 端点速度 0）；末端姿态 = r10/r11 失效点
[-0.8542, -2.3921, 2.1367, -1.3108, -1.5667, -2.4251]。

## Stage 1（裸臂 + 冻结腿，无箱/无真空/无 MoveIt）

- 定位至起点（goal ok 6.05s）→ 起点 10s 基线 → 冻结腿 → 60s 保持。
- 结果：**goal 166.05s 完成（RTF=1）**；保持窗全关节误差 ≤1e-4→0，
  斜率 ~1e-9 rad/s。j1/j2 无漂移。
- /joint_states **effort 字段可用**（100Hz）——后续运行获得直接力矩通道。

## Stage 2a（+ 生产携带箱 + 生产 follow 循环）

忠实要素（经两次失败迭代后达成）：
1. 箱 = 生产 recess SDF（pickup_visual_sdf，box_recess、lid_offset
   0.011033、**static=true**——驱动 place_smoke_driver.py:3167
   "Carry stays static so set_pose follow does not fight the solver"）。
2. 附着关系 = **r11 实测**（tf_trace before_transit 杯位 [0.565, 0.592,
   1.427] + pose_probe 箱位 [0.559, 0.600, 1.320] RPY[0,0,π/2]，
   世界差向量 [−0.006, +0.008, −0.1075]，箱 yaw=π/2）。
3. follow = 生产 `SimVacuumBackend.follow_step` 逐字（30Hz、TF 杯位、
   /world/.../set_pose）；p04 carry filter 走真实服务。
- 结果：**goal 166.05s 完成、follow 7105/7105 ok、保持窗全关节
  1e-5→0、斜率 ~1e-9**。j1/j2 无漂移。RTF=1。

## 结论：隔离条件下未复现（非全局排除；用户修正 2026-10-20）

r10/r11 的 j1+j2 恒速漂移（0.0165/0.0346 rad/s）在以下**本次隔离条件组合**
下均未出现。执行路径、前序状态与并发可能存在交互，逐项不构成全局排除；
stage 2b（MoveGroup 执行链）也不是最后一个结构差异：

| 排除项 | 依据 |
|---|---|
| 末端姿态静态特性/重力 | stage 1：同姿态完美保持 |
| 冻结轨迹内容本身 | stage 1/2a 同轨迹执行无漂移 |
| static 携带箱 + 30Hz 生产 follow | stage 2a：无漂移，follow 全成功 |
| 直接 FJT 控制路径 | 两 stage 均直发 /elfin_arm_controller |

**剩余差异（未复现的 r10/r11 要素）**：move_group ExecuteTrajectory
执行路径（motion_executor → move_group → 控制器，而非直发）；驱动全栈
并发（prior sidecars、pose/info echo、graph probe 等）；transit 前历史
（完整 pick 序列）；MoveIt 规划场景附着。→ 下一级（stage 2b）= 同冻结腿
经 /execute_trajectory 走 move_group。

## 副产品发现（记录修正/工具事实）

1. **r10/r11 contact_stream 0 行 ≠ 采集损坏**：携带箱是 static 体，
   gz 抑制 static-static 接触、箱不接触动态臂链（杯包络为声明近似），
   携带段传感器本就无接触可报。SETTLE_CONTROL_EVIDENCE.md A9 的
   "缺证"细化为：携带段 0 行是预期行为；接触证据只在释放/动态阶段存在。
2. **URDF 根 ≠ 世界放置**：机器人模型在 gz 中布置于台座偏移上，
   离线 FK（原点根）算出的杯位差 ~0.8m。世界位姿真值源=tf_trace/
   pose_probe/TF。本轮已测得生产附着关系（见上）。
3. 接触传感器未加界的 `ign topic -e` 会洪流（每条消息 ~10KB+、近乎每
   物理步都有），曾两次拖垮 RTF 使运行无效；必须 `--num` 加界。
4. 动态箱 + 中空 spawn 会自由落体/楔入 → 物理爬行（RTF 0.08），
   生产语义是 static 携带——复现必须 static。

## 两点保留（用户 2026-10-20）

- /joint_states.effort 字段存在 ≠ 已验证其物理含义（未与已知负载/
  静力矩对照标定，本轮只记录为"字段可用"）。
- 箱体接触流 0 行不能排除**其他**机器人—环境接触（传感器只在箱上；
  台座/臂链与环境接触不在其观测范围）。
