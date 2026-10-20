# Stage 2b：同冻结腿经 MoveGroup ExecuteTrajectory 执行（2026-10-20）

按用户裁定执行：与 stage 2a 完全同场景/同初始关节/同附着关系/同 323
knots（含速度/加速度字段），**唯一变量 = 腿经 /execute_trajectory
（move_group）提交**，goal 构造复刻生产 motion_executor "timed joint path"
分支（RobotTrajectory.joint_trajectory = 认证 knots 逐字；`_timed_submit`
hook 生产从未设置，r10/r11 确实走 move_group 路径——本轮已核实）。

工具：scripts/g3/hold_diag2b.py（boot 同 hold_diag_boot.sh；A′ 容器、
seed 0、static recess 箱 0xFE、r11 实测附着关系、生产 SimVacuumBackend
30Hz follow）。输入轨迹 sha256 与 2a 相同；生产等待公式值
（timed_execution_wait 与 G3 公式取小）记录为 **225.88s**（诊断实际等待
放宽至 span×2+300，不构成行为对齐）。

捕获：joint_states（含 effort）、controller_state、/clock（sim 秒）、
两层 action 状态流（/execute_trajectory 与控制器 FJT 的 status 话题）、
接触（加界 --num 300）、follow tick 计数。保持窗 = **60 仿真秒**（/clock
驱动）。

## 结果（有效运行；前两次无效尝试见文末）

| 项 | 值 |
|---|---|
| 定位（harness，20s） | SUCCESS |
| 冻结腿（move_group） | **SUCCESS，error_code=1，166.06s wall（RTF=1）** |
| follow | 7105/7105 ok |
| 保持窗（60 sim-s） | 每关节最大误差 **≤7.7e-5 rad**，斜率 ~1e-9 rad/s |
| 生产 settle 条件（0.01 rad ＋ |v|≤0.01 ＋ 3 样本/0.3s） | **全部 ~6000 个保持样本持续满足**（离线重算；脚本内置判门的 not_met 为脚本窗口选择 bug，以离线重算为准） |
| 箱接触（携带段） | 0 行（static 箱预期，同前） |

**判读（按用户规则"若未复现"分支）**：漂移经 move_group 执行链仍**未复现**
（隔离条件下）。下一项单一变量 = **恢复 pick 前序历史**；其后才考虑全栈
并发。不可归罪 move_group（本轮它执行完美）。

## 捕获层观察

- 控制器层 status 话题上**所有** goal（含成功返回的）都呈 2→4 的状态
  序列（该 JTC 版本的生命周期编码），不单独视为异常；mg 层 ACCEPTED→
  result 1。
- **/joint_states.effort 字段全程为 NaN**（字段存在、无物理输出）——
  用户保留意见的具体实证：该通道在仿真中不提供力矩证据。

## 未覆盖的驱动环节（明确清单）

perception/pick 序列及其状态、驱动侧 certified ExecutionWait 门、handoff
门本体、经真实 scene 服务的规划场景附着、prior sidecars、pose/info echo
守护、graph probe——即 executor 调用之外的全部上下文。

## 无效尝试记录（harness 教训）

1. attempt 2（结果字段 bug 崩溃前腿已跑完）+ attempt 3：崩溃遗留臂在腿
   末端 → 6s 定位走不完 6.1 rad（JTC 容差中止）→ 起点偏差 >0.05 被
   move_group 拒（"start point deviates from current robot state more
   than 0.05"）。定位改为 20s 后消除。move_group 的 0.05 起点校验门
   本身是生产链的组成事实，一并记录。
2. 崩溃遗留箱需重启清理（服务删除超时一次）。
