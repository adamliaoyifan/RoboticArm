# 2c 保存段的完整性与有界重放草案

- base_revision: 89142b4f
- dispatch_ready: no
- simulation_started: no
- input_gate: unresolved_historical_goal_evidence

## 已完成的输入检查

输入严格限定为 `inputs_2c/last_boundary.json`，SHA-256
`5a9a4803103cb752348b4331a1e83eeb4275489ab31112b387131da195fa5d25`。
这是该次 driver 向 PlanMotionV2 传入段的有损记录，不是历史实际 FJT goal。

| 字段 | 检查结果 |
|---|---|
| 关节顺序 | 完整，elfin_joint1…6 |
| 点数 | 257/257，未截断 |
| positions (`q`) | 每点6个有限数值 |
| velocities (`v`) | 每点6个有限数值；末点全零 |
| accelerations | **日志未保存（0/257字段存在）；历史实际数组空/非空未知** |
| time_from_start (`t`) | 每点存在、有限、严格递增；0至111.137935s |
| 时间精度 | driver舍入至六位小数，原始整数ns缺失 |
| header | 缺失 |

机械检查结果见 `2c_field_audit.json`。源码
`src/luggage_gazebo/scripts/place_smoke_driver.py:334` 的 `trajectory_dump`
只生成 q/v/t，因此可确认导出省略该字段，不能据此判断历史实际请求携带空数组还是非空数组。另针对该2c原始 `trials.jsonl`
和失败目录 `segments.jsonl` 检查：未发现 accelerations/a/joint_trajectory/
trajectory 字段；该检查不是对所有未归档文件的穷尽搜索。

**历史实际输入未能确定，历史原样重放维持未准入。这里缺失的是证据，未发现轨迹字段有错。**
若实际数组显式为 `[]`，它就是完整的原始语义，可以保持空数组重放；不能仅因
数组长度为零而拒绝。现有typed序列化/提交代码确实支持并保留空数组，但代码
不能替代历史实际goal。此次不再追猜旧日志，下一步转为新的一次生产边界捕获，
详见 [生产捕获准备](../production_capture_prep_20261020/PLAN.md)。

不补零、不用差分推导加速度、
不借用r10/r11字段，也不默认为空数组；这些都会新定义输入或插值语义。
不再尝试补猜此次日志；另一次明确标识的新运行应捕获完整边界；
新运行的完整轨迹不能追认成此次2c的257点原始轨迹。本轮不启动该捕获运行。

## 输入补齐后的实验设计（尚未冻结或准入）

1. 将完整输入另存为不可变CDR及规范JSON，记录 names、每点 q/v/a/effort、
   整数ns、header、字段摘要和哈希。新实验设置的header与历史未知header
   分别记载；不得宣称历史实际goal已恢复。若日志仅剩q/v/t且不能证明原始a为空，应另立
   明确的实验定义，不能复用“完整原样重放”的结论。
2. 沿既有2b隔离环境、MoveGroup ExecuteTrajectory链路执行，场景/附着/
   控制器配置、定位过程与60仿真秒保持窗保持2b定义。不改staging、速度、
   生产等待/settle门。只执行一次目标腿，不自动重试；同run完整记录定位
   goal，避免误认作目标腿。起点不满足既有门则记实验未进入执行。
3. **运行前必须具备实际FJT请求捕获**：保存送至控制器的完整goal和其UUID，
   不能只保存上游RobotTrajectory或仅status话题。若使用记录并转发的FJT
   action代理，必须先离线验证字段恒等、accept/reject/result/cancel的映射，
   记录代理延迟，并将新增代理列为实验条件；捕获方式尚未实施，不具备
   此项能力不得启动。保存ExecuteTrajectory目标/结果及UUID映射。
4. 比较冻结实验输入、ExecuteTrajectory、实际FJT请求的names、点数、
   q/v/a/effort、整数ns及header；预期逐字段相等，顺序改变须按关节名
   显式映射再对比。任何未预先声明的转换均记 `input_not_preserved`，
   不得解释为原输入复现/未复现。
5. 使用持续spin的专用采集线程，阻塞CLI查询不得进入订阅回调；关节与
   controller_state保存各自header源stamp、本地接收mono、序号与丢样计数。
   保存两层action UUID及状态/结果/cancel，保留所有终态后再结束采集。
   `/clock`只决定保持窗长度，不给其他消息重新赋源时间戳。
6. 对比执行末端2仿真秒与返回后60仿真秒。按关节报告 planned/reference
   与物理位置的最大误差、RMS、末端越程、物理速度、保持段有符号斜率、
   首次满足及持续满足既有settle条件的区间。物理量取 `/joint_states`；
   reference是本次已确认语义的控制器参考值。报告source时间范围和有效
   样本数，不能用未对齐的接收时刻拟合漂移。

这些是草案边界，并非可执行命令或已派发任务。完整输入和捕获器到位后，
还须冻结具体运行命令、有效参数/安装哈希、采样完整性阈值与时间盒。
启动/采集练习只验证脚手架，不执行额外目标腿；首次目标运动后不追加尝试。
本草案不授权消耗此前讨论的单次仿真额度。

## 最小持久捕获矩阵

| 边界 | 每次保留 | 失败/异常时保留 | 窗口与检查 |
|---|---|---|---|
| 冻结输入、上游提交、实际FJT | 全部字段、CDR、规范JSON、哈希、run/leg/UUID映射 | 原始请求与响应 | 每个goal全量，终态后核对字段一致性 |
| action生命周期 | 两层goal UUID、源stamp（存在时）、接收mono、状态、结果、cancel | 全部事件 | 定位开始至保持窗结束，区分定位与目标腿 |
| joint_states/controller_state | 源stamp、接收mono、序号、关节顺序、q/v/effort和reference/feedback/output | 目标腿全程与60秒保持数据 | 离线提取末端2秒；报告丢样/最大源时间间隔 |
| 环境与结果 | revision、dirty diff、有效参数、安装哈希、命令、起点/附着、度量 | 捕获健康状态与缺口 | 数据压缩并持久化后再使用既有stop_sim.sh拆栈 |

原始输入证据保留不覆盖；新实验目录独立。完整矩阵的存储预算/捕获健康门
尚待输入大小和实现确定，不以未定参数伪装成dispatch_ready。

## 判读边界

- 一次运行只描述该确定输入及所记录隔离条件；不估计成功率或可靠性提升。
- 输入传递和采集完整后，若出现末端减速/保持异常，可在本次捕获的确定
  输入上定位；不自动证明历史实际goal相同，亦不自动证明变换层有缺陷。
- 未出现异常只说明该条件下未触发；生产上下文差异仍开放。
- 输入缺字段、字段传递不符、源时钟无法对齐、缺关键action事件：
  `not_evaluated`，不算复现或未复现。当前停在历史输入证据门，转入新生产捕获准备；数组显式为空并非不完整。
