# 学习辅助装箱规划讨论汇总

- date: 2026-10-13
- status: discussion_draft
- dispatch_ready: no
- scope: 方案与学术讨论归档，不是运行授权

## 核心结论

2026-10-14 更新：用户已授权将双仿真后端与 Top-K 学习框架合并为独立研究任务。
执行范围以 [统一计划](05_step_bpp_dual_sim_topk_master_plan.md) 及其派发线程为准，
首代仅建设 STEP-BPP 上的 Isaac Lab / MuJoCo 后端和训练原型。本文原有讨论与
后续论文、规模训练、产品晋级仍不是运行授权；MASTER-PACK G3 不受影响。

目标是提高给定计算预算和安全约束下的**实际可执行装箱容量**，不是为了使用学习而替换规划器。
离线搜索专家轨迹、在线生成 Top-K 候选有工程价值，但这一配方本身不是充分的学术创新。
推荐先研究 proposal + refinement + certification，不让模型直接绕过验证下发动作。

场景总体静态、变化由事件驱动，因此不必每帧重做全局规划；执行反馈和异常监控仍须连续运行。
仿真平台迁移不是前置条件，静态路径数据可以先由运动学和碰撞引擎生成。

## 阅读顺序

| 文件 | 内容 |
|---|---|
| [01 技术方案](01_method.md) | 问题定义、输入输出、候选生成、约束、教师数据和适用边界 |
| [02 仿真与事件策略](02_simulation_and_events.md) | Isaac Lab / MuJoCo / 现有栈的分工，地图与路线的失效规则 |
| [03 学术定位](03_academic_positioning.md) | 相关工作、可能的贡献、不能宣称的新颖性、投稿判断 |
| [04 验证与待决事项](04_validation_and_decisions.md) | 有限阶段、对照与指标、停止条件、正式派发前的缺项 |

## 两个层次不要混在一起

**最小工程原型**：固定安装、抓取和合法落点，学习带箱进入与释放后退出的候选路径，
检验是否能减少在线搜索成本。它并不自动解决容量最优，也不自动构成论文。

**候选科研主线**：联合抓取关系、放置位姿和进退路径，研究阶段耦合是否能在相同预算下
提高完整操作成功率和最终容量。这是待验证假设，具体新方法尚未确定。

## 当前证据边界

- BASELINE G1 的 828 是几何可行候选数，不是可执行路线数。已记录失败集中于少数 portal 目标的
  GOAL_STATE_INVALID；缺提交边界，不能据此断言经典规划能力不足。
- 那次 graph 使用情况来自源码与启动参数推定，不是运行参数直接捕获；不能算作 lattice 对照。
- 后续 e3a1968 非计分捕获已补全 Select 输入及四个精确 portal 目标场景；保存的 certifier
  结果均为 elfin_link1 / b-payload 碰撞。目标生成根因和 selector 行为重放仍未完成，
  见[诊断 review](../../agents/reviews/2026-10-13_2038_portal-capture-review.md)。这不改变传统方法比较必须使用健康基线的要求。
- 几何容量、空容器末端可达率、完整放置成功率和真实连续装箱容量是不同指标。
- 当前仿真与实机动力学不能直接等同，模型输出经碰撞检查也不等于实机安全得到证明。

以上沿用前序讨论和记录，本次没有重新执行实验或复核原始 bag。

## 与现有文件的关系

- [原始方案与研究记录](../../agents/reviews/2026-10-13_2010_learned-placement-proposals.md)：本目录的讨论来源，保留不移动。
- [学习研究可行性计划](../learning_research_feasibility.md)：已有研究授权与晋级边界，本目录不修改或扩展其授权。
- [容量与联合规划宏观计划](../container_capacity_joint_planning.md)：硬件 TF、容量与操作能力研究。
- [BASELINE / SEED-CHAIN 计划](../master_pack_baseline_seed_chain_recovery.md)：独立谱系，本次不派发 SEED-CHAIN。
- [BASELINE 证据收口](../../agents/reviews/2026-10-13_1914_baseline-g1-evidence-closeout.md)：失败证据与结论边界。

规范以 [Placement](../../architecture/placement.md)、[真实场景](../../architecture/real_scenario.md)
和 [特权信息边界](../../architecture/privilege_boundary.md) 为准。本目录的研究建议不覆盖规范。
本次仅整理文档，没有启动训练、迁移仿真、修改产品代码或批准计分运行。
