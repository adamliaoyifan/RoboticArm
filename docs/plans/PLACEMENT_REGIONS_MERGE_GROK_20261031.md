# 区域功能默认关闭合并：Grok 执行合同

用户已授权派发合并、解决冲突及功能验收。仅执行本文及引用合并计划；不扩展工作范围。

| 子任务 | owner | generation | 基线 | 输入分支 | 前置条件 |
|---|---|---|---|---|---|
| PLACEMENT-REGIONS-MERGE | eng/cursor-grok-b/grok-4.6 | 1 | master@fc601ad1 | agent/eng/placement-regions@b6734be7 | S1 owner 完整提交同域改动及依赖后才能实际集成 |

精确 plan_revision 为包含本文件的提交，由派单线程固定。基线是派发时状态；S1 完成后的实际 master SHA 必须另行记录，不能冒称在原始 fc601ad1 上验证。用户授权完成本地 master 合并，不包含 push 或部署。

## 执行顺序与权限

1. 通过 `scripts/agent_start.sh` 领取线程。先读 `docs/agents/reviews/2026-10-31_2306_placement-regions-merge-plan.md`，按其 Merge procedure 1–6 执行。
2. 查 S1 是否完整提交。若同域改动仍未提交，登记具体阻塞路径与所需 S1 提交，等待 owner；不得代为提交、stash、reset、覆盖或删除他人改动。允许通过该任务线程请求 S1 owner 提交就绪信息，不发外部消息。就绪前只做只读检查；不运行大实验，不为了绕过依赖把区域分支提前落 master。
3. S1 就绪后冻结其 SHA 和 master SHA，在干净隔离 worktree 从 master 创建集成分支；普通 merge b6734be7。禁止 rebase 改写已审查区域历史，禁止全文件 ours/theirs。
4. 解决实际冲突，保留双方功能。重点是 solver 两个签名及 solve→generate 转发同时接收 `heading_reference_rad`、`candidate_windows`、`budget_s`；node 同时准备朝向参考与区域窗/预算，两组均传入 solver。保留 S1 directed-yaw/atlas/chain 和现有几何/支撑/碰撞门。
5. 完成下述有限验收，必要时只修合并引入的故障及缺失的组合测试；形成带完整证据的集成提交。
6. 确认 master 未产生未验证的相关变化，且主工作区重叠路径没有在途改动，再将已验证集成提交合入本地 master。不能安全落点时报告“集成已验收、主分支未合并”，不得虚报完成。不得强制更新正在被其他工作区使用的分支以绕开保护。
7. `agent_complete.sh` 关闭任务前必须满足本地 master 包含已验证区域与S1改动，默认仍关；Result 列出全部SHA、冲突处理、测试与结果、剩余开启边界。若阻塞，保持真实未完成状态。

## 限定范围

- 默认 `region_fast_path=False`、`placement_budget_s=0.0`。不修改 launch/profile 使之开启。
- 不新增产品功能，不做重构、性能优化、自适应网格、GC freeze、预算调参。
- 不跑18×30性能矩阵、Gazebo、MoveIt或实机；不宣称端到端500ms或在线无泄漏验收。
- 不修无关历史失败，不更新其他代理的memory，不改其工作树。仅更新本任务所需角色记录、线程和证据。
- floor_prior来源/容差统一、在线长会话生命周期、端到端性能留作开启前事项，本任务不实施。

## 验收程序（全部为必要功能检查）

环境：ROS Humble 和匹配消息接口；S1冻结基线及合并候选使用同一Python/依赖/配置。源码导入必须指向各自worktree，记录模块路径；依赖缺失或必要skip=not_evaluated，不能当PASS。没有稀缺评分预算，不做随机成功率实验。

1. 基线与候选各跑一次 `python3 -m pytest -q src/luggage_packing/test`；候选再跑 `python3 -m pytest -q src/luggage_planning/test/test_seed_chain.py`。全部新增/受影响测试须过、零新失败。历史env测试若失败，给出精确node-id、双边日志及原因，不能直接照旧deselect；影响本次必要覆盖则未验收。
2. 按引用计划 Bounded verification 表，补现有测试缺少的组合覆盖：
   - 默认关/预算0，空地板、已放箱堆叠、raised unknown、无空间4固定输入：与S1基线选中位置/朝向/评分、success/reason、候选语义相同，仅允许新增诊断。
   - 2cm/5cm × 区域开/关 × 朝向参考有效/无效，共8组合；功能对比用预算0。输入采用existing fixture，所有4有向heading（0、π、π/2、3π/2）保留；有向候选不能被区域footprint位置去重吞掉；有效参考在同分时选期望朝向，无效参考保留原确定排序。接受集与同输入不限预算基线一致。
   - atlas/chain自身朝向评分、双分支hard reject、unknown不误拒、dump-only不改选择，确认hook实际调用。
   - 原始2×2桥接、unknown孔洞、raised unknown、候选限额/边界、旧快照weakref释放、revision回落重建必须执行通过。
   - 有限预算用已有确定性耗尽测试确认共用deadline、BUDGET_EXHAUSTED不误报BIN_FULL；不判500ms性能门。
   - v1/v2请求与surface、geometry hash错误、旧revision、原支撑/重叠/孔口/走廊门：使用已有消息/无Gazebo节点测试，确认新旧参数转发无TypeError。必要时加小型节点桩行为测试，不能用源码字符串断言代替行为。
3. 每个固定用例一次、全部AND；它测功能不变式，不声称随机工作负载每单位成功率。若已有测试完整覆盖同项，复用而不重复实现。失败后保留证据，修复只重跑受影响项；最终候选的全包汇总必须对应最终SHA。

## 证据与完成条件

写到主工作区 `docs/status/evidence/occ_owner_b1/placement_regions_merge/`。T0=源/S1/基线/集成/最终master SHA、dirty状态、环境和命令；T1=JUnit/日志及8组合、4兼容场景结果；失败T2=精确输入、参数、双方输出、失败阶段/断言/堆栈和复现命令，先持久化再重跑。使用小型JSON fixture与日志，单失败bundle目标≤10MiB；超出先压缩并说明，禁止泛化全量dump。保存至少到本轮合并验收关闭，不覆盖旧失败。无异步消息采集任务，不要求无关ROS bag。

通过条件：所需测试全部有样本、零新失败；组合语义保留；实际master包含集成提交；默认关闭；所有结果可追溯。Result 明列“已合并未启用”，并保留端到端性能、floor_prior语义和在线生命周期三项开启边界。
