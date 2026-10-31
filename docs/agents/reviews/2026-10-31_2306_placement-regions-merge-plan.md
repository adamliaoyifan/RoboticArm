# 2026-10-31 -- 区域分支默认关闭合并计划

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

用户要求合并计划及 rebase/验证判断。本次只做只读检查与计划落盘，没有 merge、rebase、stash 或提交他人工作。

推荐：S1 owner 先提交其完整依赖，再从届时 master 建干净集成 worktree，普通 merge 区域分支、处理接口冲突、验证后将相同集成提交落 master。**无需 rebase；当前不能在脏的主工作区直接 merge。** 此计划尚非 runnable 派单，S1 最终提交和集成 owner 未冻结。

## Current evidence

- 本次本地 master=`41ab0b7b`；区域分支=`b6734be7`；共同祖先=`c9090064f1bfb54e4cf4a1dbaa7513626be0eacc`；两侧分别 9/5 个独有提交，不能 fast-forward 直接落区域分支。
- master 相对共同祖先仅文档/评审证据变化，与区域分支已提交改动无文本冲突。Git 2.34.1 不支持 `merge-tree --write-tree`；改用 `git merge-tree c9090064 master agent/eng/placement-regions` 检查，无双方修改冲突标记。
- S1 工作区仍有未提交 solver/node/reachability、测试及 seed_chain 等依赖。对两个重叠文件作只读三方合并：`git merge-file -p S1_WORKING_COPY MASTER_BLOB REGIONS_BLOB`，临时副本不影响任何工作区。**solver 3 块冲突、node 2 块冲突**。此结果仅针对本次读取的 S1 快照，owner 后续变化需重查。
- solver 冲突：`generate_candidates`、`solve_placement` 的可选参数，以及 solve→generate 转发。必须同时保留 `heading_reference_rad` 与 `candidate_windows` / `budget_s`，保持旧调用兼容。
- node 冲突：请求的有效朝向参考准备与区域窗/预算准备；调用 solve 时两组关键字必须都传递。S1 atlas 自身朝向评分、pi 分支选择和 chain annotation 也需保留。

## Merge procedure

1. **S1 owner 固化完整改动。** 包括其所需模块、测试与资源引用，不能只提交冲突的两个文件而遗漏 seed_chain 等依赖。不替 owner stash/reset，不覆盖或挪走其在途工作。冻结新 master SHA 并记录工作区状态。
2. **建立隔离集成分支。** 从该 master SHA 建 `agent/eng/placement-regions-integration` worktree；执行 `git merge --no-ff --no-commit b6734be7`。此命令在将来的干净集成 worktree 执行，不是在当前共享脏工作区执行。
3. **处理组合语义。** 不使用全文件 ours/theirs。保留双方参数、转发、排序及节点准备逻辑；区域按 footprint class 生成位置提示，S1 的 0/π、π/2/3π/2 有向候选不能被位置去重吞掉。区域开启也不能跳过 atlas/chain 或既有几何/支撑门。默认参数保持 `region_fast_path=False`、`placement_budget_s=0.0`。
4. **冻结候选并验证。** 在本地集成分支形成候选提交，记录 SHA；测试必须导入该 worktree 的源码及同版本 ROS 接口。修改解决失败后形成新 SHA，只重跑受影响检查和最终汇总，不重复大性能矩阵。
5. **落 master。** 验证通过且 master 仍为步骤1基线时，可将 master fast-forward 到已经验证的集成提交；这与步骤2创建 merge commit 不矛盾。若 master 又有相关变化，更新集成落点并重验受影响项。不要把旧落点的 PASS 当作新落点的 PASS。共享工作区重叠路径仍有未提交工作时，暂不更新该工作区。
6. **落档。** 记录源分支 SHA、S1 SHA、集成/最终 master SHA、冲突解决说明和测试结果；状态写“已合并、默认关闭”，不是“已启用/生产 SLA 已通过”。

Rebase 不会消除 S1 接口冲突，只会改写已评审的区域提交，增加证据映射工作；没有线性历史要求时不推荐。普通 merge 保留原提交和修复证据。

## Bounded verification

基准为“已包含 S1 的冻结 master”，不是旧 c9090064。每个确定性用例运行一次、全项 AND；任何新失败阻止落 master。此门验证给定输入的功能不变式，不据此推导线上成功率。环境缺依赖/必需测试 skip 为 not_evaluated，不得计 PASS。

| 验证 | 具体观察与通过条件 |
|---|---|
| 默认关闭兼容性 | 区域关、预算0；空地板/已放箱堆叠/raised unknown/无空间四类固定输入，相对 S1 基线 success、reason、选中位置/朝向/评分及候选语义一致；只允许新增诊断字段。节点初始化不构建区域，launch/profile 不覆盖成开启。 |
| S1 与区域组合 | 2/5cm × 区域开/关 × 朝向参考有效/无效，共8组合；开时先用无预算隔离枚举顺序影响。0、π、π/2、3π/2 同 footprint 的有向候选均保留；同分时有效参考决定朝向，无效参考沿原确定性排序；最终接受集合与不受预算限制的基线一致。 |
| atlas / chain | 保留自身 directed bin 评分、双分支 hard reject、unknown 不误拒、dump-only 不改选择；有效 hook 实际被调用。结合 S1 已有 atlas 与 seed_chain 测试，确认区域优先窗没有绕过这些步骤。 |
| 区域不变式 | 原始2×2桥接得到2区、各跨度≤容差；unknown孔洞、40cm全unknown不标prior；候选限额/去重/边界；换版旧快照释放、revision回落重建。复用现有 regions 测试。 |
| 有限预算 | 精确有限用例验证优先窗+回退共用预算，耗尽返回 BUDGET_EXHAUSTED 而非 BIN_FULL；不重新判500ms SLA，不要求耗尽后仍完整搜索。 |
| 节点与契约 | v1/v2 surface及请求链、geometry hash不符、旧revision拒收、错误分类、支撑/重叠/孔口/走廊门。使用已有真实消息/无Gazebo节点测试；检查双方新增参数完整到达 solver，无 TypeError。 |
| 全包回归 | 在相同环境对 S1 基线和集成候选各运行 luggage_packing 测试一次；新增 regions 用例必须全部执行且过，S1 seed_chain 测试也过。任何历史环境失败需精确node-id、双边结果和环境原因；不沿用“之前deselect过”作为豁免。 |

已有测试入口：

```bash
python3 -m pytest -q src/luggage_packing/test
python3 -m pytest -q src/luggage_planning/test/test_seed_chain.py
```

执行前应在干净集成环境构建/加载 ROS Humble 及匹配的 `luggage_msgs`、各包依赖，源码导入路径指向集成 worktree；先以 collection/import 检查确认加载位置与测试数。`test_frame_migration_placement.py` 需要真实消息类型；`test_placement_geometry_identity_live.py` 使用 ROS executor、无需 Gazebo。S1 尚未提交的新测试由 owner 一并纳入。

已有测试若缺上表组合覆盖，仅补组合回归文件，不新增产品功能。证据写到 `docs/status/evidence/occ_owner_b1/placement_regions_merge/`：环境/依赖与revision清单、baseline及merged的JUnit与日志、固定输入和失败输出。失败先保留输入/阶段输出，再修复，不盲目重跑。

本轮不跑18×30性能矩阵、Gazebo或实机，不调整预算，不开启区域，不做自适应底层或GC优化。端到端计时、floor_prior来源与共同容差、在线长会话生命周期仍为开启前独立事项。

## Pointers

- `docs/agents/discuss/2026-10-31_1938_occ-placement-regions-draft-review.md`
- `docs/agents/reviews/2026-10-31_2058_placement-regions-disabled-merge-review.md`
- `docs/status/evidence/occ_owner_b1/placement_regions_20261031/README.md`
- `src/luggage_packing/luggage_packing/placement_solver.py`
- `src/luggage_packing/scripts/placement_planner_node.py`
- `src/luggage_packing/test/test_placement_solver.py`
- `src/luggage_packing/test/test_placement_atlas_hook.py`
- `src/luggage_packing/test/test_frame_migration_placement.py`
- `src/luggage_packing/test/test_placement_geometry_identity_live.py`
