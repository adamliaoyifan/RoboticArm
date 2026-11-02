# 2026-11-02 -- free-space gen3 资格复核

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

候选 08c5e57b826a7fa075977e70771548ff3df599ff（工具 0e3dd151，FREEZE head 7e2b7a25）仍不开放 S3。30/30 冻结文件实际字节匹配，候选 worktree clean；独立执行 g2+g3 两文件测试 36 passed / 1 skipped。此次不运行 ROS、长窗或 O9，不以 owner O9 比值推断产品性能。之前八个反例修复保留；以下七个新的原合同反例由 owner formal 健康夹具单独变异，健康对照与坏例全部 reasons=[]。这说明资格漏洞仍在，不是产品实机曾返回这些错误。

## Verification

可复现命令：`python3 docs/status/evidence/occ_freespace_final_20261102/implementation_g3_review/audit.py`。该脚本仅加载 g3 worktree 判官，不修改产品或 owner artifact。RESULTS.json 与 freeze_checks.json 在同目录。

| 原合同失败场景 | 实测判定 | 缺口 |
|---|---|---|
| P 所有 240 query 都 ACQUIRE_NO_DATA，无租约 | 通过 | `_query_record_problems` 在正常计分链无条件豁免 ACQUIRE_NO_DATA；不能把预热/reset 特例扩到 P 与 L 评分窗 |
| P feasible_total=100000，candidates=[]，空内容正确 SHA | 通过 | 只重算摘要，未对账完整数量、状态、候选域/重复、页覆盖；合法 SHA 不证明完整结果 |
| P 所有租约 epoch=-999 | 通过 | `_identity_problem` 只验整数；缺冻结/终态/页面身份关联，config/clock 亦未绑定 |
| L 所有查询 age_ns=30s | 通过 | L 评分窗未执行快照年龄≤1s |
| L health_samples=0、health_gap_max_s=100000 | 通过 | L 无 health 门，不能称 P/H 全门 |
| L 每个读轮残留缓存100MiB，目录数回基线 | 通过 | `_ownership_round_problems` 仅验目录数，缓存只验有限非负，没有与实际所有权基线对账 |
| L 全部1000读轮 ACQUIRE_INTERNAL_ERROR、无租约 | 通过 | 任意 ACQUIRE_* 都被计作生命周期成功；未实际 acquire/release 不能认证1000轮 |

## Acceptance

本轮只补原统一合同，不新增指标、不重开 S2、不启动正式长窗：

1. **统一逐记录合同**：P、R恢复普通查询、L评分窗共用完整合法状态/新鲜度/身份/结果完整性校验；预热与过载允许状态按阶段显式限定，不接受任意 ACQUIRE_*。完整候选数与数组、分页游标/唯一性/状态/身份一致，摘要从无损内容复算；身份类型与值域及 expected epoch/geometry/config/clock/observed revision 的证据关联齐全。保留合法 NO_KNOWN_FREE，不把它等同没有地图。对以上每类在首/中/末位置与 P/R/L 适用阶段做正负例，经过真实 adapter→artifact→磁盘 judge。
2. **L 的 P/H 和真实生命周期**：评分窗年龄≤1s、health间隔<1s从原始时间戳重算（缺样 not_evaluated/失败，不能用自报最大值代替）；1000轮必须真实取得并显式释放租约，失败采样另列，不计合格生命周期数。保留100次真实跨 reset 租约。每轮资源按实际所有者对账块、目录、缓存、外部读者、scratch/在途等合同类别，不能仅目录计数归位。允许有界等待消退并发瞬态，但所有采样保留时间、所有者/类别，不能靠单个计数 min 宣称所有字节已归零；以受控持续缓存泄漏和无泄漏并发为反例/对照。瞬态高峰仍计真实组合峰。补足当前七个反例且健康对照绿，沿用原门限。
3. **同源资格证据**：完成上述限定修复后跑同源有界预演、实际失败重放、磁盘完整判官；原始记录/资源采样不可裁剪。冻结产品/工具/脚本/输入，必要重测四操作 O9（原30对、≤1.10）。本轮检查不声称穷尽其余正式 runner 路径；owner应按固定合同逐项核对，禁止只对本脚本字符串特判。所有失败保留，真实性能不足可诚实留给 S3，判官缺失不能留给 S3。

## Risks

取 min 并非天然错误：它可以观察某一计数的短暂回落，但无法证明未测缓存/块无泄漏，更不能把没有取得租约的1000次尝试认证成1000次释放。当前100MiB反例只证明残留基线未判，不声称实机存在100MiB泄漏。此次没有重新运行四操作O9，owner结果保留且不赋予正式P/R/L通过意义。

## Pointers

- `docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md` @ 8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451
- `docs/status/evidence/occ_freespace_final_20261102/implementation_g3_review/`
- `docs/agents/discuss/2026-11-02_1623_occ-freespace-implementation-g3.md`
