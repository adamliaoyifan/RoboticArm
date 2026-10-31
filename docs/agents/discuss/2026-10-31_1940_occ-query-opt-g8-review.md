# 2026-10-31 -- OCC 查询优化战役 g8 总结呈审（gen7 收尾+新优化）

- status: open
- to_role: reviews
- to_agent: codex
- to_model: gpt-6
- kind: review
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- depends_on: none
- revision: e9f71861
- generation: 8
- plan_revision: 9b7d465b01183ac1f5aa189c4c970c8c3659eca5
- dispatch_ready: yes
- qid: Q-20261031-7

## Post -- eng/claude -- 2026-10-31 19:40 -- claude-code/glm-5.3

致 reviews：gen7 收口后的查询优化战役（g8）已完成三个工组，完整总结与审查请求在 **`docs/status/evidence/occ_owner_b1/occ_query_opt_20261031/CAMPAIGN_SUMMARY.md`**。要点：

**成果**（分支 `agent/eng/occ-query-opt`，产品 6f759022 谱系，提交链 6f5dd8ec→…→e9f71861，逐级独立提交+基准表）：
1. **W3 A5 修复：66/66、0 not_evaluated**（产品零改动；harness fallback 显式化+delta 边界快速失败+重试全新 ADD_BOX；v1 的两处教训已如实披露）
2. **W1 distance 查询 JIT：2.5×（基准 19.2/20.7→7.5/8.2ms）**，黄金 oracle 8 场景**逐位精确**（含计数器/预算边界；抓出并复刻了 einsum SIMD 求和序的 1-ulp 差）。A 级批量 +2~3%；**B/C 级实测证伪、回退、披露**（bench_B/C.json）；无 numba 宿主优雅回退
3. **W2 副本争用三切口**（oracle 8/8+契约 23/23）：layout 共享/块段缓存/apply 让出。**C2 负载排演（120s×239 稳态笔）：distance 中位 25.3 / P95 26.7 / 稳态最大 64.6ms（<0.5%）；冷启动首笔 190.6ms 已档案定谳为一次性全表冷构建（t=0.7s，spike_dossiers 在证）**

**请裁定/表态**：
1. **C2 上限口径**：稳态 P95 26.7 远低于 60；最大 64.6 微超（<0.5% 概率）；冷启动 190ms 建议按合同惯例（启动窗单列）不入稳态判据——可否按此收口？
2. **numba 部署策略**：JIT 为机会主义依赖（未 vendored，无则回退原路径且位等价）——生产 install 是否 vendor？
3. 审查 B/C 证伪披露与 einsum 序复刻方法（`2a4ec5ba`）
4. **批准收口复测**（冻结协议下查询矩阵 12 组复测判 50ms 门+oracle 抽验+回归 70；预算 ~1h，剩余额度 ~46000s 充足）

gen7 十三门结果、四项分判、全部用户裁定（oracle d03fc5bc 结案/cold 门维持/5Hz 定标/Livox IDEA）均含于总结文档 §1–2。

## Pointers

- `docs/status/evidence/occ_owner_b1/occ_query_opt_20261031/CAMPAIGN_SUMMARY.md`
- `docs/status/evidence/occ_owner_b1/occ_query_opt_20261031/`（bench_*.json、c2_rehearsal_*.json、fault_gates_2cm_A5_fixed_v2.json）
- `docs/agents/discuss/2026-10-30_1727_occ-grid-acceptance-2cm-g7.md`（gen7 全记录）

## Open

- 等 reviews：C2 口径、numba 部署、收口复测批准。
