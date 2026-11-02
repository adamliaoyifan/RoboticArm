# 2026-11-03 -- gen4复核与最终集成开放

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

接受gen4针对1938复核七案的实现补正与有界资格证据；开放Q-20261102-2 / S3 gen3由eng/claude/glm-5.3领取，先完成下述有限计分前集成检查，再自动进入正式评分。不是宣布产品通过，也不是允许原样运行存在汇总漏洞的score。停止新开implementation gen5的循环。S3 owner负责限定修正、验证、冻结、正式测量和最终报告，未完成检查不得产生计分样本，无需再等一次泛化reviews才能执行已授权评分。

候选产品/工具起点3bb26935f091055d77b9defa87511489cbfb32e6；交付证据头75ec78fae792910be1de24ef4c1809dbf17d6199；worktree /tmp/elfin-occ-freespace-impl-g4。

## Verification

- 独立逐字节FREEZE检查33/33匹配；worktree clean。
- g2/g3/g4相关pytest：53 passed / 1 skipped，42.54s。未声称重跑owner全部117测试。
- audit_g4_red.py after独立输出：2健康对照绿，1938七案7/7拒绝。
- freespace_verdict.judge_evidence_root从盘重算资格14行pass、正式6行not_evaluated；包含四操作O9配对样本重算，未重新计时O9。比值1.001782/0.985820/0.985717/0.992181。
- 独立运行冻结observer_freespace_behavior退出0（freespace behavior OK）；所有权受控反例盘上复算通过；有限预演15读轮、2reset不冒充正式1000/100或120分钟。
- review.py新增正式汇总反例：O8/O5/O6只有summary.json、passed=total分别为0、true、30，无任何原始样本，9/9被_row_formal误判pass。这是实测验收工具缺口，不是产品性能失败。
- 源码确认COMMANDS score只给P/R/L传420分钟预算，后续O8/O5/O6没有同一总预算；已有verdict将正式not_evaluated→pass/fail当状态篡改，也没有明确运行版本切换。需下列集成修正，避免长窗后才发现收不了尾。

## Acceptance

**S3计分前限定检查（原合同，不增行/阈值）**：

1. 正式O8/O5/O6接实际调度器产物及原始数据判官，禁止只读passed/total。固定原计划组数/窗数/重复、时长及内容依赖。上述9例全部拒绝，足额健康对照通过，短样本拒绝；正式失败不能因缺文件被洗成未运行。P/R/L既有逐记录判官沿用。O5失败停止O6。
2. 实现一次明确的预演→正式报告版本切换：保留原verdict和资格证据，正式报告关联run_id/冻结SHA/原始输入，首次可从not_evaluated转为实测结果；同一run的事后篡改仍被拒。最终judge返回码体现失败，预算未跑项保持not_evaluated。用无长窗的健康/失败/缺样/篡改fixture验证整个score→artifact→judge→20行报告，不只单测辅助函数。
3. 外层统一8小时预算（等性能锁不计），一次持有/tmp/elfin-occ-backend-perf.lock的正式串行测量；核查实际锁持有层防重复flock。P失败停止R/L，安全O8/旧矩阵按原顺序与预算可继续；O5失败停止O6。不给120分钟L缩水；余量不足的阶段不启动，保存not_evaluated原因。失败证据落盘后才拆卸。冻结完整正式输入manifest（含seed、周期stamp映射和最终hash，不能仅以六帧预演文件代替对正式负载的说明）。环境/频率/线程/外部负载按原计划记录。
4. 只允许以上评测汇总/入口修正，提交工具并更新冻结，记录产品二进制SHA不变证据。若采集、计时、执行热路径或产品发生变化，按原合同重测受影响资格/O9；纯离线汇总变化可绑定原四操作O9及未变采集/二进制SHA，不为文档变化盲目重测。完成同源有界预演与上述反例后，owner自动执行正式评分；无需新增代次。若另有真正阻断，不假称通过，记录具体失败并按有限预算收口。

**正式执行与结束**：仍按8caa0d44完整合同，P/H九窗→R→L120分钟→O8→O5→O6。产品pass/fail/not_evaluated与工程执行完成分列；主目标与旧矩阵分列。单候选失败保留；本次不要求修成全绿，不静默重跑/改门限/拼版本。任何进一步产品修复另列后续工作，最终报告可用真实fail结束。20行固定，未覆盖不得计pass。

## Binding

- plan_revision: 8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451
- FREEZE.json SHA256: 8d1dbd6beb46dd24718eae883501633bdfc74dc3991bdd4a895ab55e7568cb83
- COMMANDS.sh SHA256: 7e79edad40d7d13fdc0adf90b8e0917a0b07e8b20035a1fbd3373835d401af8b
- FREEZE.inputs规范JSON（sort_keys=True,separators=(',',':')）SHA256: f1f0a061660339e174d61e093aa3d6f015eb0b4254e6ffb027fb0f451f55ad02
- 上述为已复核起点，S3限定工具修正后追加新工具SHA/正式manifest关联，不覆盖历史资格证据。

## Pointers

- docs/status/evidence/occ_freespace_final_20261102/implementation_g4_review/
- docs/status/evidence/occ_freespace_final_20261102/implementation_g4/COVERAGE.md
- docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
- docs/agents/discuss/2026-11-02_0702_occ-freespace-final-claude.md
