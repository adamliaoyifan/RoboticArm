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

## Reply -- eng/claude -- 2026-10-31 08:55 -- claude-code/glm-5.3

收到驳回结论与四项发现，**全部接受**，并已逐条在代码/证据中复核属实：

- **P1 预算语义**：属实。原路径 `_Work.admits`（cargo_distance_query.py:173-176）每 pop 查时钟半（:357 调用点），JIT 化后时钟仅剩 :461 点边界检查，核内只查计数半（distance_kernels_jit.py:170/208）——点内超期可返回 OK。
- **P2 冷构建归因**：属实。排演脚本未提交修改以**请求级** `volumes` 缺键记"构建"，而 `_jit_tables` :229 **代级缓存**命中时不构建——spike_dossiers 的 builds_overlapping 把每次请求首查都记成了 build，不能证明冷构建；且该脚本修改未 commit 绑定。
- **P2 汇总混场**：属实。190.6ms 出自 c2_rehearsal_180s_dossier.json（该场首笔未剔出样本，max_ms=190.65 即首笔本身）；120s 场首笔实为 179.4ms。稳态 max 64.63 超 60 上限，1/239 只能称观测超限比例。bench_B/C.json 拼接多 JSON 确认（json.load @char373 失败）。
- **裁定接受**：C2 不改判通过（冷启动单列、不豁免稳态 max≤60）；numba 先钉版本+部署环境、不 vendor、有/无 JIT 分别验证（功能回退≠性能达标）；收口复测暂缓，先修预算边界/取证/版本绑定再冻结工件。

**修复 WBS**（产品对象 e9f71861 确认；9ec038a4 仅为送审文档）：
- **W4(P1)**：kernel_nearest 增 `budget_ns`（剩余量，可为负）参数，在两准入位与计数半同点查时钟；`_nearest_jit` 以模块钟算 remaining 派发（时钟桩前推→remaining≤0→核内首检即断，桥接受控时钟复现）；新增自校准真实时钟 + 受控时钟两类边界测试；基准复跑披露时钟开销。
- **W5(P2a)**：排演插桩改三态（代级 hit/miss + 段 build/hit），dossier 可证"真冷构建 vs 缓存命中"；脚本全部 commit 绑定。
- **W6(P2b)**：bench 写出器改单 JSON、bench_B/C 原地转换；C2 120s **单场规范重跑**，此后引用数字只出自该文件，表述改观测 n/N；summary 改写。
- **W7**：VERSIONS.json（numba/llvmlite/numpy/python/platform）+ runbook 钉版本；功能门 JIT=1/0 双跑等价、性能门 JIT=1 判定。
- **W8（条件）**：若重跑稳态 max 仍>60——poller 预构建代级缓存（兼削冷启动）→ apply 让出间隔 8→4；仍不达诚实 FAIL。
- **W9**：冻结 manifest 后收口复测（12 组矩阵 50ms 门 + oracle 双旗标 + 回归 70）。

完成后本线程回帖修复结果与复测判读。

## Result -- eng/claude -- 2026-10-31 09:55 -- claude-code/glm-5.3

四项修复 + 版本绑定 + 冻结 + 收口复测全部完成（提交链 310e2fd7→e4dcbc97→9bb06d23→f4080eb2，总结 rev2 已更新）。

**修复**
- **P1（310e2fd7）**：kernel_nearest 以剩余量 `budget_ns` 入参（负=预超期→核首检即断，与原路径首败 admits 同界同 bound）；两准入位与计数半同点查时钟，objmode 读 `_clock_ns`（~255ns、可桩），几何步长 32→512（超限界 ≤~80µs 稳态/≤0.2ms 解码相）。新增 3 类边界测试（**含评审受控钟复现的双路径对偶**）；golden 冻结钟化（负载下首查表构建曾能翻转场景，输出零变化）。正确性成本：基准 7.53/8.18→8.48/9.09ms（+11%，四档全录）。
- **P2 取证（e4dcbc97+f4080eb2）**：三态插桩（gen_hit/assemble+段 built-vs-reused），首笔无论成败均出档案；脚本全部 commit 绑定。
- **P2 混场**：bench_B/C 转 `{"runs":[...]}`；C2 单场规范重跑为唯一引用源，弃用标记落 README。

**取证推翻旧叙事并驱动 W8（9bb06d23）**：实测 gen_hit 0/assemble 480——写流下每代 ~120/175 块段失效，7-9ms/态装配落在**每代首查询线程**（非"一次性冷构建"）。修复：replica poller 换版前 `prewarm_tables` 预热。**C2 规范场（120s 单场）：distance 中位 27.78→9.69、max 63.07→50.99 ≤60 门 PASS（观测 0/239 超限）**；确认场 10.59/54.62 亦过；冷启动首笔 163.2ms 单列（档案：两态 gen_hit=查询线程零装配）。**W4 野外实证：修复前首笔 202.9ms 会静默 OK，规范场确认超预算即耗尽。**

**版本绑定（W7）**：VERSIONS.json（numba 0.68.0/llvmlite 0.50.0/numpy 2.2.6/py3.10.12）+ runbook 钉版（不 vendor）；功能门 JIT=0/1 双跑 22×2 全绿（golden 路径无关=双路径逐位相等）。

**收口复测（closure_retest/，@f4080eb2+同步 install+PERF_LOCK）**：回归 **70/70**；取证抽查 **3/3 ≤1.10**；业务窗抽查 **PASS**（29.7/73.4ms）；**查询矩阵 12 组：distance_256@2cm 仍 FAIL 4/12 臂（近秩 P95 60.3–71.5 超 50）——但中位 36.5–40.9→23.4–25.9ms、>40ms 样本 138→16/360**；残余尾部=gen7 同形外部单样本尖峰（gen7 **无 JIT** 亦有 67–75ms；frontier 未触碰路径也有 101 边缘），判定公式显式声明于 MANIFEST；gen7 手算口径 52.6–57.5 未复现，原始 samples.jsonl 在证。另：曾试 owner 侧 store publish 预构建削矩阵尾部，因加重已裁定的写入 cold 门而**回滚未入提交**。

**请裁定**：2cm distance_256 残余尾部口径（外部尖峰、中位已 23-26ms）——维持 FAIL 记录不改判，或按尾部归因另行裁定。

## Open

- 等 reviews：四项修复与 C2/收口判读审查、残余尾部口径裁定。
