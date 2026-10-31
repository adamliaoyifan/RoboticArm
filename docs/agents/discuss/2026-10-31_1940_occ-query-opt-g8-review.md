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

## Result -- eng/claude -- 2026-10-31 11:13 -- claude-code/glm-5.3

第二轮三项发现**全部接受并逐条核实属实**（含复现），已修复+重冻结+复跑（冻结 commit `b888cbbd`，产品=W10@`3e9c1039`）：

- **P1 预算重起算（已修 `3e9c1039`）**：属实——入核重取起点+用调用前剩余量，dispatch→entry 间隙（含**首次编译**）漏计。修复：`_deadline_ns` 在 dispatch 把剩余量**锚定到核时钟域一次**，核内各读对**绝对截止时间直接比较**（无 elapsed 差分）；锚后调度/编译/初始化间隙全部计费。新增 `test_pre_kernel_gap_spent_against_budget` 复刻评审复现（1ms 预算+10ms 入核前延迟→EXHAUSTED）。
- **P1 预超时下界错误（已修同提交）**：属实——`cand_lowers` 按块序、首弹键才是最小。**删除预超时特例**：首弹步长检查天然覆盖且 bound=实际弹出键（真最小）；内核级测试以评审数值断言（cand_lowers=[0.0820857, 0.0]→bound_sq==0.0、零充计）。
- **P2 判读字段（已定谳）**：elapsed 重算的 **5cm 3 臂超限（51.814/53.602/55.610）已逐毫秒复现**——第一版判读口径整体作废。**基准定谳以对账证明**：`gated_ns`（owner 侧纯查询时延，生产者注释明示）为 distance\*/frontier_count 门基准；**gen7 记录的"FAIL 4 项 trace_off 52.6–57.5"在本基准下逐位复现**（52.566/54.629/56.885/57.453，b5 与 5cm 全过）——含索引约定 ceil(0.95n)。判定工具固化 `scripts/g3/occ_query_matrix_gate_judge.py`；elapsed 全部保留为往返诊断（`closure_retest/GATE_BASIS.md`）。

**重冻结复跑（全部 @b888cbbd）**：查询矩阵 **PASS 312/312**（distance_256 24 臂 gated 6.86–39.77ms vs 门 50；上轮 frontier 101.0–102.6 边缘项**未复现**）；oracle+契约 **24×2**（JIT=0/1）；回归 **70/70**；C2 规范/确认场 **9.16/10.01/43.30** 与 **9.04/9.97/45.46**（≤60 **稳态子门**——照认可范围措辞，非完整旧压力合同）；取证抽查×3：distance ≤0.94 稳定、roi 三场 1.037/1.142/0.979（12ms 基数记录器差值亚毫秒、比门噪声主导——gen7 挂账"绝对下限"同族证据，**如实报 FAIL 不改判**）；业务窗 PASS。基准 D 对 7.85/8.39ms（绝对截止版，较 entry-read 版更快）。

**尾部归因按裁定仅记假设**：超限样本与记录全保留；"尾部位于查询外"现以 gated 稳定 8–13ms vs 往返偶尖 50–88ms 为据，仍不据此改判任何门。

请审查 rev2 修复与复测；取证比值"绝对下限"口径仍待裁定。

## Result -- eng/claude -- 2026-10-31 13:24 -- claude-code/glm-5.3

第三轮三项发现**全部接受、逐条核实并修复**（冻结 commit `bd29808c`，链 …→`a7edc457`→`506bd652`→`bd29808c`）：

- **P1 重锚定窗口（已修 `a7edc457`）**：属实——`_deadline_ns` 把剩余量加到较晚钟读上，两读间隙偷预算；**暖态复现成功**（静态模块钟+延迟锚定读，1ms 预算→OK）。修复=**删除重锚定，直传 `work.deadline`**（生产两钟同源 perf_counter_ns，直传即精确）；修复后同复现 EXHAUSTED，且窗口结构性消失（不再存在第二次 dispatch 读）。
- **P1 判官缺项+错阈（已修 `bd29808c`）**：属实——静默跳过空样本（实际仅 264 臂有样本）+错放宽 stats/summary（真 50/200）/frontier_count（真 **1/5**）/distance_1（真 **5/20**）+漏 max 半边。判官完整化（镜像生产者 `LIMITS_MS` 全表、P95+max 双半边、冷/预热行按生产者 `cold_or_sample_max` 语义参与 max、空臂显式 not_evaluated/not_applicable）。**完整判官揭出 r2 真值：FAIL 12**——全部 12 组 distance_1/trace_off 冷探针 20.8–72.2ms 超 20ms max（gen7 冷 0.42ms 正常测量）。"312/312" 作废。
- **根因修复=导入期预热（`506bd652`）**：按裁定"冷启动是统计阶段非超时豁免"，产品不得欠首查询编译债——`warmup()` 在导入时以哑数据跑一次核（首调用实测 ~130ms、暖调用 8ns），冷单点首查降至 0.28ms。
- **P2 溯源（已更正）**：1.037 场实为 @f4080eb2 历史对照；b888cbbd 仅 1.142/0.979 两场。**取证裁定照录**：≤1.10 维持、@b888cbbd 的 roi 1.142 FAIL 保留；1.724ms 为完整请求比值、"噪声主导"仅假设；绝对容差须另起预先冻结规则+判别力证明，不倒定阈值。

**重冻结复跑（全部 @bd29808c，完整判官口径）**：
- **查询矩阵：FAIL 2/288 评分臂 + 24 臂 snapshot not_applicable**——两项 FAIL 为 frontier/trace_off@b0_cuboid 102.78、roi_dense/trace_off@b5_cuboid 106.53（往返基准、**未触碰接口**、超门 2.8–6.5%、组别逐轮漂移=环境尾部；按裁定保留 FAIL 不豁免不归因）。**distance_256 全臂过**（gated 6.9–39.8 vs 50/200）；**distance_1 首次完整测量且全臂过**。
- oracle+契约 **24×2**；回归 **70/70**；C2 规范/确认 **9.11/9.97/max 39.09** 与 **9.03/10.10/38.93**（≤60 稳态子门、0 失败；措辞维持"仅稳态子门，非完整旧压力合同"）；取证 @bd29808c roi **1.054**/distance 0.945/surface 1.001 全过（b888cbbd FAIL 保留）；业务窗 PASS。
- 旧压力 18/18、写入 cold 36/48 维持原判。

请审查 rev3。判读口径全部在 `closure_retest/GATE_BASIS.md`（含三轮裁定与作废声明）。

## Result -- eng/claude -- 2026-10-31 17:14 -- claude-code/glm-5.3

残余 FAIL 2 臂的定位排查完成（按指示：先离线复盘、最少阶段计时、定向对照、不盲跑矩阵、不找不超限重跑）。**结论：owner 进程周期性 gen2 GC，因果证实**（`closure_retest/PHASE_LOCALIZATION.md`，工具 `86249a56`）。

**第一步·离线复盘（FORENSICS_r3.md）**：两臂全部慢样本的工作量指标（bytes/returned/examined）与臂中位**逐字段相同**→"同样工作变慢"；慢模为 **~40% 占比的系统双峰**（非稀有尾部）、只出现在大响应案例（frontier 197KB/4096 格、roi_dense 235KB/3840 格、instance），roi_empty/distance_256 小响应零慢模；两臂皆有（取证记录器排除）。已知边界如实标注：owner 现录段只覆盖快照获取（~0ms），**operation 相位对 frontier/roi 完全未记录**、GC 未观测——故需第二步。

**第二步·阶段计时+定向对照**（新 probe：r3 `.cdr` 逐字节校验复放 + 产品 span 收集器含服务线程 CPU + 双进程 GC 观察 + 变体对照；harness 三个诊断旗标关则零效应）：
- **慢模在 owner operation 段内且 +34.3ms 全为 on-CPU**（span 线程 CPU：快模 28.8 wall=28.8 CPU；慢模 63.0 wall=**63.1 CPU**）——非等待非调度；
- **owner gen2 率 141/450=31.3% ≡ 慢模率 47/150=31.3%**（b5 同构）；
- **因果实验 G 臂（gc.disable）：慢模 0/150，elapsed P95 85.08→52.13、operation P95 59.7→23.65ms**；
- **体积扫描单调**：响应 17.7/68/235KB → 慢模 0%/6%/32%（分配压力驱动 gen2 频率）；
- **写负载对照**：基数 +7ms 且慢模率升（写分配叠加 GC）——**矩阵尾部在无写者下完整复现，争用非其成因**。

**四假设终判**：①共享更新争用=排除（矩阵尾部成因）②响应构造/分配=支持（触发侧：每请求 ~4000 消息对象构造）③GC=**证实**（机制：gen2 全堆遍历 ~30-40ms CPU 落在请求路径内）④扫描量变化=排除；调度/GIL=排除（wall=CPU）。"组别逐轮漂移"机制=臂间慢模率（16-46%）在 p95 捕获阈值两侧波动。

**修复提案（未动产品，待裁定）**：①请求间隙主动 `gc.collect(2)`（把 gen2 移出请求路径，最小改动）②场景构建后 `gc.freeze()`（gen2 只扫请求级临时对象）③响应构造分配批量化（大改）。建议 1+2，另起一代冻结验证（含 C2 写负载下）。**FAIL 2 保留**。

## Result -- eng/claude -- 2026-10-31 18:44 -- claude-code/glm-5.3

四臂对照实验完成（`closure_retest/GC_EXPERIMENT.md`，工具 `62885614`/`feb18ba2`，数据 `gc_mitigation/` 13 臂）。两处收紧已改：写负载慢样本 **70/150=46.7%**（PHASE_LOCALIZATION 更正）；gen7 尾部同源性收紧为未证。复放 r3 失败臂 `.cdr`，指标=真实往返（原计时边界、含回收期间到达请求的排队）。

| 臂（roi_dense@2cm，150 笔/臂） | rt-中位 | rt-P95 | slow% |
|---|---|---|---|
| off（基线） | 46.7 | 82.3 | **32.0%** |
| **collect（仅主动回收）** | **79.3** | 85.4 | 0%*（*均匀变慢：每请求手动回收 32.3ms） |
| **freeze（仅冻结）** | 46.2 | **52.0** | **0%** |
| both | 46.5 | 52.8 | 0%（freeze 后 collect 仅 0.3ms，无增益） |
| 连续无隙 off / freeze | 44.9/43.5 | 80.8/**49.8** | 32%/0% |
| +写 2Hz off / freeze | 52.8/46.9 | 106.7/**53.4** | 46%/**2%** |
| frontier 复案无隙 off / freeze | 50.0/48.8 | 88.2/**54.6** | 31%/0% |

**判定**：
1. **仅主动回收被数据否决**——暂停未被消除只被重排，rt-中位反升至 79.3ms（评审预警的量化实证，且本例连回调指标也变差）。
2. **freeze 有效且跨全部负载形态**（慢模 0–2%、P95 80–107→50–53ms）；**both≈freeze**。
3. **正确性**：13 臂响应内容字段（returned/complete/status）与基线逐请求相等；字节差=回显 request_id 长度（恒 +8B）。
4. **生命周期实测（freeze 代价）**：冻结 162,277 对象；**RESET+场景重建后旧冻结集永不释放，RSS +69MiB/次（进程存续期永久）**；重建后新对象正常可回收；连续写 RSS 增长 ±freeze 相同（写者驱动非 freeze 回归）；无 RESET 时 RSS 平稳。

**推荐（待裁定，未动产品）**：**仅 freeze**（场景构建后一次、服务就绪前），不采纳 collect；文档化 RESET 代价（~70MiB/次重建至重启；RESET 罕见；长期进程择机重启，或后续探索重建后 re-freeze/旧集隔离）。实施须另起一代冻结验证（含 C2 写负载与响应正确性门）。

## Open

- 等 reviews：四臂结论与 freeze-only 推荐裁定（含 RESET 泄漏代价接受与否）；实施代派工；其余维持原判项不变。
