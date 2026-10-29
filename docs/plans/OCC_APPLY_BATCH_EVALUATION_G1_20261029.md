# ApplyBatch 冻结版本代表评测与资源复核 G1

- owner: eng/cursor/grok-4.7
- product_revision: 4c8f1862ec357cb01616f4b04be636227461cf44
- baseline_revision: c4407bc01f3efa3d17071241e919fb623679d71f
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-APPLY-EVALUATION
- generation: 1

## 1. 目的与边界

独立复核最终 ApplyBatch 版本：概率/查询结果是否一致、代表负载是否达到原延迟标准、首次查询与资源成本是多少。执行者 Cursor Grok 4.7，自行完成测量工具准备、测试、冻结、测量、归档和任务关闭。

沿用现有 worktree `/tmp/elfin-occ-octomap-perf-cursor-g1`，不建立新 worktree。产品固定为上述 SHA；对照是关闭本轮 lazy 批维护的 A 版本。只允许修改包内既有测试类、评测脚本和文档，禁止为获得通过修改产品、概率策略、射线、网格或门限。只测 1 cm，discretize=false，光学 Z=0.52–6 m，完整射线、无 maxrange；不运行 Gazebo、机械臂或迁移生产。

本任务是代表性能与正确性复测，O5 对应指标可逐项给结论；不能宣称完整正式 O5/O6/O7 或产品通过。原计划第12节还列有 O6 组合和 O7 心跳合同未冻结项，本任务不悄悄替它们作裁定。OCTOMAP-OBSERVATION 保持 open，不解锁 OCC-BACKEND-COMPARISON。

## 2. 来源与版本冻结

- 上轮：`docs/status/evidence/occ_owner_b1/occ_apply_batch_20261029/run01/FINAL_REPORT.md`、`VERSIONS.json`、`REPLAY.md`。
- 原门限：`/tmp/elfin-occ-dual-backend-plan/docs/plans/OCC_OCTOMAP_OBSERVER_G1_20261026.md` 第8–12节。把本次所读文件及 SHA256 存入新证据，避免路径漂移。
- 工具：包内 `test/benchmark/fusion_benchmark.py` 的 `FusionBenchmarkSuite`、`fusion_report.py`，既有 `FusionProfileChecks`，CLI 与 pytest。继续扩展它们，不写一次性评测程序。
- 输入：主工作区 `docs/status/evidence/occ_owner_b1/occ_octomap_perf_diag_g1_20261026/run01/fixtures/` 四个 `o5_b{0,20}_c{30000,100000}`；geometry/config 取上轮 run01/config 下两份 YAML，逐字节复用。
- 使用同一主机、Release -O3 -DNDEBUG、OctoMap 1.9.8，记录 CPU/内存/内核/编译器/链接库、频率策略、并发负载、二进制与输入 SHA256。不可混用系统 1.9.7。
- 基线和候选均从 git archive 导出只读源码构建（不是新 worktree）；工具可单独新提交。候选 runtime 必须与产品 SHA 一致，记录测试工具 SHA 和产品 SHA 两个身份，不以新测试提交冒充被测产品。
- 测量前后记录包内 status/diff；产品脏或二进制/hash 改变则受影响窗口不可比较，保留记录，不能静默换版。

## 3. 矩阵、样本和判定

| 项目 | 固定样本/过程 | 判定或用途 |
|---|---|---|
| 正确性预检 | 候选 Release 全部 CTest；既有 profile CLI/report/fusion benchmark/eval CLI pytest | 全部通过；缺项明确记录，不按旧报告代替 |
| 主对照 Off | 0/20箱 ×30k/100k，4组×3重复；每侧新 store，3预热+30计分，同输入，AB/BA交替 | 每对33帧输入身份、概率 canonical digest、ROI/Surface内容一致；任一 mismatch 停后续计分并留证 |
| 冷启动查询 | 4组各30个新进程；均用该组 manifest 首帧，不预热；一次 integrate 后 acquire/ROI/首次 surface，再立即第二次 surface | 首次与缓存分列；每组独立 P95/max；新进程不声称 OS cache 冷 |
| Stages 归因 | 候选 Off/Stages 四组各1对，3+30帧，查询与主对照完全一致 | P95比≤1.10、max比≤1.20 才可作低扰动归因；否则仅机制观察，不否决 Off 成绩 |
| 资源 | 候选 Off 四组各1窗，3+30帧，关闭 checkpoint/摘要序列化，保持 acquire/ROI/surface | 进程峰值RSS、CPU时间、树节点/估计字节、known/changed、块复制数，描述性；不设置新增资源门 |

旧延迟标准（毫秒）：30k integrate P95≤500/max≤1000；100k≤2000/5000；acquire、ROI、surface 每类 P95≤100/max≤250。首次 surface 和更新后重建、立即再次查询分别统计，缓存成绩不得代替首次成本。冷启动 integrate/进程启动时间另列，不混入预热积分窗。

每个30样本窗口独立排序，P95=第29个，中位=第15/16个平均，max=第30个；阈值相等通过。12窗按 AND 判定“本矩阵达标”，不能把90样本合并掩盖单窗失败，也不从此推断长期成功率。这是每窗经验延迟要求，不是每帧100%低于P95阈值。n<30、摘要缺失、失败捕获缺失标 not_evaluated/incomplete，不报通过。性能超标必须完成当前30样本留存，不删异常点、不替换失败窗；语义错误停止依赖测量。

树物理结构/raw digest、node_count 同时报告；lazy/prune 引起的结构差异不自动等于概率差异，须 canonical 概率及查询内容一致。此次没有硬件部署评测，Ryzen 的CPU/RSS不得换算为 AGX Orin 性能或 YOLO 共存余量。

## 4. 工具工作与可执行流程

1. 先在主工作区调用 `scripts/agent_start.sh --thread <派发线程文件名> --role eng --agent cursor --model grok-4.7 --cli cursor`。
2. 核对版本，导出两个 archive，分别 `cmake -S <package> -B <build> -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release`、`cmake --build <build> -j4`。候选 `ctest --test-dir <build> --output-on-failure`；用 OBSERVER_PROFILE_CLI 指向候选运行上述四组 Python 测试文件。ROS 节点本轮不重测、不声称运行覆盖。
3. 在 `FusionBenchmarkSuite` 增加持久的 `cold` campaign：每组30个独立子进程，每进程一帧并保留两次 surface，输出逐样本输入/hash/状态/时延/digest。可复用 run_window/process cleanup，不用临时循环脚本绕开 runner。cold 每组和基线同输入配对（共120对），不预热；诊断可另跑 Stages 验证 cache_hit/scanned_cells，不能给 Off 填造计数。
4. pytest 增加 cold 的30进程隔离、一次帧、首次与第二次分开、29样本拒绝、摘要缺失、超时进程清理。沿用已存在阈值相等、BA、合并掩盖失败测试，不重复搭另一套判定器。诊断计数若不可得则注明，不为补计数改产品。
5. 先非计分预演验证 argv、输入/配置 hash、输出字段、摘要对齐、失败证据和清理。准备限60分钟；失败修 runner 后可重复，旧失败记录不删。之后提交/冻结 runner，生成 `COMMANDS.sh`，以 `--help` 和预演结果核对真实命令。
6. 正式使用以下参数模式（变量写入 COMMANDS.sh 为实际绝对路径）：

```bash
python3 "$RUNNER" --campaign baseline --repetitions 3 --require-content \
  --baseline-cli "$BASE" --candidate-cli "$CAND" --fixture-root "$FIXTURES" \
  --geometry "$GEOMETRY" --config "$CONFIG" --source-root "$PRODUCT" \
  --out "$EVIDENCE/paired_off" --timeout-s 900 --budget-s 10800
```

随后同一接口运行 `--campaign cold --repetitions 1 --require-content`（步骤3实现）；`--campaign stages --repetitions 1 --require-content` 两侧都指候选；`--campaign memory --repetitions 1` 只指候选、不要求摘要。四次 out 各不相同。整个测量累计预算3小时，不是每条命令3小时；调度器传入剩余秒数，等锁不计。每个新窗前检查剩余时间，不中途人为截断正常窗；单进程超时仍900秒。

锁 `/tmp/elfin-occ-backend-perf.lock`，使用 runner 已有锁；不嵌套锁导致自阻塞。等待用短轮询并报告，不并发跑重负载。LOCK_BUSY 不算性能失败。若预算结束，保留已完成窗口并对未开始项逐项 not_evaluated，不用少样本下结论。

## 5. 统计字段与边界

每帧记录 frame/request ID、时间戳、输入SHA、原始/Z有效/ULD合法端点数、rays、known/changed、状态，integrate端到端（过滤/变换/射线/融合/块维护）、查询时延、raw/canonical概率及查询摘要。序列化与摘要在计时外；保持双方查询开关一致。报告冷启动 integrate、第一/第二 surface 和预热后更新重建，revision未变帧不可算更新重建。

Stages 仅使用粗粒度 RayPreparation、Fusion、ApplyBatch 的 WriteLoop/InnerUpdate/Prune、BlockBatch、Finalization 与 residual，不加逐key时钟。ApplyBatch 包含父节点维护和 prune，不能把它们挪出计时。各阶段单独中位数不可相加冒充总体中位数。列出 index3/4填充帧与末两帧及 changed_keys，解释尾延迟，但不删前两计分帧。

资源报告标明 RSS 是整进程，不是树或单帧分配；CPU毫秒/帧与墙钟分开。若取可持续频率，只作串行回放测量范围描述，不能用1/P50宣称实机吞吐。缓存/读视图专属字节无法可靠归属则 not_measured，不拿RSS冒充。

## 6. 证据与失败保留

新目录 `docs/status/evidence/occ_owner_b1/occ_apply_evaluation_20261029/run01/`；若存在非空则使用下一个run号，不覆盖旧目录。

| 边界 | T1 | 触发 | T2/窗口 | 格式/上限 | 重放 |
|---|---|---|---|---|---|
| 输入→integrate | 输入hash/计数/ID/状态/时间 | 错误、概率差异、超时、延迟超标 | 新store至失败帧全部输入前缀（最多33帧），完整计分窗时延；语义失败不强行追加帧 | lossless YAML/JSONL、每例512MiB | 归档 argv.json 及 manifest，原CLI重放 |
| integrate→查询 | revision/内容摘要/查询类型/时间 | 摘要差异或查询超标 | 同一前缀+查询参数/输出；冷启动保留该1帧 | 同上，共用前缀不重复复制 | CLI相同查询开关 |
| worker→runner | PID/返回码/CPU/RSS/开始结束 | 崩溃/超时/样本短缺 | stdout/stderr、最后有效帧及退出/清理结果 | 文本+JSON | COMMANDS.sh对应case |

T3只在T2定位不足时另开非计分诊断，不污染Off。总留存≤8GiB，哈希引用的原fixture必须存在且可校验；不能只留摘要而丢重放输入。capture_complete/replay_possible 缺失或false须报告证据不完整。性能失败留满30帧分布；语义失败保存出错前缀与两侧输出，禁止盲重跑覆盖。先固化证据再清进程。

## 7. 交付与关闭

交付 FINAL_REPORT.md、VERSIONS/ENVIRONMENT、INPUT_MANIFEST、逐帧 JSONL/原始samples、每窗判定表、冷启动120对表、资源4组表、阶段资格表、COMMANDS.sh、工具提交、SHA256SUMS和TEARDOWN。可复核每个结论到原始样本；实际样本数单列。

报告最后三行固定区分：①正确性回归；②本轮代表性能（主窗/冷启动分别）；③完整产品验收仍未评估。可如实报告性能 fail，同时完成测量任务；完成任务不等于产品pass。主工作区写 eng 角色记录并在派发线程追加 Result，以 `scripts/agent_complete.sh` 关闭本子任务，不关闭父产品验收。禁止自动迁移、发布或修改生产配置。
