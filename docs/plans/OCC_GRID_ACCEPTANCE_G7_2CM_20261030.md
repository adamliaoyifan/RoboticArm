# 自研Voxel grid验收 gen7：2cm代表目标，2/5cm矩阵

- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- generation: 7
- owner: eng/claude/glm-5.3
- product_revision: 6f75902202081ee59077faac62289400adfb627d
- baseline_revision: b33dda919521ef043fa9e30b07dcc79c87f76fce
- inherited_plan_revision: 4a09389888fe6f8ddbab48e8a14219b1ff52239e
- inherited_plan: docs/plans/OCC_GRID_ACCEPTANCE_G6_CLAUDE_20261030.md
- supersedes: gen6后续执行目标；历史证据/Claim/失败保留

## 1. 本代差异与继承

用户确认两条线均不再测试1cm，以2cm为可接受代表目标。本文件与 `OCC_RESOLUTION_TARGET_2CM_20261030.md` 覆盖gen6关于分辨率/组数的要求；gen6所有未明确变更的时延、资源、正确性、样本、步骤、输入来源、隔离、判官、失败取证和预算要求原样继承，不另行放宽。

Claude先在gen6回帖StopAck：不再启动旧组，已有组保存证据并按原超时结束，列PID/命令/锁、工具与dirty、已耗测量秒数、剩余额度、case完成清单。复用现有 `/tmp/elfin-occ-grid-acceptance-claude-g6` 和独立候选/b33 copy install，不新建worktree、不重做无影响部署。只有涉及评测配置/工具改变的工件重冻结；冻结产品不改。gen7可做轻量目录准备，StopAck与实际进程/锁复核前禁止新测量；旧OPEN行至StopAck才移除。

## 2. 完整新矩阵

从gen6 CASE_CATALOG及其原始manifest按数值resolution过滤，保留0.02/0.05，排除0.01；保留逐ID来源SHA与理由，旧完整表不得覆盖。不得按字符串随意删除包含数字1的用例，也不能用预期计数替代逐ID映射。

| 单元 | 新范围与数量 | 每组样本与门（继承gen6） |
|---|---|---|
| oracle | 2/5cm×3箱数×2hull×20seed×500事务=120000行 | 两侧相同journal；逐字段及约定float位模式零差异，缺/重行拒绝 |
| 查询 | 2/5cm×0/5/20箱×两hull=12组 | 各接口3预热+30评分；ROI/frontier/instance100/250ms、summary50/200、count1/5、distance1点5/20和256点50/200、surface/snapshot100/250 |
| 写入 | 原72组剔除1cm24组，保留48组 | 原输入维度不变；3预热+30配对、AB/BA各15；P95≤b33×1.10、max≤b33max×1.20+20ms、事实一致 |
| 取证 | 原198项剔除1cm66项，保留132项 | 每项30对ON/OFF，AB/BA各15；P95比≤1.10、join完整且dropped/failed=0 |
| 旧placement响应 | 原18组/612对剔除1cm6组/204对，保留12组/408对 | 原res/boxes/hit-or-fallback逐字段合同，不与hull维度混淆 |
| 真实placement | 2/5cm×3箱数×两hull=12组 | 3预热+30评分；RTT5/20s、目标地图获取≤2s、surface/snapshot100/250ms |
| 资源代表 | 2cm倒角0/20箱各一组 | 各3预热+30提交、1次FULL resync；部署RSS≤2GiB、派生唯一资源≤512MiB、init/reset≤30s |
| 持续业务 | 2cm倒角0/5/20箱×3次×120s，seed0/1/2 | 原完成驱动writer、1Hz120查询链、每窗20次CURRENT/placement；所有原逐窗门不变 |
| 旧压力 | 2cm倒角0/5/20箱，C1/C2各120s一次 | C1完成驱动写/C2固定2Hz240计划；原summary10Hz/ROI5Hz/distance256 2Hz/surface1Hz、miss与排空门不变 |
| 增量/生命周期 | 原六类各30事务、原全部故障语义 | 显式代表配置2cm；每个原语义仍须覆盖，不因删除1cm重复组而删除故障类型 |

oracle120000必须从原journal逐行解析resolution证明60000行1cm排除，不能只给公式。capture132/response408/writes48/placement12已通过gen6 catalog逐ID分辨率计数核对，owner仍须冻结最终表、源hash和排除表。

原316回归不强制机械重跑。保留已经有效的全部历史316通过；本代部署影响检查仍做。明确参数化的1cm重复项可不再运行，登记out_of_scope；若某语义仅在硬编码1cm fixture存在，则在测试工具中增加对应2cm正负对照，以独立oracle重算期望，不删除语义门、不改产品。记录旧node_id→复用/排除/2cm替代node_id及理由，不能固定声称仍“316个本代新样本”。

## 3. 输入与统计冻结

执行配置显式0.02，拒绝隐式默认0.01；每次进程实际加载config digest、分辨率、shape和fixture SHA必须入证，两侧一致。1cm切2cm时保持物理几何、有效点数和射线、不减箱数/点数/outside/crossing。固定cell ROI与物理ROI分别报告；沿上位目标第4节，不重新解释旧O5范围。

先修改工具的matrix过滤/显式代表参数和新证据根，自测后冻结工具SHA。行为测试至少覆盖：0.01不会进入新矩阵；0.02/0.05两侧完整集合与旧表筛选全等；数量正确但漏/重复ID会失败；代表业务/压力/资源实际进程接收到0.02（行为检查，不能只查源字符串）；不匹配shape/config被拒；排除项不计PASS；缺样/错误响应仍失败；测试输出不覆盖gen6。其余判官行为测试按gen6继承，可复用同SHA无影响证据。

COMMANDS.sh在运行前冻结真实参数，不虚构现有CLI支持；需要新参数先实现并验证--help和实跑预演。保留原单元顺序：准备→正确性/资源→查询/真实业务→写入/取证→持续业务→旧压力→收口。已有同产品/工具判据/输入配置一致的2/5cm覆盖可复用并标来源；不得把1cm样本重命名。首查/重建/缓存分列、nearest-rank P95、逐组AND、各30成功最低样本门均不变。

## 4. 预算与交付

沿gen4–gen6共享16小时剩余额度，先减已实际消耗，不重新发额度；锁等待不计。原性能锁、操作30s/子进程30分钟/排空30s、失败前10秒后5秒取证、每run8GiB/总32GiB和停止规则照旧。仅用户取消的1cm组记out_of_scope；预算不足未完成的2/5cm组仍not_evaluated。

新证据根 `docs/status/evidence/occ_owner_b1/occ_grid_acceptance_g7_2cm_20261030/`，旧gen6只读。除gen6必需交付，再加 `RESOLUTION_SCOPE.json`（用户变更、逐IDincluded/excluded/复用/替代和来源）、`BUDGET_CARRYOVER.json`、`CONFIG_EFFECTIVE.json`。报告四项仍分别判：功能、实际业务、旧压力、资源恢复；完整结论限定2cm代表及2/5cm矩阵。历史1cmFAIL不改判、不再阻塞新分辨率范围，任何新范围必需门FAIL/not_evaluated仍不能签通过。不解锁公平比较/生产迁移。

## 5. 派工

| ID | Owner | Generation/base | Plan | Acceptance/tests | Commit/evidence |
|---|---|---|---|---|---|
| G4-B-FULL-ACCEPTANCE | claude/glm-5.3 | 7 / 6f75902202081ee59077faac62289400adfb627d | 本文件提交由thread精确绑定；继承4a093898 | 第2–4节及gen6未改门；先StopAck、矩阵与配置行为验证、再计时 | 工具提交SHA、产品diff、gen7角色记录/证据/Result |

同一owner可在确认旧任务已停止后自行登记StopAck并进入gen7，无需重复用户审批。agent_start/complete按新代执行，工程交付pass与产品验收通过分开。
