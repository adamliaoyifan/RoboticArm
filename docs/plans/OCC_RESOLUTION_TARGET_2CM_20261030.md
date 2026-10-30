# OCC 两条线测试目标调整：2cm 为代表目标，取消新增1cm测试

- date: 2026-10-30
- authority: 用户明确确认同时适用于OctoMap和自研Voxel grid
- representative_resolution_m: 0.02
- retained_matrix_resolutions_m: 0.02,0.05
- excluded_resolution_m: 0.01
- scope: 测试目标与执行配置；不改变冻结产品或产品默认参数

## 1. 新旧目标分账

用户不再要求1cm；代表性能、持续业务、过载/恢复、资源代表组及后续针对代表配置的诊断统一使用2cm。原矩阵只删除1cm维度，已有2cm与5cm保留，未要求新增原范围外的5cm实验。取消的是后续测试要求，不是把1cm失败改成通过。

历史产品SHA、工具/输入/配置SHA、分辨率、指标和判定均保留。排除项标 `out_of_scope_by_user_resolution_change`，不是PASS、失败修复或预算不足的not_evaluated。新报告必须称“2cm代表目标及2/5cm矩阵验收”，不得再声称满足原1/2/5cm完整合同；产品其它必需功能/资源/正确性门不豁免。

已完成的1cm实验不重跑、不删除；正在执行的旧代先保存当前组及失败证据，按原超时有界收尾，不启动新组，不杀无关进程。旧代STOP ACK后才能启动替代代次正式计时。不能把1cm结果改标签当成2cm结果，也不能按体素数推算2cm性能/容量。

## 2. OctoMap

冻结产品仍为 `4c8f1862ec357cb01616f4b04be636227461cf44`；接口取证工具 `57d8c0bfe23fee54920c89c06a2b6bdcb97de976`，已完成任务保持done，不重开。历史取证证明的锁依赖/入口缺口可作为机制证据；其具体时延、吞吐和失败率不外推为2cm结果。

| 后续目标 | 新范围 | 保留规则 |
|---|---|---|
| O5代表 | 2cm倒角、0/20箱、30k/100k；3预热+30评分，首读和重复分列 | 原时延门不变；正式O5仍未评估，物理ROI映射须在执行计划中固定 |
| O6矩阵 | 2/5cm ×0/5/20箱×两容器×30k/100k×outside .5/.9×cross 0/.5 =96组 | 每组原3+30及30冷启动规则、时延/启动/RSS/512MiB归属门不变；历史同版本有效96组优先复用，不重跑凑新报告 |
| O7正常 | 2cm倒角、30k、outside .5/cross0；0/5/20箱×3×120秒 | 2Hz输入240帧、1Hz120查询链、原排空/内容/健康/计数与延迟门不变 |
| O7过载/恢复 | 2cm20箱、10Hz60秒、30秒排空、2Hz30秒恢复 | 不预判10Hz能力；保留明确拒绝与publish→callback缺口分账，恢复首查语义不在此次自动改动 |
| 后续资源/接口诊断 | 代表组改2cm，旧5cm已有范围保留 | 资源归属unknown不因分辨率降低而通过，取证ON/OFF资格仍须核实 |

原O6唯一1cm时延失败属于旧目标历史失败，不再作为新分辨率范围的时延阻塞；读视图/缓存归属缺口仍阻塞完整资源验收。O7只有1cm历史结果，不足以判2cm通过。此次仅更新目标、通知Cursor，不自动启动新OctoMap评分、修复或重跑已完成接口取证；后续执行须冻结采用本目标的完整计划和manifest。公平比较仍暂停。

## 3. 自研Voxel grid

由 `G4-B-FULL-ACCEPTANCE` gen7替代已领取gen6后续执行，owner保持Claude/glm-5.3。完整变更合同见 `docs/plans/OCC_GRID_ACCEPTANCE_G7_2CM_20261030.md`。候选6f759022/b33产品不变，沿共同16小时剩余额度；已完成工具、独立copy install、回归和可追溯2/5cm证据按影响复用。

## 4. 输入、内容和正确性约束

只改评测显式resolution配置，重建必要2cm fixture与期望内容并记录SHA。保留物理ULD/箱体/传感器位置、每帧点数、outside/cross、有效Z过滤、射线全集、种子与次数；不能以点数减少或射线抽样冒充分辨率效果。网格shape/索引/geometry/config digest随实际配置重新计算。

ROI须分别标物理边界与实际cell数；固定4096cell接口测量和固定物理ROI不是同一负载，不因分辨率改变而偷换。功能/业务链验证以原物理空间与独立oracle为准；性能的4096cell查询仍记录真实体积，报告其与O5物理ROI要求的覆盖关系。概率、域边界、unknown/free/occupied、快照身份、epoch、lease、TTL和失败终态语义不改。

## Pointers

- `docs/plans/OCC_GRID_ACCEPTANCE_G6_CLAUDE_20261030.md` @ `4a09389888fe6f8ddbab48e8a14219b1ff52239e`
- `docs/plans/OCC_O6_O7_EVALUATION_G1_20261029.md` @ `16a8e6d3882824d457346377545945a941522d91`
- `docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/FINAL_REPORT.md`
