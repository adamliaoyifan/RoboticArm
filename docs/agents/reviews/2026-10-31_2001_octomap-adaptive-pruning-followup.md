# 2026-10-31 -- 占用区域自适应合并补充

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

用户新增考虑项：OctoMap占用区域的自适应自动合并。请在本次回复/交付中一并给出核查与建议，不把已有剪枝再包装成新增优化。

源码事实：当前occupancy_grid.cpp帧末已执行updateInnerOccupancy()+prune()；OctoMap 1.9.8的isNodeCollapsible要求八个子节点全部存在、均为叶节点且节点数据相同（标准占用树为相同占用概率）。所以“都超过占用阈值”不足以无损合并，不得把不同log-odds直接改成同一父概率；也不能把unknown或空闲区域吞入占用父节点。均匀空闲区域也可按相同无损规则合并。

建议先测已有prune实际收益：每帧维护前后节点/叶节点数、叶深度分布、树归属字节、RSS、inner-update/prune耗时及integrate P95/max。分别统计OctoMap树与只读块/目录/Surface缓存；树变小不等于读视图内存同步变小，更不能用剪枝掩盖资源归属缺项。

行为验证增加可合并8个同概率兄弟→父节点、不同概率但同为occupied不合并、缺子节点/混合free-occupied不错误合并、后续局部射线更新能再展开且概率正确；合并前后在2cm基础网格逐cell概率/known/occupied、ROI、Surface、边界、快照/reset/checkpoint与独立oracle一致。occupied细节和unknown不能因粗化丢失。保持外部2cm基础分辨率、全部射线和概率更新合同。

若收益不足，可另提受影响祖先路径的局部无损剪枝或带明确误差界的近似粗化方案；前者先量化成本，后者会改变当前概率/查询合同，不能直接混入候选。此条是用户要求的评估与回复补充，不重写S2 gen5已发布的执行器修复验收。若决定实施产品优化，按更高代次/独立子任务冻结具体算法与门限、提交新候选，并重跑受影响行为/捕获/性能门；不得沿用7537b033的O9给新产品代签。S3仍关闭。

官方源码：https://raw.githubusercontent.com/OctoMap/octomap/v1.9.8/octomap/include/octomap/OcTreeBaseImpl.hxx 。本次只核查代码与提出验证项，未宣称测得内存或时延改善。

## Pointers

- `docs/agents/discuss/2026-10-31_1956_octomap-2cm-repair-s2-g5.md`
- `src/uld_occ_observer/src/occupancy_grid.cpp`
- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
