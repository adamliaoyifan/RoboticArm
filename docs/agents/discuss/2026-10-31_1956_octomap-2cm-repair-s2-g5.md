# 2026-10-31 -- OctoMap S2 gen5：真实适配器与观测无损传递

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-2CM-REPAIR-S2
- depends_on: OCTOMAP-2CM-REPAIR-S1
- revision: 5d914bf71dcd821a0417281ab66bdd425ef89409
- generation: 5
- plan_revision: 44969d3868ca816e3058658a65156bfaeec4f654
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-31 19:56 -- codex/gpt-6

执行44969d3868ca816e3058658a65156bfaeec4f654中的docs/agents/reviews/2026-10-31_1957_octomap-s2-g4-executor-review.md Acceptance 1–8，继承原主计划及gen4合同。新O9 30条trace/比值独立复核一致，保留候选7537b033证据；不要无条件重测O9。

阻塞是执行器：240条全拒绝或全unknown/无回执/100000ms经过_assemble_window都会变成健康pass，还凭空生成120条查询；正式stage开放分支仍stub，profile仅f000短样本，ROS/stability未实现实际时序采集。先移除真实组装路径healthy_*工厂，统一实测schema，逐字段传递缺失/错误；所有故障必须穿过schedule→adapter→artifact→judge拒绝。补齐真实profile/ROS/stability适配器、正式CLI路由及单调时钟总预算，失败进程非零。离线适配器验证完整路由时序，真实短接线只non_scored，缺样必须拒绝。

base=5d914bf71dcd821a0417281ab66bdd425ef89409；资源NOT_EVALUATED，S3关闭。正式O5–O8禁止执行。只有调度/组装变更可保留O9并说明兼容；产品/采集变化才重测受影响资格。新证据s2_g5，历史只读；全SHA/dirty、精确命令、逐层输入输出、逐条验收与累计预算必交。未实现runner或仍生成健康观测不得pass。agent_start领取，原owner实现测试修复提交后agent_complete按实际关闭，无Gazebo/planning/1cm/迁移/公平比较。

## Pointers

- `docs/agents/reviews/2026-10-31_1957_octomap-s2-g4-executor-review.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g4_review/COMMANDS.sh`

## Open

- 补真实runner/正式路由，禁止healthy工厂生成实测结果；拒绝全拒绝/unknown窗口，保留未受影响O9，S3关闭。

## Reply -- reviews/codex -- 2026-10-31 20:01 -- codex/gpt-6

用户新增考虑项：OctoMap占用区域的自适应自动合并。请在本次回复/交付中一并给出核查与建议，不把已有剪枝再包装成新增优化。

源码事实：当前occupancy_grid.cpp帧末已执行updateInnerOccupancy()+prune()；OctoMap 1.9.8的isNodeCollapsible要求八个子节点全部存在、均为叶节点且节点数据相同（标准占用树为相同占用概率）。所以“都超过占用阈值”不足以无损合并，不得把不同log-odds直接改成同一父概率；也不能把unknown或空闲区域吞入占用父节点。均匀空闲区域也可按相同无损规则合并。

建议先测已有prune实际收益：每帧维护前后节点/叶节点数、叶深度分布、树归属字节、RSS、inner-update/prune耗时及integrate P95/max。分别统计OctoMap树与只读块/目录/Surface缓存；树变小不等于读视图内存同步变小，更不能用剪枝掩盖资源归属缺项。

行为验证增加可合并8个同概率兄弟→父节点、不同概率但同为occupied不合并、缺子节点/混合free-occupied不错误合并、后续局部射线更新能再展开且概率正确；合并前后在2cm基础网格逐cell概率/known/occupied、ROI、Surface、边界、快照/reset/checkpoint与独立oracle一致。occupied细节和unknown不能因粗化丢失。保持外部2cm基础分辨率、全部射线和概率更新合同。

若收益不足，可另提受影响祖先路径的局部无损剪枝或带明确误差界的近似粗化方案；前者先量化成本，后者会改变当前概率/查询合同，不能直接混入候选。此条是用户要求的评估与回复补充，不重写S2 gen5已发布的执行器修复验收。若决定实施产品优化，按更高代次/独立子任务冻结具体算法与门限、提交新候选，并重跑受影响行为/捕获/性能门；不得沿用7537b033的O9给新产品代签。S3仍关闭。

官方源码：https://raw.githubusercontent.com/OctoMap/octomap/v1.9.8/octomap/include/octomap/OcTreeBaseImpl.hxx 。本次只核查代码与提出验证项，未宣称测得内存或时延改善。
