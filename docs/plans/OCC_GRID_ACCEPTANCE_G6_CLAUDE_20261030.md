# 自研 OCC 完整验收与 Claude 交接计划 gen6

- date: 2026-10-30
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- generation: 6
- owner: eng/claude/glm-5.3
- cli: claude-code
- product_revision: 6f75902202081ee59077faac62289400adfb627d
- baseline_revision: b33dda919521ef043fa9e30b07dcc79c87f76fce
- inherited_plan_revision: 6ac2a1ad998a9af4f6ed9d5e72d9946bb3b12daa
- inherited_plan: docs/plans/OCC_GRID_ACCEPTANCE_G5_20261030.md
- supersedes: gen5后续执行及owner；历史Claim、失败、回归证据保留
- scope: 自研OCC事实、增量、读取、真实placement、并发和资源验收；不改产品，不比较OctoMap

## 1. 交接与版本隔离

用户指定转交Claude。Cursor收到停止请求后不再启动新组；已启动组先保存原始记录及失败工件，按原超时有界退出，不删除、不择优补跑。交接必须列工具HEAD/dirty patch、输入/配置/install hash、证据目录、进程PID和命令、perf锁使用者、已消耗测量秒数。Claude可Claim并做轻量准备，但正式计时必须先有Cursor停止确认及进程/锁复核。锁文件存在本身不代表运行，不能只看历史PID或日志推断已停，不杀无关任务。

Claude新建 /tmp/elfin-occ-grid-acceptance-claude-g6，分支 agent/eng/occ-grid-acceptance-g6，基座为产品6f759022。不操作Cursor的工作树或stash；只定向迁移已提交脚本和专用测试（现有645d3d0c及交接后明确SHA），记录每项来源。排除未提交叶子批量化和lexsort。运行协调仍用主工作区docs/agents，不建立第二邮箱。

候选和b33分别建独立build/install/log，使用copy install、--base-paths src和实际运行依赖包闭包；不得污染共享install。启动时记录实际加载Python/共享库/IDL/配置路径与sha256，拒绝源树意外遮蔽或陈旧install。产品src/st09_contracts/配置相对固定SHA零改动，工具提交与产品身份分账。重建install是新工件，重算逐文件清单，不沿用旧哈希。

## 2. 范围与允许修改

允许修改评测脚本、适配器、独立判官和专用行为测试，含旧G2证据根参数化。必须验证新根传到全部worker、旧目录未新增覆盖、路径逃逸被拒、失败记录可重放；不存在的CLI先实现测试再冻结。不得改融合数学、射线采样、地图语义、查询算法、服务IDL、默认配置、lease、缓存/执行器/调度实现来过门。

无Gazebo、运动执行、n=3、OctoMap运行或双后端速度比。旧失败不按新合同改成PASS。性能问题只报告，不自行转为优化任务。已批准的gen5业务/旧压力分判、所有未在本计划明确覆盖的阈值和停止纪律继续有效。

## 3. P0 清单与仪表准备

从以下真实历史来源生成CASE_CATALOG，保留源文件hash、每条case键和输入/期望来源，不以凑数字替代：

| 清单 | 已核实来源（相对docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025/s5） | 重建规则 |
|---|---|---|
| 198取证项 | query_matrix_02/manifest.json | groups[*].instrumentation排除not_applicable；键resolution/boxes/shape/case，18x11、唯一198 |
| 612响应对 | placement_matrix_03/manifest.json | groups[*].response_comparison.rows；键resolution/boxes/path/name，18x34；34=冷1+预热3+评分30 |
| 180000 oracle | oracle_matrix_10/manifest.json及workers/input journal | 固定20seed x500事务 x3分辨率 x3箱数 x2hull；保存逐行键、fixture摘要，不只核公式 |
| 72写/18查询 | 冻结生成器及历史manifest | 逐维度展开并与现有CASE_CATALOG交叉核对，不更换输入或种子 |
| 316回归 | gen5 regression_316/RESULT.json及历史junit node_id | 集合精确一致；新增相关测试另列 |

placement旧18组为res x boxes x hit/fallback，不是res x boxes x hull；与新增真实业务服务18组不得混为同一矩阵。原始取证case含distance_1/distance_256/frontier/frontier_count/instance/instance_empty/patch/roi_dense/roi_empty/summary/surface。发现输入文件缺失时列具名缺口，不伪造历史结果。

316项已经源码级通过；保留该证据。冻结安装后重跑依赖ament/IDL/配置/服务或实际导入不同的集合，逐node说明复用/重跑依据，不机械重跑无影响单元，也不将源码单元通过冒充部署通过。

工具行为tests须包含正负对照：错/空身份、重复/漏case、29条不足与30条P95第29条、全拒绝、提前/迟到CURRENT、重复start重置期限、非法合批覆盖、丢完成、慢/快写、记录器全关/人为慢、共享计费/独立副本/短命buffer/超资源、故障重放。健康对照可通过，故意错误必须被判官拒绝。现有测试可按确切SHA复用，不只检查源代码字符串。

冻结MANIFEST.json、COMMANDS.sh、CLI --help、工具SHA、产品差异和完整case表后才评分。COMMANDS必须是真实展开argv，包含各矩阵、judge、replay、环境和绝对路径；不虚构未实现参数，不仅列测试名称。

## 4. P1 正确性与资源代表组

先运行180000条事实/操作/旧读取oracle及相关回归。逐字段/合同要求的float64位模式零差异，零新增失败；缺行、重复行、未证明的skip均不能PASS。输入journal两侧一致，产品及工具来源可核验。

增量六类每类30事务：重复operation、同观测新stamp、概率饱和、稀疏变化、metadata-only、reset。事实变化与占据分类变化分账；未变块复用，脏集覆盖所有变化，未变对象不重复重建/解码的工作断言按原合同验；同观测不假定no-op。

生命周期清单按既定故障合同复用或补齐：FULL/DELTA、catch-up/resync、owner重启、损坏/缺对象、pointer producer/path/digest变化、重启重验、payload变化、持有旧句柄跨代稳定、close/kill。每个未覆盖边界至少1个确定性正/负复演；旧SHA故障通过不能直接外推。稳定pointer下正文原位损坏的实际检出时机单列，不称立即检测；若违背硬合同则保留FAIL/待裁定。

资源先做fixture，再做1cm倒角0/20箱各3预热+30提交、每组一个受控FULL resync，监测启动至排空。RSS部署合计<=2GiB、派生唯一物理资源<=512MiB、init/reset<=30s。派生涵盖writer缓存、receiver解码索引、队列、staging/resync、共享文件映射和临时缓冲；物理对象去重，独立拷贝分计，不相加磁盘/RSS/PSS。至少1Hz RSS/PSS、VmHWM及短命分配事件；缺字段是unknown/not_evaluated而非0。

## 5. P2 查询、写入与取证全矩阵

| 矩阵 | 每组样本 | P95/max门 |
|---|---|---|
| 18查询组：1/2/5cm x0/5/20箱 x cuboid/chamfer | 3预热+30评分，各接口分别统计 | ROI/frontier/instance 100/250ms；summary50/200；count1/5；distance1点5/20、256点50/200；surface/snapshot100/250 |
| 原72写入组，保留30k/100k/outside/crossing全部维度 | 3预热+30对基准/候选，AB/BA各15 | P95<=b33x1.10；max<=b33max x1.20+20ms；结果一致 |
| 原198取证项，同接口同部署路径开关记录器 | 30对，AB/BA各15；记录器唯一变量 | ON/OFF P95比<=1.10，记录身份join完整，dropped/failed=0 |

全部终态与成功子集同时统计；每组至少30成功才能评时延，缺样not_evaluated；拒绝不伪装成功。单次、首读、变化后、缓存命中分列。旧性能FAIL仍FAIL，不为低时延接口改绝对增量门。b33无V2时只共享语义做对照，候选V2测自身绝对门，禁止facade和typed直接计算加速比。

## 6. P3 实际选位链

先完成旧18组612对完整响应字段比较（忽略已定义为无语义的CDR对齐填充，原始字节仍保留），再做res x boxes x hull的18组真实ComputePlacement，每组3预热+30评分。冻结planner/atlas/Attachment/结构搜索输入，覆盖结构命中、route失败续取、网格回退和预算截断fixtures；不得只用facade代替ROS服务。

客户端明确目标地图身份，证明planner采纳同版本surface/账本且单次求解固定快照。获取<=2s、重试不重置预算；服务RTT P95<=5s/max<=20s，surface/snapshot100/250ms。硬几何门、支撑/倾角/hull/间距、端到端身份错误或假合法容忍0；不完整覆盖不得报箱满。地图等待单列，输入就绪至结果返回直接计量，不把分段中位数相加。本轮无相机Hz或整体观测链硬时限。

## 7. P4 持续业务与独立旧压力

业务：1cm倒角0/5/20箱各120s x3、seed0/1/2；3写+每类3查询预热，初始同步、共同barrier。单写在途完成即续投，不等READY不休眠；查询1Hz acquire->ROI->surface->release，120计划链全记、零miss、每类型>=30成功且原查询门成立。每窗完成写>=30，否则not_evaluated。

独立客户端t=3+6k的20次CURRENT获取及真实placement，获取2s绝对期限、仅合法传播等待50ms重试、20/20成功；句柄持有<=2s，求解用固定输入副本。20样本不足P95则不签该窗placement P95，正式30样本服务门由P3负责。所有计划请求保留；迟发/漏发不能变更计划掩盖。

旧压力C1完成驱动写、C2固定2Hz写，分别0/5/20箱各120s一次，查询summary10Hz/ROI5Hz/distance256 2Hz/surface1Hz；每类型单在途，不突发补偿。C2单发送+单等待、240计划到达，所有not_admitted_capacity保留输入身份。两档各自完整账和原响应/零漏发门单独判，容量不足不单独否决，但静默丢失或错误不可豁免。业务通过不能把旧压力FAIL改绿。

每窗排空30s；双向health间隔<1s、零lease，真实frame-runtime不可用则标not_evaluated，不造假心跳。实测完成率、每10秒吞吐、CPU/帧、队列/导出峰值完整报告，不以1/平均服务时间声称可持续吞吐。三次业务窗逐次AND通过仅代表有限矩阵，不推断长期100%可靠。

## 8. 传播计量与失败处置

记录input_ready/send/receive/write_return、融合/派生/快照、offer/编码/hash/文件写/发布、receive/validate/READY、query scheduled/send/callback/result、planner采纳和求解时刻；完整run/input/request/快照身份、PID/TID和单调时钟。墙钟/线程CPU/进程CPU不混淆，异步段允许重叠。

传播门P95<=1s/max<=2s。精确commit不可观测则保留其区间；READY观察-write_return同时有起终点偏差，不称精确延迟或上界。可构造上下界时，上界全达门才能PASS、下界超门可FAIL、其余not_evaluated。exact/coalesced/timeout/export_failed逐提交join真实覆盖，未送达不出分母。

统计P95=sorted[ceil(0.95*n)-1]，n30第29条，不删首尾、不池化、不择优补跑。正确性错误/崩溃/资源失控停止依赖组；性能失败完成有界组后可继续独立组，不能因distance单项拖住所有业务验收。若仪表不足先补工具，不在评分中修产品。任何后续产品修复另单。

## 9. 额度、环境与交付

沿gen4/gen5共同16小时实际测量剩余额度，不新发16小时。交接复核消耗，锁等待不计；预算到期剩余not_evaluated。持/tmp/elfin-occ-backend-perf.lock且确认旧进程结束；每run记录CPU/governor/affinity/线程/外部负载/软件/文件系统。不外推Orin。操作30s超时、子进程最长30分钟、窗口排空30s；超时保留证据不自动复跑。

证据新根docs/status/evidence/occ_owner_b1/occ_grid_acceptance_g6_20261030/，旧gen5只读引用。每run<=8GiB、总<=32GiB；失败前10秒后5秒边界及重放所需输入前缀/manifest/对象保留，达到配额具名停止，不删除关键证据。输出GATE_INDEX、CASE_CATALOG、MANIFEST、COMMANDS、raw、judge及tests、replay、资源账、FINAL_REPORT。

逐门必须列产品/工具/install/input SHA、来源、样本数、统计和阈值、PASS/FAIL/not_evaluated/unsupported及原因。四项最终结论：功能正确性、实际业务、旧压力、资源恢复；另列完整验收资格。任何必需门未通过不可宣称完整产品PASS。任务Result的pass仅代表交付符合本合同，产品FAIL须显著保留；交接/输入/仪表真正不可满足时具名blocked，不能持续空转。

阶段回帖：接管清单 -> 工具/install冻结 -> 代表组 -> 各独立矩阵 -> 最终收口。无需为已授权脚本/安装准备反复请示；新语义或产品修改须另行授权。公平比较持续暂停，本代收口不自动解锁比较或迁移。
