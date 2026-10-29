# OctoMap 树更新与块维护：可复用测试类开发与归因执行计划

- base_revision: d49b49bfbae247d6e69246c2b7d6e79eb39be0a0
- owner: eng/cursor/grok-4.7
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-FUSION-BREAKDOWN
- generation: 1
- scope: /tmp/elfin-occ-octomap-perf-cursor-g1/src/uld_occ_observer

## 1. 目标与交付边界

用户要求把树更新和块维护测试做成可维护的测试 class，明确接口并交给 Grok 实现、测试、测量和归档。本任务回答：慢帧主要耗在树更新批次还是块维护？早晚帧差异与 changed_keys、树节点、复制格和 mutation 容量有何关系？

只开发诊断与测试基础设施，不实施 lazy_eval、改块容器、排序、饱和跳过、线程、GPU、减少射线或换后端。沿用1cm、完整DDA/端点、discretize=false、原概率参数。不得以完成本任务宣称 O5/O6/O7 或产品验收通过。不增加产品安全门；本任务的完整性与一致性检查仅用于保证测量可解释。

继续现有 worktree，不新建 worktree。主工作区 docs/agents 协调与 docs/status/evidence 取证；不编辑其他脏文件。执行者先确认产品包干净且HEAD等于基线；若有并发修改，记录版本事件并停止依赖该基线的修改/测量，不静默换基线或覆盖。

## 2. 已知结构与计时命名

当前 OccupancyGrid::apply 包含旧值search、updateNode、变化判断和mutation vector生成；ObservationStore::integrate随后复制/维护块并写回registry。不能将整个apply命名为纯OctoMap更新耗时。

receipt.fusion_ms 当前包含 finalization，ProfileStage::Fusion 不包含；保留两个旧字段的含义，不悄悄重定义。新报告以词法 ProfileStage::Fusion 为分母，并单独展示 legacy_receipt_fusion_ms 与 Finalization。不要把相减后的残差直接命名为分配、析构或树更新。

## 3. 生产诊断接口：仅扩展现有统一写者

修改 include/uld_occ_observer/profiling.hpp，保留现有九个顶层阶段及Detailed字段，不另建诊断manager、不新增运行模式。新增只读报告成员：

```cpp
enum class FusionPhase { ApplyBatch, BlockBatch, Count };
struct FusionBreakdown {
  bool collected = false;
  std::array<bool, 2> visited{};
  std::array<double, 2> milliseconds{};
  bool mutation_storage_collected = false;
  std::size_t mutation_size{}, mutation_capacity{}, mutation_element_bytes{};
};
// UpdateProfile新增 FusionBreakdown fusion_breakdown;
// OccupancyDiagnostic新增：
FusionSpan fusion_span(FusionPhase phase);
void record_mutation_storage(std::size_t size, std::size_t capacity,
                             std::size_t element_bytes);
```

FusionSpan 为现有 OccupancyDiagnostic 内的局部计时类型，finish() 幂等，析构兜底；不可复制，移动后原对象失效。Stages/Detailed 启用，Off不读新增时钟、不写新增计数，报告collected=false。该类是唯一字段写入者；调用方不得直接修改 UpdateProfile。测试可以通过真实执行验证，不要求修改全局时钟或添加复杂时钟注入框架。

每帧正常路径只有两个粗粒度区间，不在每格循环创建新对象：

- ApplyBatch：调用 grid_.apply 前开始，返回mutation vector后立即结束。包括旧值查询、树更新、mutation增长与返回；不含随后block循环，不含最终mutation销毁。
- BlockBatch：现有modified块表构造前开始，到全部blocks_写回后结束。包括块表查找、COW复制、逐格更新、registry写回；不含scope退出时局部modified/mutations销毁。
- 保持原对象寿命与原Fusion外层scope，不移动清理到总计时以外。fusion_remainder_ms由测试类逐帧计算 Fusion-ApplyBatch-BlockBatch，明确包含未划入上述两段的工作及测量边界开销，不声称已细分。
- mutation元数据在apply返回后记录一次：size/capacity/sizeof；仅诊断启用时调用。effective_bytes=size*element_bytes，reserved_bytes=capacity*element_bytes由报告计算，不能称为本帧累计分配或RSS。
- 继续复用 copied_blocks/copied_cells、receipt changed_keys/rebuilt_blocks，以及store diagnostics tree_nodes/known_cells/block_count，不增加逐key诊断计数。
- 既有Detailed逐格 TreeUpdate/BlockMaintenance 不用于本次计分/归因；不通过相减Detailed读数估计Off成本。

src/serialization.cpp 加 additive `profile.fusion_breakdown` 字段。Off输出collected=false，时长/容量使用null或省略，不让未采集的0伪装成测量值。Stages/Detailed输出采集标志、visited、两段时长和storage字段。文档写明schema兼容与计时边界。不改变ROS消息、配置digest、默认YAML或业务DTO语义。

## 4. 测试class与新增接口

### C++：FusionProfileChecks

在 test/fusion_profile_checks.hpp/.cpp 定义可复用测试类，加入现有 observer_profile_checks 目标，由 test_profiling.cpp 调用，避免扩大main或另造临时检查二进制。建议公开：

```cpp
class FusionProfileChecks {
 public:
  void check_modes_and_equivalence();
  void check_phase_lifecycle();
  void check_sequence_and_failure();
};
```

class持有/构造独立测试store与确定性夹具，调用真实integrate。旧测试helper可少量提取复用，不复制生产apply/block逻辑。小夹具覆盖1cm与5cm，默认discretize关闭。不得把assert源文件字符串作为功能验证。

### Python：FusionBenchmarkSuite

新增包内 test/benchmark/fusion_benchmark.py（必要时拆一个report模块，避免变成通用巨型runner），定义：

```python
class FusionBenchmarkSuite:
    def validate_inputs(self) -> InputManifest: ...
    def run_window(self, binary, mode, case, repetition) -> WindowResult: ...
    def compare_frames(self, left, right) -> ComparisonResult: ...
    def summarize_window(self, window) -> WindowSummary: ...
    def run(self) -> CampaignReport: ...
```

构造时显式接收 baseline/candidate profile CLI、fixture_root、geometry、config、out、campaign与超时；路径用CLI传入或相对manifest解析，禁止固定个人home和/tmp输入。类调用包内 observer_profile_cli，不复制第三份C++ worker、不从旧evidence目录import driver/gate、不把产品测试代码藏在runXX/harness。evidence只保存配置、manifest、运行数据和脚本SHA。

CLI固定入口：

```text
python3 test/benchmark/fusion_benchmark.py
  --baseline-cli PATH --candidate-cli PATH
  --fixture-root PATH --geometry PATH --config PATH --out PATH
  --campaign rehearsal|baseline|stages
```

- baseline：4组各1对完整窗，baseline Off vs candidate Off。
- stages：4组各3对完整窗，candidate Off vs candidate Stages。
- rehearsal：小夹具演练，不能被当成正式性能样本。
- 同一pair两侧使用完全相同输入顺序，各自空store，依次运行完整33帧；按case序号+rep奇偶决定AB/BA先后并记录。这里是窗口级AB/BA，不冒称逐帧交错。
- profile CLI沿用现有manifest协议、查询顺序、两次Surface和checkpoint摘要；不为本次更改查询范围。基线与候选均相同调用acquire/get/4096cell ROI/Surface/release，surface=true、state_digest=true。
- 若需要逐帧Surface/ROI内容对照，新增受state_digest控制的确定性content摘要接口/工具函数，放包内测试支持层；遍历已有查询结果，不再次查询地图，摘要计时在integrate和query时长外。候选Off与Stages必须都有；旧基线缺此字段不能冒充比较过，基线比较树摘要/receipt/identity，另由功能测试覆盖查询。

## 5. 功能测试目标与预期观察

| 测试 | 明确预期 |
|---|---|
| Off/Stages/Detailed相同序列 | 每帧概率树字节、free/hit/changed/known、身份、ROI与Surface内容完全相同；无模式引起地图分叉 |
| Off行为 | collected=false、visited全false、内部新增计数0；序列化不展示未采集时长为观测值 |
| 正常Stages | 两段visited=true、时长有限且>=0；两段和<=Fusion+0.1ms；原九顶层总计规则继续成立 |
| scope生命周期 | finish两次只记录一次、移动后不重复记录；异常离开scope正常关闭；不借此声称apply异常回滚 |
| storage | size等于当帧changed_keys，capacity>=size；bytes计算正确且不是RSS；空变化帧size=0仍正确标识已采集 |
| 状态演进 | 至少12帧同射线不同stamp达到hit/miss夹紧，然后反向观测；包含新格、旧格、饱和无变化与再次变化；实际记录触发的帧 |
| 失败/下一帧 | 有效前缀+越key范围端点触发collect失败：两段不执行、树/known/identity不变；随后有效帧与独立store相同 |
| runner和报告 | 29个计分样本=>incomplete；缺摘要/少帧/模式错误/输入hash不同=>不可比较；相同计数不同摘要=>mismatch；不要按summary的semantic字符串自证 |
| 统计与故障 | P95第29/30、中位第15/16均值；单窗失败不能被合并均值盖住；进程超时终止该进程组，保存stdout/stderr/最后完整帧，不遗留子进程 |

所有确定性行为用例要求零失败；这是本次测试代码正确性标准，不是对长期可靠性的统计证明。运行包内全部CTest与相关Python CLI/report/benchmark测试，禁止因找不到binary而skip。修改公用头后额外完成一次OBSERVER_WITH_ROS=ON构建，确认observer_node链接，无需启动ROS/Gazebo。

## 6. 测量矩阵、指标与判断

Release -O3 -DNDEBUG、OctoMap 1.9.8 exact、单进程串行、1cm chamfer、0/20箱×30k/100k、discretize=false。fixture沿用：
`docs/status/evidence/occ_owner_b1/occ_octomap_perf_diag_g1_20261026/run01/fixtures/`，geometry_chamfer.yaml，四个o5_b{0,20}_c{30000,100000} manifest。保持原始文件与hash，复制/引用须可重放，不重新随机生成替代数据。

每窗3预热+30计分，原33帧全保存；早期指原index3/4，晚期index31/32，需完整回放历史，不能单独从空图执行这些帧。baseline 4对、stages 12对，共32次完整窗=1056帧执行，逐对比较528对帧。只做此有限矩阵，不扩展2cm/5cm性能战役、不重跑冷Surface战役、不跑O5/O6/O7全矩阵。

报告逐窗P50/P95/max：external integrate、receipt总计、preparation、词法Fusion、ApplyBatch、BlockBatch、fusion remainder、Finalization、legacy receipt fusion、top-level residual、acquire/ROI/两次Surface；Off没有的阶段标not_collected。

逐帧同时记录free/hit/changed/occupancy_changed、known、tree_nodes、block_count、rebuilt/copied blocks、copied_cells、mutation size/capacity/bytes、process_peak_rss_kib。只在Stages有的字段不得补成Off实测值。份额先逐帧除以同帧Fusion再统计，不把各阶段独立P95相加。记录CPU/机器、binary/config/input/库SHA、版本和dirty状态。

诊断有效性沿用已有本地约定：每对Stages/Off integrate P95比<=1.10、max比<=1.20时，阶段数可用于低扰动归因；超出标perturbed，仅机制观察。该约定不新增产品门；超出时保留数据，允许非计分修整一次诊断实现，版本变更后重测受影响矩阵，不无限重跑择优。

baseline Off vs candidate Off不新增硬百分比验收线，报告每窗比值和逐帧差异；若明显受扰，不把候选与历史耗时差解释为算法变化。本任务未改算法，本就不应宣称优化收益。绝对旧产品门仅作参考列（30k500/1000、100k2000/5000、查询100/250ms），达不到不会阻断这次归因任务完成。

最终结论分开：实现/功能测试；测量完整性及归因可信度；哪段成本更大及未解释项；产品验收仍未完成。若阶段差异不稳定，明确未能确定主因，不强行推荐优化。所有AND只代表指定有限矩阵，不推导实机成功率。

## 7. 执行步骤与命令

1. 用scripts/agent_start.sh认领主工作区派发线程。读架构索引相关合同和debug-evidence规则，记录产品HEAD/status。先冻结基线profile CLI：从d49b49bf当前源码Release构建并复制二进制/动态依赖信息到独立build目录，验证SHA。若修改后需要重建基线，可git archive提取该提交包到临时构建输入目录，不创建新git worktree，不checkout/reset当前树。
2. 一次实现诊断字段、两个测试class、序列化/文档和测试接线。先跑功能与runner演练，保留失败输入输出，再修复。
3. 提交包内改动，记录实现SHA；从提交重新构建并执行功能测试。测量期间不改产品或harness，变更则新版本新证据。

```bash
cmake -S src/uld_occ_observer -B /tmp/occ-fusion-build -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/occ-fusion-build -j2
ctest --test-dir /tmp/occ-fusion-build --output-on-failure
OBSERVER_PROFILE_CLI=/tmp/occ-fusion-build/observer_profile_cli python3 -m pytest -q src/uld_occ_observer/test/test_profile_cli.py src/uld_occ_observer/test/test_profile_report.py src/uld_occ_observer/test/test_fusion_benchmark.py
```

其它CLI测试按现有包的binary发现规则传入本次构建，不使用其他版本。ROS构建使用另一个build目录、现有Humble环境；只链接不launch。

4. 在主工作区新证据根写配置resolution_m:0.01、原depth范围和clock_domain；manifest冻结四组输入与两个二进制。通过上述FusionBenchmarkSuite CLI顺序运行baseline和stages。out各为baseline/与stages/，目录不得已有测量文件。实现者将全部带绝对参数的实际命令写REPLAY.md。
5. runner持有 `/tmp/elfin-occ-backend-perf.lock` 的排他flock贯穿测量；占用则LOCK_BUSY，未开测不消耗样本，排队后再启动。单窗进程超时15分钟，测量墙钟2小时（不含等锁），下一窗口启动前检查预算；未完成组标not_evaluated。不要启动Gazebo或清理其他agent进程。
6. 输出报告、更新role note和线程，用agent_complete.sh关闭。功能不通过不能宣布任务完成；完整测量结果即使性能门fail也可完成诊断任务，若关键组没有有效低扰动证据则明确归因未完成及具体阻碍。

## 8. 取证与交付

证据：`docs/status/evidence/occ_owner_b1/occ_fusion_breakdown_20261029/run01/`，已存在则run02，不覆盖旧目录。最多8GiB/run，输入可引用不可变既有lossless文件+SHA，检查预算不删除失败证据。

| 边界 | 常规记录 | 失败触发与保存 |
|---|---|---|
| 输入→integrate | manifest/输入SHA/版本/模式/原index | hash错/状态错/超时：保留完整有效历史输入引用、最近3帧至故障帧输出、错误日志 |
| apply→块维护 | 两段timing、mutation元数据、receipt计数 | 同输入树摘要不同：两侧对应帧及全部前缀引用，离线重放可选保存该帧checkpoint和确定性差异；不在Off时钟内dump |
| snapshot/query | identity、树摘要、候选查询摘要、查询耗时 | mismatch：两侧输出与具体首个不同key/column，保留seed/config/TF原始证据 |
| runner退出 | exitcode、超时原因、pid清理结果 | 不覆盖、不自动从失败半窗续接；先定位修复，新attempt目录重跑完整窗 |

交付实现提交、API/README、CTest/Python/ROS构建日志、IDENTITY.json、REPLAY.md、逐帧YAML/JSONL、逐窗summary、FINAL_REPORT.md。报告链接每个结论到具体窗口，不只写pass。无需新引入审批流程、独立复核角色或额外产品门。
