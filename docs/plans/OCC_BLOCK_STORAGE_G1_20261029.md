# BlockBatch 优化执行方案：本帧直接定位与连续块存储

- base_revision: d0d3913949ccc966f680e24b58ecfbbcf6599b51
- owner: eng/cursor/grok-4.7
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-BLOCK-STORAGE
- generation: 1
- depends_on: none
- scope: /tmp/elfin-occ-octomap-perf-cursor-g1/src/uld_occ_observer

## 1. 目标与执行约束

用户已授权发布本方案给Cursor Grok4.7，实现、功能验证、分步测量并交付。按两个独立提交推进：A减少重复定位，B改为连续块存储；分别冻结、测量，判断收益来源。

继续现有worktree，不新建worktree。只改observer包；主工作区docs/agents协调、docs/status/evidence取证。认领前确认包干净且HEAD为精确基线；并发修改不得覆盖、reset或静默换基线，须在线程记录版本事件后解决。

本轮只优化块维护及必要查询适配。保持OctoMap更新、完整DDA/端点、概率规则、默认分辨率及discretize=false；不同时实施lazy_eval、排序、线程、GPU或分配器替换。保留外层持久BlockRegistry，不引入Open3D依赖。不新增产品安全门，不自动迁移生产，不将本任务完成等同O5/O6/O7通过。

| 工作单元 | owner | 基线/依赖 | 交付 |
|---|---|---|---|
| A本帧直接定位 | eng/cursor/grok-4.7 | d0d3913949ccc966f680e24b58ecfbbcf6599b51 | 实现提交A、功能验证、基线到A对照 |
| B连续块存储 | 同上 | 本任务A冻结提交 | 实现提交B、查询/恢复/快照回归、A到B对照及最终归因 |

同一owner顺序完成，无需另派routine测试角色；精确plan_revision见派发线程。

## 2. 实现A：本帧直接定位

新增聚焦的块寻址与批写模块，替换ObservationStore::integrate中的临时modified红黑树。A保留现有CellBlock=std::map<CellIndex,CellValue>，不混入块内表示变化。

```cpp
struct BlockAddress {
  CellIndex block_index;
  std::size_t directory_index;
  std::uint16_t local_offset;
};
class BlockLayout {
 public:
  explicit BlockLayout(CellIndex grid_shape);
  BlockAddress locate(CellIndex index) const;
  CellIndex cell_index(CellIndex block, std::uint16_t offset) const;
  std::size_t block_count() const;
};
class BlockUpdateBatch {
 public:
  BlockUpdateBatch(const BlockLayout&, const BlockRegistry&, OccupancyDiagnostic&);
  void write(CellIndex index, CellValue value);
  void commit(BlockRegistry&);
  std::size_t touched_blocks() const;
};
```

固定块边长8，公式集中维护：block形状各轴ceil(grid_shape/8)，目录下标(bz*blocks_y+by)*blocks_x+bx，局部下标lx+8*ly+64*lz。负数与网格边界在无符号转换前检查；乘法检查溢出；逆变换拒绝无效block/offset和边缘填充槽。Snapshot::block_index保留兼容入口，委托共享的块坐标公式，不留第二套实现。不增加任意业务容量阈值，遵守既有几何限制、checked算术和容器分配能力。

batch构造长度block_count的空指针目录。首次触及某块时查持久registry，新建或复制旧块；本帧touched列表持有shared_ptr，把其稳定对象地址存入目录，后续直接数组定位。commit只遍历touched列表，以shared_ptr<const CellBlock>发布。提交后继续写或重复提交明确拒绝。不能由裸指针负责释放，也不能根据use_count原地修改旧块。

当前1cm形状的块槽为19*25*19=9025；64位裸指针目录72200字节。每帧清空目录和临时引用，不跨帧复用分类或可写块；旧快照始终不可变。目录初始化、首次复制、写入和提交都计入BlockBatch；临时对象销毁留在词法Fusion内，不转移到计时外。该改动不新增整个integrate对任意分配异常的回滚承诺。

## 3. 实现B：封装连续CellBlock

在A上将CellBlock别名改为封装类：

```cpp
class CellBlock {
 public:
  bool read(std::uint16_t offset, CellValue& value) const;
  bool write(std::uint16_t offset, CellValue value); // 返回是否首次known
  std::size_t known_count() const;
  template<class Visitor> void for_each_known(Visitor&& visitor) const;
};
```

内部固定std::array<CellValue,512>、8个uint64_t的known位图、known数量。unknown只由known位图表达，零log-odds不能代替unknown。write只在首次known时增加数量，概率或occupied改变不重复增加。未知read返回false并清空输出值，避免旧调用结果残留。

for_each_known按位图置位槽遍历，使用编译器支持的位扫描且先排除零word；不每次扫描512个值。复制块为数组、位图及计数的值复制，记录实际sizeof，不假设结构无padding。存储字段私有，写入只通过write，业务模块不得直接改位图/数组/计数。

统一适配三条路径：

- 在线write：BlockUpdateBatch用共享寻址产生offset并调用CellBlock接口，保持每触及块每帧只复制一次。
- Snapshot::cell/read_cells/surface：复用寻址；ROI顺序/分页不变；Surface只遍历known槽，恢复全局坐标后保留domain.active判断，边缘填充/倒角外保持unknown，输出列顺序不变。
- restore：临时OctoMap展开校验后，经相同batch写入接口重建；完整完成再替换现有状态。重建使用独立空registry及本地diagnostic，不伪造在线融合计数。checkpoint文件格式不变。

旧快照、旧Surface缓存、版本推进与失效规则不变。Surface scanned_cells继续表示访问的known格；copied_cells表示复制块中的known格数，不能改成固定512。实际复制字节如需展示另列字段，不改旧字段含义。

## 4. 功能验证

扩展已有FusionProfileChecks和FusionBenchmarkSuite；必要的小型CellBlock测试class放包内测试层并接入CTest。不得新造一次性worker或从evidence导入测试实现。不以源代码包含字符串替代行为测试。

| 场景 | 预期观察 |
|---|---|
| 寻址 | 覆盖0/7/8/末端、负数、越界、非8整除形状；所有合法地址往返一致；越界明确拒绝 |
| 重复定位 | 同块连续/交错写只新建或复制一次；不同槽互不覆盖；未提交registry不变 |
| known | unknown/known free/occupied区分；零log-odds正确；重复写不重复计数；读取未知不残留旧值 |
| 部分块/倒角 | 填充槽与域外槽不写入、不进入查询及Surface统计 |
| 旧快照 | 持有旧快照并更新当前图，旧ROI/Surface/概率均不变 |
| Surface | 最高occupied变free后正确回落，无占据列、cache hit与known计数正确 |
| 恢复 | 旧格式checkpoint可恢复，继续更新与不中断序列一致；坏恢复不替换当前状态 |
| 模式 | Off/Stages/Detailed树、语义计数、identity、ROI、Surface一致 |
| batch生命周期 | commit后write或再次commit拒绝；未commit退出只释放临时块 |
| 表示对照 | 小地图逐known key对测试参考map完全一致，1/2/5cm全覆盖 |

两步均运行全部CTest、相关Python CLI/report/benchmark测试；最终跑ASan/UBSan聚焦测试与ROS节点Release链接，不启动ROS/Gazebo。确定性测试要求零失败，只代表已列行为，不声称长期成功率。

## 5. 性能矩阵与判定

基于现有FusionBenchmarkSuite扩展--repetitions（baseline原缺省1，本任务显式3）与--require-content（本任务开启）。保持现有CLI兼容。缺任一侧查询摘要=>不可比较，不能以树摘要代替；增加对应runner行为测试。

Release -O3 -DNDEBUG、OctoMap1.9.8 exact、单进程串行、1cm chamfer、discretize=false、surface=true、state_digest=true且摘要在integrate/query时钟外。固定已有四组lossless输入：
`docs/status/evidence/occ_owner_b1/occ_octomap_perf_diag_g1_20261026/run01/fixtures/`，geometry_chamfer.yaml，o5_b{0,20}_c{30000,100000}/manifest.yaml；config使用原depth .52–6、clock_domain ros和resolution_m .01，保存字节/hash。禁止重新随机生成替代输入。

| 对照 | 窗口 | 目的 |
|---|---|---|
| 基线→A，Off/Off | 4组*3对 | 重复定位收益 |
| A→B，Off/Off | 4组*3对 | 连续块存储增量收益 |
| B Off→B Stages | 4组*1对 | 最终阶段归因 |

共28对，56个完整进程窗，1848次帧执行。每侧3预热+30计分，保留原始33帧历史；窗口级AB/BA交替，非逐帧交错。每对输入、树SHA、ROI/Surface摘要、语义计数和identity一致；A/B不改树写算法，树结构也应相同。prepared/copied等诊断字段不当作Off实测值。

每窗P50/P95/max，P95第29/30项、中位第15/16均值。报告integrate、ApplyBatch、BlockBatch、Fusion remainder，index3/4及31/32，acquire/ROI/首次Surface/缓存Surface，changed/copied/known格、触及块、mutation容量、sizeof(CellBlock)和进程峰值RSS。阶段份额按同帧计算，独立阶段P95不能相加。

Stages/Off沿用既有低扰动约定：integrate P95比<=1.10且max比<=1.20时用于归因；超出标perturbed、只作机制观察，不增加产品门。无新增固定加速百分比门；收益和回退逐组报告，产品旧阈值只作参考。零mismatch是有效对照的前提，样本不足30=>not_evaluated。

全部AND只代表此有限矩阵，不外推实机成功率。没有稳定收益也必须归档，不无限重跑择优，不自动选择上线版本。

## 6. 执行程序

1. scripts/agent_start.sh认领线程；读架构索引的相关合同与.cursor/rules/debug-evidence.mdc；记录包HEAD/status，冻结基线CLI与ldd/hash。后续需要基线源码可git archive提取到构建输入目录，不checkout/reset现有树、不建新worktree。
2. 实现A、接口文档和测试，先非计分小夹具演练；功能通过后提交，从提交重新构建，再执行基线→A，保存A二进制。
3. 实现B与全部适配；功能通过提交，重建后执行A→B和最终B Off/Stages。每冻结版本测量中不改产品或runner；如改动，用新版本新目录重测受影响对照。

示例构建命令在产品worktree执行，A/B使用不同build目录：

```bash
cmake -S src/uld_occ_observer -B /tmp/occ-block-a-build -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/occ-block-a-build -j2
ctest --test-dir /tmp/occ-block-a-build --output-on-failure
OBSERVER_PROFILE_CLI=/tmp/occ-block-a-build/observer_profile_cli python3 -m pytest -q src/uld_occ_observer/test/test_profile_cli.py src/uld_occ_observer/test/test_profile_report.py src/uld_occ_observer/test/test_fusion_benchmark.py
```

性能调用包内test/benchmark/fusion_benchmark.py：baseline campaign传冻结前后CLI、--repetitions 3 --require-content；最终stages传B CLI到两侧、--repetitions 1 --require-content。每次显式传fixture-root/geometry/config/out/source-root及timeout/budget。所有实际带完整路径命令、各CLI版本对应关系写REPLAY.md，不能依赖本机隐式PATH或其他版本binary。ROS节点用独立build目录OBSERVER_WITH_ROS=ON，ASan/UBSan也用独立目录，避免污染计分Release。

4. runner统一持有/tmp/elfin-occ-backend-perf.lock；占用则排队，等锁不消费样本。单窗超时15分钟，测量总预算3小时（不含等锁），在下一窗前检查预算，缺组标未完成。不得清理其他agent进程。
5. 提交最终报告、role note和线程结果，用agent_complete.sh关闭；报告实现/正确性、性能收益或回退、未解释项、候选版本建议，正式产品验收仍未评估。

## 7. 取证与交付

证据根：`docs/status/evidence/occ_owner_b1/occ_block_storage_20261029/run01/`，已存在则递增run，不覆盖。上限8GiB/run，lossless输入引用既有不可变路径+SHA；预算不足先记录缺项，不删失败证据。

| 边界 | 常规记录 | 触发与失败保存 |
|---|---|---|
| 输入→integrate | 版本、config/input hash、index、模式、receipt | 状态错/hash错/超时：保存完整历史输入引用、最近3帧及故障帧输出、stdout/stderr/退出状态 |
| apply→block写回 | 原有粗阶段计时、changed/copied/touched/容量 | mismatch：两侧同帧输出、版本及完整前缀引用，非计时重放保存首个不同key/column |
| snapshot/query | identity、树和查询摘要、查询耗时 | 缺摘要或内容分叉：标不可比较或mismatch，保存实际ROI/列差异；不能仅保存pass/fail字符串 |
| 进程退出 | pid、exitcode、超时及清理结果 | 只终止本次进程组；保留最后完整帧，不从半窗续跑；修复后新attempt完整回放 |

交付A/B提交、接口README、CTest/Python/ASan/UBSan/ROS链接日志、IDENTITY.json、REPLAY.md、逐帧数据、逐窗summary和FINAL_REPORT.md。每个结论链接具体窗口。若计时受扰或重要对照未完成，明确限制，不升级产品验收状态。
