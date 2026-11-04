# 双后端比较 G3：版本、使用方兼容与同输入对照执行方案

- date: 2026-11-04
- author: reviews/codex/gpt-6
- status: approved
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCC-BACKEND-COMPARISON
- generation: 3
- owner: eng/claude/glm-5.3
- dispatch_ready: yes
- supersedes_comparison: docs/plans/OCC_END_TO_END_BACKEND_EVALUATION_20261030.md comparison_generation=2

本方案落实用户已确定的路线：冻结双方 → 使用方兼容 → 同输入比较 → 独立分支共同集成。旧计划的比较部分由本代替代；历史评分及其他任务不改写。用户现已要求完善后派发Claude执行：owner固定eng/claude/glm-5.3，入口与脚本由该owner交付，不声称它们已存在。精确plan_revision取派发线程绑定的提交，其包含本文件、能力矩阵及强制附件 `OCC_DUAL_BACKEND_DATA_ACCEPTANCE_G3_20261104.md`。附件是数据/统计/验收的权威定义；若正文概述与附件冲突，以附件为准。准备后自动按固定合同评分，不再等待泛化review或另一次用户确认。

| 唯一派发任务 | Owner | 基座 | 依赖 | 范围/完成标志 |
|---|---|---|---|---|
| OCC-BACKEND-COMPARISON gen3 | eng/claude/glm-5.3 | a4ac889e | none（两线历史收口作为输入证据复用） | 独立树准备、消费者兼容/适配、判官行为测试、预演、冻结、B1–B3评分、独立复算、报告与提交；按附件C0–C8判定，未评/失败真实报告也可工程收口 |

共同集成候选是比较后的独立任务，本轮不派发、不合master、不授权硬件运动。没有live CLI RPC保证，文件派发不等同于owner已Claim。

## 1. 固定版本与复用范围

| 对象 | 固定版本 | 使用规则 |
|---|---|---|
| 自研产品G | a4ac889e | 已通过结构验收，不重跑其M1–M8/S1–S9或局部性能campaign |
| 自研现有工具 | 18fa18ce | 只作工具来源，不把结构surface_columns成绩当双后端结果 |
| OctoMap产品O | cbdd5364 | reset run03已评分产品；读取实际binary/install哈希，不以分支HEAD替代 |
| OctoMap driver修复 | 9cc59da1 | 只含run_formal.sh及测试的后继修复；不能说该driver已被run03完整运行验证 |
| 共同使用方源码C | cbdd5364中的ROS2 packing/planning及依赖 | 两侧都用同一份C及同配置；不得分别运行两分支各自的solver再宣称后端差异 |
| 协调master快照 | c9506893 | 仅用于本次计划的主线差异定位；未来集成时另固定当时master |

新比较工具T、配置、输入和两个独立install均在准备阶段提交/哈希冻结。C若需要适配修改，必须记录独立adapter提交C'，跑本方案兼容门后才能评分；G/O产品内部保持不变。若确需改产品才能提供必要能力，结束受影响项为BLOCKED，给出最小修复任务和受影响门，不能在计分中边改边测。其他独立公共操作可继续比较。

reset长稳、S3历史矩阵不重跑。run03人工续跑R、driver修复未在原campaign验证及历史FAIL继续明确披露。driver只需行为测试/短预演，不为验证编排重跑120分钟。

## 2. 隔离和责任

先检查目标路径不存在；存在则核对其用途，不reset或覆盖。推荐由单一owner顺序执行：

```bash
git worktree add -b agent/eng/occ-compare-g3 /tmp/elfin-occ-compare-g3 a4ac889e
git worktree add --detach /tmp/elfin-occ-compare-g3-octomap cbdd5364
git worktree add --detach /tmp/elfin-occ-compare-g3-consumers cbdd5364
```

第一棵树承载比较工具T及G产品；后两棵为O和C。各自独立build/install，不借用主workspace overlay；ROS_DOMAIN_ID和命名空间隔离。前缀冲突用独立进程+显式环境解决，不能顺手修改产品接口。两后端轮流运行，测量不同时启动双方抢CPU。

主workspace只写docs/agents协调及证据索引；完整证据归档至主workspace `docs/status/evidence/occ_dual_backend_compare_20261104/<run_id>/`。不改主workspace在途源码，不合master。任务claim/complete沿主workspace脚本；实施者负责实现、测试、修复和归档，不另扔给test角色。

## 3. 使用方能力矩阵与兼容门

矩阵文件见 `OCC_DUAL_BACKEND_CONSUMERS_G3_20261104.md`。准备者按G/O/C的安装清单、launch入口、调用图和接口引用补全所有**活跃**使用方；源码存在但未安装/未启动的ROS1参考不算生产能力。每行记录实际文件/函数、入口、输入字段、后端映射、运行测试、状态和差异。禁止只对方法名做grep就宣称兼容。

每个活跃读取/使用方运行下列10种固定状态各1次/后端：已观测free且存在可行候选；occupied冲突；unknown；无数据；陈旧数据；错误geometry、错误frame、错误单位三种分别测试（按其真实合同）；请求期间reset导致旧epoch返回；reset后新图+新机器人样本恢复。共20次/使用方，方法级单元可共享fixture，但实际ROS2订阅/服务边界至少覆盖placement和planning admission。reset用有界注入，不是新长稳。拒绝原因按既有接口冻结，不能把所有非OK当通过。

通过标准：

- 正常合法输入被真实消费者接受；不合法/过期/旧epoch输入不得授权新规划或提交；零假合法、零身份串用；reset后满足既有恢复条件能重新接受。
- 两侧映射轴序、米单位、height datum、unknown/free/occupied、geometry/config/clock、观测时刻及epoch/revision语义一致。各后端内部revision/digest值可不同，但请求、快照、输出、使用方实际读入必须各自闭合。
- Surface只提供支持面/摘要；KNOWN_FREE必须有同快照完整候选体积的ROI依据，不能把顶部以上默认当free。共享下游验证算法必须两侧相同，其耗时单列。
- 固定同一solver、atlas、参数、Attachment及机器人状态输入。测试实际ComputePlacement/ComputePlacementV2回调、地图订阅接纳和planning入门/提交前复核；可用固定关节/EEF输入，绝不调用执行器发送运动。
- committed/add/remove/实例/距离等能力若被活跃消费者使用，不能以空列表或unsupported冒充兼容；提供语义等价适配及正常/拒绝测试，或将该使用方BLOCKED并停止依赖它的业务计分。仅未使用的可选接口可标N/A并给调用证据。
- 输出地图不同允许导致不同位置/候选数量；独立几何判官验证各自结果安全合法，不要求两侧逐格相同或选位坐标相同。无已知可用空间可合法返回NO_KNOWN_FREE，但不能伪报容器已满。

矩阵全部必需行PASS才可称“使用方兼容”；有缺口时只发布公共操作性能，不签真实业务链兼容。新增活跃调用者须在首次冻结前纳入，冻结后不能凭改进建议加新场景。

## 4. 输入、两种配置与规模

分辨率固定2cm。继承现有fixture来源：OctoMap run03 FREEZE_INTEG.inputs与旧比较/O5夹具，选cuboid和chamfer两种hull、0/5/20箱、30k/100k有效点。准备工具按这些确定性字段选择唯一fixture并冻结实际路径/hash；缺某组合则用现有生成器seed=20261104补齐，保存全部原始点、ray origin、TF、时间戳、几何及参数。不能以混合箱数代替20箱。选定结果写INPUTS.json，评分时不重新生成。

共同预处理仅做一次：光学Z有效范围及同采集时刻TF按选定配置冻结；记录原点及实际端点，不只给XYZ。两侧收到逐帧相同的规范输入和完整历史。测试GT只供独立质量判官，不能作为任何后端或使用方占用输入；两侧都不注入额外已知箱体锁。

| profile | 输入/配置 | 可得结论 |
|---|---|---|
| FULL_RAY | 同一规范点云及端点/原点、2cm、同hull；G关闭射线限额，O discretize=false；记录实际送入/保留/处理计数 | 同输入/无采样限额下的后端系统成本，内部遍历/更新次数差异如实报告 |
| DEPLOY | 同一原始帧，但分别固定各自当前部署过滤/采样参数；输出实际保留点、射线和更新格数 | 当前部署效果，不称等工作量算法加速比 |

两侧不强制使用同一概率内部公式，按各自已冻结概率/分类规则验证；分类混淆矩阵和surface高度差是比较结果，不设跨后端零差异门。

## 5. 一次冻结的比较矩阵

### B1 更新与公共查询：24组

2 profiles × 2 hull × 3箱数 × 2点数=24组。组按(profile FULL_RAY/DEPLOY, hull cuboid/chamfer, 箱数0/5/20, 点数30k/100k)字典序编号；奇数组G→O→O→G，偶数组O→G→G→O，共四个独立进程块。每块地图reset至同一初态、回放同一33帧序列，3帧预热+30帧计分。每后端共60次计分，分块报告并给组合统计。块内累计地图，不每帧重置；单次测试流程为更新→该输入可读→acquire→完整Surface→同域ROI全部页→release。同帧未完成不偷偷取旧图。

每次操作30s超时，超时计入失败；不补样择优。先完成后发下一帧，得到单链成本及吞吐上限，不把此串行值称开放环2.5Hz能力。冷首帧另记4块的首帧，不以每侧2次冷样本计算P95或宣称冷启动验收。

### B2 真实选位：12组

2 profiles × 2 hull × 3箱数，固定30k帧。沿B1相同G/O/O/G块、3预热+30计分；待目标图被真实消费者接纳后调用共同C/C'的ComputePlacementV2（既有v1入口同样通过兼容门但不重复完整性能矩阵）。两种探测箱(.30,.20,.20)、(.50,.30,.25)m按请求奇偶交替；其他solver/atlas/Attachment配置完全相同并冻结。

记录输入就绪→地图可读、消费者接纳等待、服务往返、solver、完整体积验证及全链时间，避免重复相加。规划侧只执行实际接纳和输出校验，不发运动。合法NO_KNOWN_FREE纳入有效完成终态与耗时分布，不能当作失败剔除；有效候选数量/安全合法率分开报告。B2沿B1的奇偶平衡块顺序，输入每步仍更新，不可只反复查询不变旧图。

### B3 部署目标负载：6组

2 profiles × 3箱数，chamfer、2cm、30k。每组一对G/O，各120s，组序交替GO/OG；每个后端连续2.5Hz输入300帧、2Hz完整公共查询240次，绝对时钟调度，预热3帧且就绪后才起窗。结束最多30s排空，未闭合ID不删除。此处不夹入额外选位请求；选位成本由B2衡量。没有额外120分钟稳定性或压力扫频。

业务结果记录成功/合法空结果/拒绝/超时/丢失与积压，以及freshness/health。能否维持2.5/2Hz按300/240期望请求与实际截止/闭合报告；不因任一侧慢而偷偷降低该侧输入频率。该组是新双后端相同负载画像，不重签旧reset/P-H合同。

## 6. 指标、环境与判定

锁：父进程持`/tmp/elfin-occ-backend-perf.lock`覆盖A/A资格与同run评分。环境：当前x86、ROS2 Humble、O Release/OctoMap1.9.8；同Python/NumPy，按manifest实际记录。共同机型/线程数/affinity/governor在准备时冻结：双方后端及共同使用方都用同一可用CPU集合（启动时父进程被允许的CPU与online的交集，精确集合写入manifest），不套结构microbenchmark的单核CPU0。governor保持现状并记录起止，不新增特定值硬门。原O reset是无绑核profile，本轮标为独立comparison profile，不能冒称复验旧环境。

同一输入的协议资格：B1代表组FULL_RAY/chamfer/20箱/30k，对G/G和O/O各按相同四块/3+30流程执行一次；更新与公共完整查询的P95，两个同后端块max/min≤1.10才取得该后端本run时间比较资格。不得以G/O比值判工具资格。资源与正确性数据可在时间无资格时保留，但不签时间优劣。内存采集用进程树RSS/PSS（100ms采样）及后端已有唯一资源计数，采集不可见项写unknown；不把tracemalloc代替RSS，不把采样峰称瞬时绝对峰。

| 指标 | 汇总/判定 |
|---|---|
| 时延 | median、P95=ceil(.95n)排序项、max；B1/B2每侧每组60个终态；成功子集与全部终态分列；至少30个有效完成样本才给该操作成功P95，否则NOT_EVALUATED |
| 失败与闭合 | 每个预期input/request ID恰一终态，重复/缺失/迟到具名；失败不从分母删，超时不当快速失败的性能优势 |
| CPU | 进程树user+system增量，秒/输入、秒/完整query、累计核占用；记录进程生命周期，不用单节点CPU冒充全系统 |
| 内存 | 进程树峰值RSS/PSS、已实现资源计数及采样周期；共享页不按多个进程重复当唯一物理字节；PSS不可读明确未知 |
| 正确性 | 各自独立oracle及共同几何/身份合同，错身份/假free授权/非法候选零容忍；跨后端分类/高度/候选差异只报告，不要求字节相等 |
| 速度比较 | 每个同配置同操作分别报告G/O绝对值与比值，不设“必须赢10%”；不能以某单指标决定整体优胜，不跨profile求总加速比 |

不将旧1cm、O5/O6、1Hz、distance压力门偷偷继承为本次通过条件；原结果作为历史单列。没有独立硬性SLA的新性能指标只报告，不以事后阈值返工。32GiB总证据预算、8GiB/run；进程树RSS超过2GiB持续3个100ms样本视为既有资源保护触发，停止受影响配置并保存，记录RESOURCE_LIMIT，不无限耗机。该保护不同于精确瞬时资源认证。

**判定四分开：** 工具/证据有效性；使用方兼容PASS/FAIL/BLOCKED；后端逐组比较完成/FAIL/NOT_EVALUATED；共同集成准备状态。更慢不是比较失败；正确性错误才停止依赖它的业务组，其他独立组可完成。单侧缺能力时不得虚构另一实现填数；保留公共操作数据，完整使用方兼容保持BLOCKED。

矩阵是有限样本AND，最多只证明这些组内零观测违例；不推断实机每单位可靠性，也不增加连续N次全通过campaign。

## 7. 工具、预演与执行交付

准备阶段在T实现一个明确入口`python3 scripts/occ_compare_g3.py`，子命令合同如下（当前尚不存在，须作为工具交付，不得称已可运行）：

```text
inventory --grid ROOT_G --octomap ROOT_O --consumers ROOT_C --out PREP
prepare-inputs --profile-spec PROFILE.json --out PREP
verify-compatibility --manifest PREP/MANIFEST.json --out COMPAT
rehearse --manifest PREP/MANIFEST.json --out REHEARSAL
freeze --manifest PREP/MANIFEST.json --out FREEZE
score --freeze FREEZE/MANIFEST.json --matrix B1,B2,B3 --out RUN
judge --run RUN --out VERDICT
replay --run RUN --case CASE_ID --out REPLAY
```

也可用等价现有入口组合，前提是交付COMMANDS.sh逐阶段精确argv、非零退出传播和manifest绑定，不能留占位符/手工隐含步骤。构建命令与环境source顺序同样纳入；runner遇错误不自动跳过某阶段并报总PASS。9cc59da1 driver采用与否均须明确，不能因旧人工R补跑丢失而复用不完整自动链证明。

预演只跑每profile一个20箱30k代表组，每侧3输入+3查询+2真实选位，及§3十状态夹具；验证实际接线、身份、租约释放、工件与判官。工具负例固定覆盖错输入hash/帧身份、重复或缺ID、少页/错页、过期旧epoch、将unknown标free、非有限数/缺样、不同配置混合比较、子进程失败被吞；每类至少一正常控制和一负例，负例必须非零退出。不得添加与本合同无关的新负例类别阻断。

先完成inventory/兼容预演/工具，冻结准确T/C'/install/输入/配置/commands再评分；不要求另一次泛化审批。若必要使用方BLOCKED，冻结文件明确禁用其B2计分，其余公共比较仍可执行。测量开始后产品不改，工具变化终止受影响run并保留证据，重跑仅受影响项。

## 8. 预算、停止与证据

准备、构建、适配与非计分预演合计≤8h owner活跃墙钟；正式测量≤8h（不含等锁），不自动续预算。这是本代比较预算，不借用已结束reset/结构任务的预算。正式顺序：资格→B1的30k组→B2→B3→B1的100k组。不足时剩余NOT_EVALUATED；不按跑得快的组补齐总数。每个普通块≤30min、完整更新/查询/选位链绝对deadline30s（重试/分页不得延长）；B3固定120s+30s排空。正确性失败先持久化再停依赖组，性能较慢继续其他独立组；无盲目重试或择优轮。

保存：MANIFEST/INPUTS/CONFIG/CAPABILITIES、COMMANDS、实际argv/退出码/导入hash、逐input/request/快照身份与原始时间、实际工作量、CPU/RSS/PSS、oracle输出、全部终态、独立可重算judge与FINAL_REPORT。失败case保存输入历史引用及最近3帧、阶段边界输出和前10s后5s诊断窗口；保存失败所需产物后才拆进程。每份summary必须由盘上原始文件可重算。

只需一轮有效比较，不要求每个后端都更快/每个指标都满足旧SLA。未完成或BLOCKED也可完成工程归档，不能改称全面兼容/可上线。

## 9. 后续共同集成的边界

比较报告完成后，用当时干净master新建独立integration worktree，按已确认能力矩阵选取G/O、共同接口和C'，明确依赖与冲突清单；适配/代码变化跑受影响回归及真实无运动消费链。提供可审查diff和每组证据，不直接合并两条数百commit开发历史。本任务只准备候选，不自动merge、部署、Gazebo或实机运动。

后续review仅按本文件已写条款指出具体违例，不能逐轮增加分辨率、箱型、长稳、追因、实机或新门限。新增需求另立任务；本轮只交付两profile、上述B1–B3和既定使用方兼容结论。

## Pointers

- docs/agents/reviews/2026-11-04_2132_occ-next-plan-status.md
- docs/agents/reviews/2026-11-04_2129_octomap-gen4-rule-adjudication.md
- docs/agents/reviews/2026-11-02_2001_occ-structure-final-acceptance.md
- docs/plans/OCC_DUAL_BACKEND_CONSUMERS_G3_20261104.md
- docs/plans/OCC_DUAL_BACKEND_DATA_ACCEPTANCE_G3_20261104.md
