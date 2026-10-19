# BASELINE G2：带载 portal 根因、修复与冻结请求回归

- date: 2026-10-13
- parent: MASTER-PACK-20261013
- subtask: BASELINE
- generation: 2
- owner: eng / cursor / grok-4.7
- base_revision: e3a1968cd6f15a99c8f98ea16f0a02e78db0e585
- plan_revision: 派发线程固定为包含本文件的不可变 Git 提交
- scope: 离线修复和隔离 MoveIt plan-only 验证，不授权 Gazebo 或真实硬件运动

## 1. 目标与替代关系

本代替代 G1 的后续执行范围，不改写 G1 的失败历史或原验收。
解释并修复带载 portal 的无效目标选择，使保存的完整请求在原约束下选出完整认证路线。
G2 通过仅代表本冻结请求的离线回归通过，不代表原 B0-B6 全通过、两箱计分、n=3、容量或 S6/S7。

已知事实：e3a1968 的四个 PORTAL_TRANSIT 保存场景均被 certifier 判为
elfin_link1 / b-payload 碰撞；MoveIt 返回 -27。这不是 MoveIt 自身的接触列表。
请求有 828 槽，载荷约 0.4756802532 x 0.3234236131 x 0.4174256199 m。
尚未证明尺寸错误、附着错误或 portal 采样未考虑载荷；不得预先选择其中一个归因。

## 2. 单一 owner 与生命周期

| 单元 | Owner/model | 依赖 | 基线 | 验收 | 必需测试与提交 |
|---|---|---|---|---|---|
| BASELINE G2 | cursor/grok-4.7 | none | e3a1968cd6f15a99c8f98ea16f0a02e78db0e585 | G2-0 至 G2-5 | 本文矩阵全覆盖；产品/测试/文档提交及证据索引 |

Cursor 先在原 G1 线程用 `agent_mailbox.py stop-ack` 确认不再执行旧代，再对 G2
用 `scripts/agent_start.sh` 领取。不能由 reviews 代签 owner 停止确认。
新工作树建议 `/tmp/elfin-master-pack-baseline-g2`、分支 `agent/eng/master-pack-baseline-g2`，
从 e3a1968 建立；协调仍只写主工作区 `docs/agents/`。
不得带入主树未提交 seed-chain、评分、Atlas 或场景改动。

owner 负责实现、测试、安装、离线集成、证据及关闭，不另派 test。
通过后用 `scripts/agent_complete.sh` 提交 pass；外部材料缺失或超出授权范围时明确 blocked。
普通测试失败应在范围内修复，不把它当成可以停止的外部 blocker。

## 3. 冻结输入与执行边界

输入根目录（主工作区）：
`docs/status/evidence/master_pack_recovery/2026-10-13_g1/capture/`。

| 输入 | 身份与用途 |
|---|---|
| `select_capture/select_place_path_v2_00.bin` | 222500 bytes；SHA256 `33de07dd031b33bb2ec1ffaf36afbeaffe9d1ce40e08ebfa57e6c9dd0cf3e220`；原始完整请求 |
| `select_capture/select_place_path_v2_00.json` | 请求类型、大小与哈希索引 |
| `portal_capture/portal_00` 至 `portal_03` 的 JSON 与 `.scene.bin` | 4 个精确目标、起点、场景、ACM、目标约束和已保存认证结果 |
| `portal_capture/capture_index.json` | 88/88/36/36 次出现；不能推断重复场景等价 |
| `manifest.json`、`diagnosis.md`、`review_2038.md` | 运行身份、事实与捕获局限 |

不改变 slots 顺序、维度、grasp、start_joints、scene TF、occupancy、Atlas 内容/身份策略、
planner=graph 或现有通道族。请求 box_id/generation 不与其他箱子混用。
冻结请求与 payload 的尺寸/位姿分别按误差 <=1e-9 m、姿态夹角 <=1e-8 rad 核对，
允许四元数 q/-q 等价；规划变换结果按 <=1e-6 m/rad 核对。超差须记录第一处来源。

路线总预算仍为 60 s；认证仍为 2 s；其他限位、fraction、奇异性、连续性、地图、
支撑与执行门均使用 e3a1968 的解析配置，逐项导出并比较，不改门槛。
这不是吞吐基准：详细捕获耗时单列，但不从 60 s 搜索预算中扣除以制造通过。

## 4. 允许修改的行为范围

1. 核对已授权测量 -> Select 请求 -> 附着变换 -> b-payload -> portal 目标来源，
   给出值、frame、阶段身份和实际函数调用路径；缺原测量则写 not_captured。
2. 基于证据修复该链中确定的实现缺陷，或修复现有 portal 候选对载荷有效性的处理。
   合法替代可来自既有 portal/IK 候选集合，不得按 slot ID 写特例。
3. 非法目标在同一冻结场景下不得反复送给昂贵 MoveGroup 尝试；必须有明确拒绝记录，
   并在既有候选中继续找合法方案。采用何种实现由 owner 决定，不强制引入缓存。
4. 修正诊断器的缺失限位误判，并用结构化 XML 解析 URDF。捕获去重须区分不同请求、
   场景内容、附着与 ACM；语义等价身份不能直接依赖非规范的 CDR padding 字节。
5. 补必要的 focused tests、失败捕获和邻近文档，复用现有生产构造/回放接口。

禁止：扩大 touch_links 或 ACM 豁免 link1、删除载荷/障碍、缩箱、改目录尺寸来夹测量、
更换 TF/Atlas/箱型、切 lattice、引入 seed-chain/学习策略、扩充新通道族、增加预算。
GT/目录只可作离线诊断对照，不可取代冻结请求的生产测量。
若根因必须修改上游感知、硬件安装或候选族，保存反证并请求下一范围，不在本代悄悄改请求。

## 5. 环境和可执行验证程序

使用 ROS Humble 与隔离 MoveIt，`ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`，
无 Gazebo、无控制器执行、无 `/clock` 依赖。先解除外来 PYTHONPATH，清环境 colcon 构建。
从生产模型生成路径恢复相同 URDF/SRDF 和碰撞几何，不使用 mock 模型冒充，也不替换 hash。
请求文件字节保持不变。历史时间的离线适配只能在独立 harness 中显式建模：
场景不变，按冻结请求的有效时刻创建可核查的基准时钟，真实单调墙钟仍计入搜索预算。
不得冻结预算时钟；无法忠实初始化地图/附着合同则 G2-0 不通过，不通过重写时间戳绕门。

精确程序如下；owner 在 `commands.sh` 保存实际可复制命令、参数和环境：

1. 校验上述输入哈希，反序列化并作消息语义往返，核对四目标的起点和附着。
2. 在未改动的 e3a1968 上运行下述单测命令及受影响回归；保存 JUnit 失败 ID。
3. 隔离场景每次从 `.scene.bin` 完整重置。对四个原目标分别检查 URDF 与 MoveIt
   实际限位、约束、MoveIt 状态有效性及认证器结果；两轮，合计至少 8 个目标样本。
   固定现有结构豁免，显式记录原 ACM 和认证器组合差异；不同检查器不能冒称同一次输出。
4. 保存 portal 生成来源和使用的载荷/场景；在相同输入上比较修改前后目标拒绝、
   合法替代、MoveGroup 调用量及接触。不得只展示最终选择成功。
5. 修复后执行测试矩阵；提交并重建安装，核对源码/安装及资产身份，dirty=0 后冻结。
6. 通过生产构造和真实 Select 入口恢复完整 828 槽请求，执行两次 60 s 回归。
   每次新建 producer 并重置场景、地图和附着，清空请求级缓存；不复用胜者。
   保存实际运行参数（graph、预算、限制），使用原排序，允许生产算法选择不同胜者。
7. 两次均需完整认证通过。产物齐全后先停自己启动的 MoveIt，再写结果与完成线程。

已有 focused 测试命令（在隔离工作树且 source 对应 install 后）：

```bash
python3 -m pytest -q -p no:cacheprovider \
  src/luggage_planning/test/test_portal_capture_boundary.py \
  tests/st09_parallel/I/test_portal_submission_capture.py
```

新增行为测试放入相应模块测试目录；实际全命令记入 commands.sh。
受影响回归至少包含 `src/luggage_planning/test` 与 `tests/st09_parallel/B`、`C`、`I`。
与 e3a1968 逐失败 ID 对照，不能只比较失败数量；新增 failure/error 为 0，相关旧失败不得跳过冒充修复。

允许非计分离线开发迭代；隔离 MoveIt 总占用上限 60 分钟（不含排队和纯单测/构建）。
启动就绪单次上限 120 s，未就绪不计请求样本，但计资源时间、记录失败并 teardown。
最后两次必须同一 clean commit；失败后可修复，但旧结果不丢弃，新提交重新做两次。
达到占用上限未完成，保留证据并报告预算外需另授权，禁止转成 Gazebo 盲跑。
其他代理占用同机仿真/规划资源时排队，不杀它的进程。

## 6. 验收与可靠性口径

| Gate | 输入/次数 | 通过条件 |
|---|---|---|
| G2-0 输入保真 | 1 份完整请求、4 份场景 | 大小/hash 对上；828 槽语义保留；模型身份可复核；无未经声明的输入/时钟替换 |
| G2-1 原因链 | 4 目标 x 2 冷重置 | 每例来源明确；每轮同一目标几何有效性结论一致；原 link1/payload 碰撞仍被拒绝；差异不可强行归因 |
| G2-2 行为修复 | 下述正负矩阵各至少 1 例 | 碰撞目标预拒绝、合法替代能被尝试，所有断言通过；缺失限位不是越界；无场景跨用 |
| G2-3 冻结请求集成 | 原 828 槽请求 x 2 冷重置 | 两次均 Select OK，选中路线完整认证，单次搜索墙钟 <=60 s，认证服务 <=2 s，安全合同未变 |
| G2-4 回归和版本 | 基线/修复各 1 套受影响 suite | 新增 failure/error=0；提交/安装源码内容一致；最终两次同一 dirty=0 修订 |
| G2-5 证据与 teardown | 每次有效/失败尝试 | 必需输入输出可关联、重放入口齐全；最终自有 MoveIt/bridge 残留=0；不把本代记成在线放置 |

G2-1 是固定目标的确定性有效性回归，每例要求 100% 一致。G2-3 是两次请求的 AND，
本任务要求固定夹具 2/2 完整成功，不能一胜一败取平均判通过。
它只筛查该请求的可重复性，不能区分总体 99% 与 90% 的系统可靠率，不签总体稳定性。
要声明人群/生产可靠率必须另设多场景样本计划。本门含原碰撞负例和必须有完整合法路线的正例，
“全部拒绝”的系统不能通过。任何必需单元不足最少样本记 not_evaluated，而不是 pass。

行为测试矩阵：

| 输入变化 | 必须观察到的行为 |
|---|---|
| 携带载荷碰 link1 的 portal | 命名拒绝；该非法目标 MoveGroup 提交次数为 0 |
| 该目标被拒绝后有合法替代 | 同请求继续，合法替代进入建边；不能整体宣布 BIN_FULL |
| 所有允许 portal 都失败 | 有限结束、结构化未解原因；不提交非法动作、不声明物理不可达 |
| 同 q，scene / grasp / 尺寸 / ACM 任一变化 | 分别独立记录和验证；不得复用旧碰撞结论 |
| 同条件重复目标 | 不重复昂贵规划已判无效目标；记录次数和拒绝依据 |
| 两目标三位小数相同但精确值不同 | 捕获不合并 |
| 限位缺失、非法/不完整数值输入 | not_evaluated 或输入缺陷；不误报越界/有效 |
| 真正超限、目标约束不一致 | 各归到正确原因，不被碰撞类别覆盖 |
| 合法 XML 属性顺序/引号变化 | 读出相同关节限位 |
| 捕获文件写入失败或检查器超时 | 明确证据不完整/未评估，不冒充成功或确定碰撞 |

## 7. 证据矩阵与保存

输出根：`docs/status/evidence/master_pack_recovery/2026-10-13_g2/`，
按 base、diagnostic、final_run01、final_run02 分目录，失败追加编号，不覆盖 G1 或旧尝试。

| Boundary | T1 | 触发 | T2/T3 | 窗口 | 格式/上限 | 复核程序 |
|---|---|---|---|---|---|---|
| Select 输入 | request ID、hash、参数、计时 | 缺失/身份不符 | 完整请求、模型配置与输入映射 | 该请求调用前至响应 | bin/json，单例 5 MB（网格共享） | 哈希与语义往返；程序第 1 步 |
| portal 生成 | 目标来源、q、载荷/场景身份、预拒绝原因 | 碰撞/来源差异 | 四目标生成链与合法替代，IK 来源及带载检查 | 每条失败边从目标生成至响应 | jsonl/bin，单请求 25 MB | 程序第 3-4 步 |
| MoveIt/认证 | 精确目标、错误码、contacts、耗时 | 无效/超时/检查器分歧 | 调用前场景、完整 ACM、实际请求响应、有效限位 | 对应调用前后边界，不录整栈 bag | json/bin，单请求 25 MB | 冻结场景重新调用对应接口 |
| Select 输出 | rows、计数、阶段与预算耗时 | 非 OK/超预算 | 完整胜者或首失败路线、认证输入输出、全部尝试行 | 本次 <=60 s 搜索及响应 | json/bin，单请求 25 MB | 程序第 6 步完整回放 |

每轮运行上限 100 MB，共享模型资产另存一次，总保留上限 500 MB，文本可压缩。
超过上限先停并记录原因，不能删除失败重放输入换空间；不另录摄像头或控制器流。
T0 manifest 含 commit/dirty、CPU/内存/软件版本、模型与资产哈希、参数、时间、命令、文件清单、
capture_complete/replay_possible、缺项和 teardown。stat/反序列化/hash 为 capture-health 门。
捕获写盘计时单列；不预设零开销，也不据此比较 G1 与 G2 的速度改进。

交付：`input_audit.json`、`portal_provenance.jsonl`、`validity_matrix.json`、
`test_results/`（含 JUnit 和基线 ID 差集）、每次完整边界包、`commands.sh`、`summary.json`、
`verdict.md`、源码提交和角色笔记。任何根因结论需区分直接观测、源码机制与仍未证实的假设。

## 8. 规范与入口

- `docs/architecture/placement.md`
- `docs/architecture/privilege_boundary.md`
- `docs/architecture/frame_contracts.md`
- `.cursor/rules/debug-evidence.mdc`
- `.cursor/rules/sim-lifecycle.mdc`
- `docs/agents/reviews/2026-10-13_2038_portal-capture-review.md`
- `docs/agents/eng/2026-10-13_2025_master-pack-baseline-g1-portal-capture.md`
- `src/luggage_planning/luggage_planning/moveit_edge_adapter.py`
- `src/luggage_planning/luggage_planning/portal_goal_classifier.py`
- `src/luggage_planning/luggage_planning/route_producer_factory.py`
- `src/luggage_planning/launch/isolated_moveit.launch.py`

不恢复 G1 的两箱计分、不启动 n=3、不派发 SEED-CHAIN、不训练 policy；这些必须另行批准。
