# G4 Attachment 保守几何迁移及两次闭环结果

## 结论与停止边界

已执行用户授权的两次闭环尝试，均受阻，停止进一步仿真/搜索调参，等待讨论。
Attachment 输出、场景附着、容量候选和碰撞认证的主要链路已统一；但最终源码
审计发现建路锚点仍有两处使用名义箱高，迁移尚未全部完成，不能报整体完成。
该遗漏属于本轮实现缺口，尚未用对照试验证明它是本次失败的根因。

## 版本和已完成行为

- 基线 `702e3c44`；计划 `6d9ee6dc`；不可变几何 `c02e7967`；
  全链传输与 planning 清理 `14816905`；入口/启动捕获 `d9050946`；
  预选容量确认与失败场景捕获 `6ebaac7c`。
- 测量尺寸与实际附着变换保留；Attachment 单一构造入口生成保守尺寸、中心、
  朝向及配置/输入摘要。ROS 消息往返保留摘要。planning 不再读容差配置或重算
  包络；旧的晚置槽位包络过滤及旁路已撤销，碰撞认证门保留。
- ComputePlacement 接收保守尺寸，输出转换回名义箱体目标；共享 slot/candidates
  曾导致中心变换两次，已用独立输出修复并保留失败及回归证据。
- 抓取前查询明确标记为名义尺寸预览，实际抓取后重新生成保守候选。预绑定目标
  必须通过实际保守候选确认，否则具名拒绝，不擅自更换绑定目标。
- 保留诊断参数：W/D 总增长10%，H只向远离接触面增长10%并叠加5mm位置界限，
  yaw +/-5deg，XY位置界限0；roll/pitch准确、箱体竖直的适用假设不变。

## 功能验证

- 核心 Attachment/消息/场景/容量/认证测试161项通过（1.33s）。
- 额外调用入口134项通过；另3项在基线 `702e3c44` 的独立源码归档上原样失败：
  两个prior测试fixture缺少_args，一个atlas测试仍期待已改成unknown的unreachable。
  它们未被计入通过数，也未改产品行为绕过。复现日志见 baseline_702e3c44_three_failures.log。
- 后续捕获/预选容量检查19项通过；计数有重叠，不相加声称总测试数。
- st09_contracts、luggage_msgs、luggage_planning、luggage_packing、luggage_gazebo
  普通安装构建成功；核心模块安装路径及源码字节一致检查通过。
- 启动预演通过；两次正式尝试启动均通过单一clock、相机流和URDF捕获门。
- 两次保存的Select输入均可重新编码成真实消息并通过request_dict准入，几何摘要不变。
  第2次完整场景里的attached primitive尺寸逐项等于Attachment输出；网格和ACM已保存。
  这是输入/场景一致性验证，不代表成功轨迹或物理连续碰撞验证。

## 两次闭环指标

同一inch20、seed0、A-prime场景、gain0.8、domain7、prior off；目标每套3箱，
遇首个失败停止。没有跳过箱子、缩小裕量或放宽碰撞许可。

| 指标 | live_01 | live_02 |
|---|---:|---:|
| 冻结运行版本 | d9050946 | 6ebaac7c |
| 尝试/抓取/装入 | 1/1/0 | 1/1/0 |
| 验证/提交装箱数 | 0/0 | 0/0 |
| 候选数/已试 | 650/119 | 650/122 |
| route build次数 | 49 | 52 |
| 已认证路线/携带段输出 | 0/0 | 0/0 |
| Select总时长(s) | 178.258 | 177.586 |
| driver wall(s) | 205.5 | 204.5 |
| EDGE_BUILD_FAILED | 70 | 70 |
| NO_FEASIBLE_ROUTE | 43 | 46 |
| ATTACHMENT_EXPIRED | 6 | 6 |
| 终止 | PLANNING_BUDGET_EXHAUSTED | PLANNING_BUDGET_EXHAUSTED |

两套均未完成3箱，功能套通过0/2；总尝试2箱、成功装入0箱。没有完成周期样本，
完整装箱周期时延为not_evaluated(n=0)。两次重复不支持总体成功率/可靠性推断。
认证入口因过期拒绝有样本，但重验证和成功执行没有样本；日志中的certify_ms=0
不应解释为认证已经通过或性能优异。

两次名义尺寸均约514.327x332.526x217.541mm，保守尺寸
595.487x413.696x244.295mm；yaw联合外包使W/D超过单纯1.10倍。
从旧791候选变为650候选符合容量生成开始消费保守几何；不能据此宣称性能改善。

## 失败证据和限制

live_01保存请求/响应、URDF、全部候选行、关节/插件流和失败dump，但旧场景只有
摘要且旧motion清单错误要求last_boundary。因此保留capture_complete=false，
不宣称完整场景回放。修复捕获后才启动live_02，没有盲目重跑。

live_02在Select前保存GetPlanningScene完整CDR（包含世界网格、附着体、ACM），
失败时冻结Select请求JSON和原始响应CDR；replay manifest为stage=selection、
capture_complete=true、replay_possible=true。此状态表示输入完整可供回放，
并非已经重新执行全部搜索并复现每个随机规划结果。两次原始证据均保留，未覆盖。
第二次额外捕获改变了观测开销，未测量该开销，不作两次耗时优劣比较。

## 待讨论项（停止后不继续改动或第三次评测）

1. **实现遗漏**：route_producer.py的_prepare_candidate两次anchor_plan仍传入
   request[box_wdh][2]（行885和909）。容量/碰撞使用保守模型，但初始建路高度
   仍用名义217.541mm；保守高度为244.295mm，差26.754mm。应先补齐这个输入，
   并添加检验实际锚点高度的行为测试；不能仅凭当前结果声称该修复必定通路。
2. **已观察时效拒绝**：每次6行ATTACHMENT_EXPIRED，部分路线进入认证后拒绝。
   180s搜索预算不会自动延长所有认证时效门。后续需要有界的重新确认/时效方案，
   当前没有擅自延长时效。
3. **剩余规划可行性**：中间姿态碰撞和路线拼接失败依然存在。当前样本只证明
   冻结候选/预算未找到可认证路线，不证明全局无解，也不能归因于所有历史异常。

用户要求两次仍block后先保存并讨论；已按此停止。任务保留open，未标记完成。

## 复查入口

- `METRICS.json`：逐次机器可读指标、版本、dirty状态和捕获清单。
- `live_01/selection_response.json`、`live_02/selection_response.json`：全部拒绝行。
- `live_02/selection_boundary/place-v2-2/`：输入、原始响应、完整场景。
- `scripts/g3/audit_attachment_geometry_run.py <run-dir>`：离线准入和几何一致性审计。
- 原始闭环命令：`PHASE2_RUN_OUT=<fresh-dir> bash scripts/g3/n3_functional_run.sh`。
  本轮次数已用完，该命令仅供记录，不表示已授权继续运行。

Teardown audit: no matching Gazebo/bridge/driver/runner processes after live_02; see teardown_audit.json. Retained artifacts total about250MiB, below the 2GiB budget.
