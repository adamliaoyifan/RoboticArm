# summary wire局部修复82955d6f审查

- reviewer: reviews/codex/gpt-5
- verdict: not_accepted_for_B_rebinding
- scope: 现有三字段修复合同，不扩展性能/生命周期任务

## 已核平

diff确为两个wire产品文件和专用test。独立运行无ROS的schema/缺键/不可变3测：3 passed,4 deselected in0.14s；未独立复跑1456全回归或真实ROS。

逐文件重读新install，共2748文件，SHA256=273322273951cf7ea2c33b32c9c2a0dfbb55e3ccc101cc1468c692641ebc3171；IDL与原绑定182项逐定义hash全等。manifest的相对listing文件不在证据目录，实际位于/tmp/elfin-occ-summary-wire-fix-g1/files_candidate_swf.jsonl（未跟踪）；从此路径核对逐文件清单成功。交付前移入证据目录或写明确绝对指针，不把临时树未跟踪文件当长期工件。

## P1 错误类型被静默转换为合法事实

_MapSnapshotView仅检查键是否存在，随后bool/str/int强转。真实解码器直接探针：

| 输入 | 实际接受值 |
|---|---|
| occupancy_confirmed='false' | True |
| change_event=None | 'None' |
| pending_changes=[[123,1.9]] | (('123',1),) |

计划§3明确三字段错误类型拒绝6例的一半，但本提交未验证类型。必须按wire格式严验bool、str、二维条目形状、event str、token int（bool不是合法token），不把非法值修正为新事实；用稳定具名reason拒绝，并经真实receiver证明不能发布该坏代或污染旧pinned代。保留正常JSON list→不可变tuple转换。此项修复在原授权内，不改变owner语义。

## P2 缺键测试并未逐字段删除，冻结矩阵未覆盖

test_v2_missing_leg_is_named_rejection_not_default_fill初始provenance没有任何SUMMARY_LEGS，循环remove操作没有改变输入；三次都测“全部三键缺失”，错误串包含所有key使断言通过。需从完整合法对象分别删除一键，并校验只对应缺失项；还须实际wire/receiver路径拒绝，不只调用私有view。

计划16例=confirmed false/true×pending空/至少2条×event空/非空×FULL/DELTA。当前2个variant+3次空点云更新不是该矩阵的等价替代；未记录三字段真实发生哪些转换、实际manifest FULL/DELTA类型、跨代identity，不能仅以“有多代”覆盖。按原矩阵补有限case；不可达组合可标codec fixture，真实路径保留且不得谎称全无DTO。summary_fields只比较部分字段，漏identity/grid/values等；应业务字段完整对照，elapsed_ns单列。

## P2 恢复与typed门宣称超出测试

恢复测试改v1文件后，在receiver读取前又发布新代；第一次fresh.poll只断言非READY，没有断言UNKNOWN_SCHEMA、被拒绝manifest身份或恢复代确是FULL。该非READY可能是其他原因，当前不能证明v1拒绝→v2 FULL恢复。用屏障确保先观测指定v1拒绝及原因，再发布v2 FULL，最后校验完整身份和summary；不要依赖sleep和非READY大集合。

typed测试仅直接调用service.summary并做CDR往返，未创建client发真实ROS服务请求；build_pair也是同进程QueryReplica对象，并非独立OS进程。可以作为callback/序列化门，但不替代计划规定的独立reader及typed真实请求门。补一次有界无运动ROS服务往返/实际每代对照即可，不要重开仿真或性能矩阵。typed对照也须覆盖pending与完整身份、实际IDL对应全部业务字段。

## 最小后续

先类型拒绝+修正缺键测试，再补原有限矩阵和确定性恢复、一次真实进程/服务路径；记录精确命令/源版本/退出/输入输出。报告区分私有view fixture、真实组件、跨进程和DDS端到端，不以7测试数代替8类门。失败保留，不重跑A全部门。当前0da4086b历史Binding不前移，B接线准备继续而评分仍暂停。修复新SHA/install后只复核受影响证据，已通过IDL/清单口径无需另开诊断。
