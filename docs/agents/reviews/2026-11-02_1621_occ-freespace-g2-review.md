# 2026-11-02 -- free-space gen2 资格复核

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

ca28c885ce435dbda0453cf99ab0cc21e11c0b62不具备S3开放资格。27个FREEZE条目的实际字节全部匹配；18个gen2离线测试通过、1个未设置live环境的测试跳过。上轮具体坏例已有测试，但全合同未闭合，不能把owner pass作为reviews通过。S3 gen3仍dispatch_ready=no。未修改产品、未启动ROS/正式样本、未重测O9。核心F/C和owner O9数值保留；源码确认O9已新增独立acquire并计入时间，finally释放、显式4线程等修复保留。

## Verification

独立audit.py从owner健康对照逐项变异，不改变判官。三个原健康对照绿，以下八个坏例也全部reasons=[]：

1. P的300条终态全部改为同一个无关input_id。
2. P所有query结束时间早于计划时间，延迟为负。
3. R全部恢复回执和查询、过载查询清空。
4. R第二条查询INTERNAL_ERROR。
5. L除第一条之外所有14399条查询INTERNAL_ERROR。
6. L缺1800输入、1440查询（10%）。
7. L的RSS除首条外全部相同时间戳。
8. L缺整个10分钟RSS桶。

原健康R对照本就只有1条恢复查询；健康L只做2轮read/1轮reset，这些不足合同的夹具不应称formal健康对照。证据：implementation_g2_review/RESULTS.json、audit.py、pytest.log、freeze_checks.json。

## Acceptance

继承8caa0d44和fd84b8b5，不新增指标。按以下四组补全既有工具合同；必须覆盖所有索引而非只给上次反例加分支，给每条合同门提供实际adapter→artifact→judge健康/坏例映射。

### A 逐记录独立判定

freespace_judge.py:judge_r_stage与judge_l_stage查询循环有无条件break，只查第一条；P只比timeline长度不比ID集合，不计重复终态。删除这种抽样漏洞，检查完整(epoch,sensor,id)对应与唯一终态，原始回执不得receipt_timeline按ID保留最后一条而丢掉重复。三阶段所有输入/查询/健康/资源字段从原始记录独立复算，校验时间有序且非负/非bool/有限，完整身份与候选内容及sha不能仅检查字符串存在。正常与恢复沿原P/H全部门，恢复60条含首Wait，75输入；不得仅看首条或自报聚合。

L严格18000/14400、10min预热后110min逐请求及120s窗AND，不能允许90%。RSS按真实时间首个预热后10min与末10min下中位比较，不是按RSS大小排序取上下十分位。12个计划桶及评分11桶按合同完整覆盖、时间严格单调、每桶600样本；不得忽略整桶缺失或豁免最后一个计划完整桶。1000read/100reset必须逐轮实测所有权回基线，不能以自报轮数或最终句柄<=2替代。有限时长无泄漏不等于只查最终峰值。

### B 实际调度及租约闭环

stage_r当前恢复发送线程join(timeout=15)后才run_queries，正式75帧需约30s，后续查询不是同时按t0+0.5s开始；数量recovery_inputs//2为37，不是59。首Wait成功后未使用/释放其handle。修为并发且首Wait租约直接完成候选链并finally释放，之后59完整查询按同一时间表。

排空期没有60条query；in_flight列表是在额外receipt_timeline等待后按状态生成，未比较receipt时间与drain_deadline。必须用准确30s截止时刻，截止后的成功仍算未按时排空；额外观察只能诊断，不能改评分。

L虽改绝对时钟，仍只保留send_index，无逐输入发布时间/终态与查询计划/起止，无法检验P/H。需要同源逐记录采集。停止输入后才做1000查询，max_age1s将产生STALE；reset后空树路径无法触发有效租约错误释放；改成真实可观测生命周期行为，原错误租约必须finally释放，不能靠TTL替代。

当前R/L直接传frames（混合0/5/20箱），不是固定20箱；正式还循环6个短预演帧，需绑定合同生成器seed序列/输入manifest，不得拿短帧集合代替最终工作量。P九窗每窗独立文件含window_id；当前p_bNN.json同箱三窗覆盖。P失败应阻止依赖R/L，现在main仅收集结果继续全跑。保持原安全门可执行与预算收口；加入离线调度事件序列及非计分同源验证，不能再花长窗测启动顺序。

### C 冻结、磁盘判官与失败证据

COMMANDS的require_freeze未检commands字段、未查实际HEAD/当前dirty（只信freeze.dirty），空冻结字典也不报缺清单。候选后COMMANDS的judge分支还发生修改，git diff ca28c885..HEAD可见；需分别绑定产品/工具/命令实际SHA，不能用候选标签掩盖工具变更。27/27字节匹配是当前清单真实性，不证明冻结合同完整。

judge只查rehearsal/campaign而非scoring，P空glob不报缺窗；O9仍只读verdict.performance_certified，并未重算trace。这些和报告的磁盘重判声明不一致。实现全部正式文件存在/9窗集合检查、内容复算、O9独立重判；删页/删窗/删trace/伪造summary不能绿。失败前后边界与完整前缀必须拆卸前持久化，重放真实执行产品，8h预算和失败停止条件在调度器执行，不只写预算文本。

### D 资格与交付

修复前先回放本审查八个坏例；修复后应全拒，足额健康对照全过。建立字段×位置（首/中/末）×缺失/错ID/重复/非法数值/超门/内容损坏的固定矩阵，避免只验证首元素。实际R首Wait TIMEOUT有时间线不等于已定位；短接线先说明真实触发路径并验证正确顺序。完成同源有界接线、冻结后四操作O9及真实失败重放才申请S3；本任务不得启动正式长窗，不重开S2或改门限。

## Pointers

- docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
- docs/agents/reviews/2026-11-02_1112_occ-freespace-implementation-review.md
- docs/status/evidence/occ_freespace_final_20261102/implementation_g2_review/audit.py
- docs/status/evidence/occ_freespace_final_20261102/implementation_g2_review/RESULTS.json
- docs/status/evidence/occ_freespace_final_20261102/implementation_g2/FINAL_REPORT.md
