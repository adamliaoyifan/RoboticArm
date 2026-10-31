# OctoMap S2 gen5 观测传递

- scheduler: d5a309e1071ae54047fc91c6e587d36e33f73ba9
- o9_candidate: 7537b033daf20561c247bb7e09b6ee8805358a93
- base: 5d914bf71dcd821a0417281ab66bdd425ef89409
- official_o5_o8_samples: not_run
- resource_certification: NOT_EVALUATED
- s3: closed

组装层不再调用 `healthy_*` 或 `_resources_ok`。适配器没交回的查询、health、资源和耗时保持缺失。240 条全拒绝、以及 unknown/无回执/100000 ms，经 `schedule → adapter → artifact → judge` 后都不通过，产物里没有凭空生成的 120 条查询。

`ULD_OCC_S3_OPEN` 未设置时 stage 仍拒绝。设置之后，注入适配器会进入同一个 `schedule`。未再设置 `ULD_OCC_S2_CONFIRM_OFFICIAL=1` 时，live 路径不启动正式样本。`judge` 和 `replay` 失败返回非零。

## 验收

| 项 | 结果 |
|---|---|
| 1 拒绝/未知窗口不再被补成健康结果 | pass |
| 2 故障穿过调度路径，缺字段保持缺失 | pass |
| 3 开放分支分发到注入 runner；默认关闭；失败退出码非零 | pass |
| 4 离线 profile 按帧分目录，三次 surface 耗时各自保留 | pass |
| 4 实机 CLI 的第三次 surface（更新后） | not_evaluated：当前 CLI 只有首次和重复两次调用，没有把同一次耗时复制过去 |
| 5 离线时钟上的 2 Hz/10 Hz、排空 30 秒、首个 WaitSnapshot、O8 180 秒 × 3 与 10 秒 × 3 hold | pass |
| 5 实机长窗 | not_evaluated |
| 6 非计分 1 帧 CLI 和 1 次 ROS integrate，正式判官拒绝 | pass |
| 7 保留 7537b033 的 O9；产品 `src/` 与 `include/` 相对该提交无差异 | pass |
| 8 资源 512 MiB | not_evaluated |

重复核 30 条已保存 ON trace，仍是 30/30 完整。比值复算仍是 ROI 1.0156359705334954、融合 0.9809603064420823、首次 Surface 1.0257049963063405。没有重跑 30 对。

## 剪枝

帧末本来就会 `updateInnerOccupancy()` 和官方 `prune()`。八个同概率兄弟会收成父节点；同为 occupied 但 log-odds 不同不会合并；再补一次局部命中后父节点会展开，概率与官方树一致。

受控的 8 叶用例：合并后 16 节点 / 1536 字节，不同概率时 24 节点 / 1728 字节。该次剪枝耗时 0.00025 ms，展开 0.00009 ms。这只说明规则，不是整图收益。

一次非计分的 30000 点 stages 探测（不是 O9）：剪枝 1.500 ms，内节点更新 1.839 ms，写循环 24.376 ms，剪枝后的树是 177451 节点、4947696 字节。这个数字已经包含现有剪枝，没有一份关闭剪枝的对照，所以不能把它写成“省下了多少”。读视图和 surface cache 不属于这棵 OcTree，剪枝不会改写它们。资源门仍是 `NOT_EVALUATED`。

## 时间

离线测试约 1.2 秒。非计分 CLI/ROS 接线约 3.3 秒。剪枝探测约 0.3 秒。正式 8 小时预算未使用。
