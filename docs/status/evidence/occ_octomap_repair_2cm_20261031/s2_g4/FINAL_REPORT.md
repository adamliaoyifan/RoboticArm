# OctoMap S2 gen4 执行器与严格捕获判官

- candidate: 7537b033daf20561c247bb7e09b6ee8805358a93
- plan_revision: 38dc898601fdf1bc994a966aa7287d193c05d746
- base: 967d01a63eb6862b838d370659d859958b1d868a
- official_o5_o8_samples: not_run
- resource_certification: NOT_EVALUATED
- s3: closed

O5–O8 走同一个调度器。离线适配器按计划调用全部组、窗、冷启动、发送/执行和 O8 hold；正式样本仍要 S3 打开，本代调用直接拒绝。页脚-only、缺边界、换 ID、重复、串 run 的 trace 不完整。30 个 100000 ms、NaN、Inf 或 bool 的积分样本不通过，完整健康组仍通过。短样本被正式判官记为缺样，不能代签。

## 旧 O9 复核

`/tmp/s2-o9-g3` 的 30 条 ON trace 在新判官下 `capture_complete` 为 0。ROI 和 surface 的 `request_id` 是空的。采集实现已改为把观察 ID 绑到这些事件上，所以原候选不沿用。没有把 gen3 数值拼进本代。

## 本代 O9

冻结候选之后重测。2 cm chamfer，`o6-load-v1`，组 `r02_b00_chamfer_c30000_o50_x00`，outside 0.5，cross 0，30000 点，ROI 4096，首次 surface miss、第二次 hit。30 对 AB/BA。30/30 ON trace 与预期操作和观察 ID 一一对应，drop 为 0。30/30 内容 hash 相同。epoch 拒绝在另一个进程重执行，sensor、reason、TF stamp 和 tree_nodes 一致。伪造包未执行。

| 操作 | ON/OFF P95 比 |
|---|---|
| ROI | 1.0156359705334954 |
| 30k 融合 | 0.9809603064420823 |
| 首次 Surface | 1.0257049963063405 |

资格是延迟门和这次捕获的合取，结果 `pass`。S0 判官对象里的 `capture_certified` 仍固定为 false，捕获结论以 `capture_gate` 为准。资源门不在这次资格里。

旧 5 cm 比值留在 `s2/`。gen3 的 2 cm 比值留在 `s2_g3/`，不给本候选代签。

## 非计分接线

同一调度路径对 1 帧 profile CLI 和 1 次 ROS integrate 产生了可解析回执。正式判官拒绝这两个缩减样本。ROS 发送线程和 executor 线程不是同一个。行为钩子关闭。没有启动 Gazebo，节点已停。

## 环境与时间

- profile CLI sha256 `8e668c95956ae97cfe4a2f44fd9f4ebfb5b31e649745426f066361275ebfa025`
- 节点 sha256 `d486f3791c61e76b3f1f3ca2c6f2be02029c9379ce717de2fc59f8ba2f23d8d8`
- eval native sha256 `e87408f3986d53a8c9998669c0a2cd7d10ee34b3d5a8f5b3ee232a30fc8354d2`
- 工作树在测量时干净，提交 `7537b033`
- RMW `rmw_fastrtps_cpp`，governor `powersave`，cpu0 频率 3634625 kHz，不绑核
- 逻辑 CPU 32，不是利用率。负载 0.80 0.41 0.23。上下文切换 141048599126
- 锁等待约 0 秒。峰值 RSS 142308 KiB
- 离线测试与非计分接线约 4 秒，O9 约 93 秒。正式 8 小时预算未使用

## 资源

未覆盖：`octree_internal_nodes`、`std_map_directory_nodes`、`ledger_map_nodes`、`ingress_queue_nodes`、`completion_map_nodes`、`middleware_transport_buffers`、`tf_buffer`、`fingerprint_table`。读视图加缓存的 512 MiB 门仍是 `NOT_EVALUATED`。

原始 pair 目录在 `/tmp/s2-o9-g4`。本目录保留比值、30 条 ON trace、epoch 包和可复核的 pair 摘要。
