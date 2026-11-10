# S2 非计分快照短链

- base: 4b71073d850db16194d8c4fa9dd94c7ca6d1f2c3
- o9_candidate: 7537b033daf20561c247bb7e09b6ee8805358a93
- official_o5_o8_samples: not_run
- resource_certification: NOT_EVALUATED
- s3: closed
- placement_bridge: not_implemented

终态查询的顶层状态是 `COMPLETED`，积分结果在 `receipt.status=INTEGRATED`。没有订阅回执时，`receipt_received` 仍是 false。`REJECTED`、`EXPIRED`、`NOT_SEEN` 不会被写成成功。

一条非计分 topic 输入：订阅回执与状态查询都匹配同一身份。`WaitSnapshot` 为 `OK`，年龄 12473579 ns。该响应的 `handle` 为空，因为服务实现没有填写 `WaitSnapshot.srv` 里的 handle 字段；短链随后用 `GetSnapshot` 取得句柄 `1:1`，15 页 Surface 与 4096 格 ROI 的身份一致，`ReleaseSnapshot` 为 `OK`。正式判官拒绝这条短样本。

同一进程的两帧 profile：首次 Surface 4.578498 ms，重复 0.0001 ms，更新后 2.36358 ms。正式判官同样拒绝。

已保存 O9 的 30 条 trace 复核仍是 30/30。产品 `src/` 与 `include/` 相对 `7537b033` 无差异。placement 转换和 512 MiB 资源门没有做。
