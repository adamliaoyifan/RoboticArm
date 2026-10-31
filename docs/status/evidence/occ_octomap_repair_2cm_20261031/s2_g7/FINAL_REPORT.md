# S2 WaitSnapshot 句柄与身份校验

- product_change: observer_node WaitSnapshot 成功时登记并返回租约
- historical_o9: 7537b033daf20561c247bb7e09b6ee8805358a93
- o9_signs_this_candidate: false
- official_windows: not_run
- placement_bridge: not_implemented
- resource_certification: NOT_EVALUATED
- s3: closed

`WaitSnapshot` 返回 `OK` 时句柄是 `1:1`，由 `SnapshotRepository::acquire` 登记。短链只读这个句柄，不再另调 `GetSnapshot`。15 页 Surface 与 ROI 的完整身份一致，包含 epoch、map_revision、observed_revision、stamp、geometry hash、config digest 和 clock。内容 sha256 是 `969cb7c308d20843f6ce707112387d2fb77e49b4370d163811676713ce387291`。释放结果 `OK`。年龄 12366233 ns。

离线注入的错误 epoch、未覆盖的 observed_revision、错误 map_revision，以及空句柄，`identity_stable` 都是 false。健康对照为 true。

终态仍是顶层 `COMPLETED`、回执 `INTEGRATED`。正式判官拒绝短样本。7537b033 的 O9 只作为历史结果保留，不为这次产品改动代签。
