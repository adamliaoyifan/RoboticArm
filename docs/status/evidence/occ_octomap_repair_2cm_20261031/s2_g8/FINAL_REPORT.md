# S2 缺样、目标覆盖与持锁等待

- product_change: short-chain judge requires a full ROI page and file-pinned identity; WaitSnapshot tries the store lock and rechecks the deadline
- historical_o9: 7537b033daf20561c247bb7e09b6ee8805358a93
- o9_signs_this_candidate: false
- official_windows: not_run
- placement_bridge: not_implemented
- resource_certification: NOT_EVALUATED
- s3: closed

非计分短链终态是顶层 `COMPLETED`、回执 `INTEGRATED`。`WaitSnapshot` 句柄 `1:1`。回执的 `observed_revision` 传入覆盖检查，geometry、config digest 和 clock 来自夹具文件而不是响应回声。Surface 15 页、7425 列（75×99）、ROI 4096 格且 `done`、`next_cursor` 4096。释放 `OK`。年龄 13233994 ns。正式判官拒绝该短样本。

内容 sha256 `c53bb531ebfa562d61110495cf9ed2adccf02590483c8e313dd83a97ec1c90e5` 由 `chain.json` 的 `content_material` 复算，与汇总一致。这个摘要是指纹。格值对错的 oracle 在离线对照里：相同校验拒绝 1 格、重复列、缺列、越界列和错误状态，健康对照通过。实机短链没有第二套几何实现来核对每个 log-odds。

持锁行为测试：目标 `frame-a` 已完成时，另一个 integrate 持有 store 锁，300 ms 等待在 0.8 s 内返回 `TIMEOUT` 且句柄为空。8 个即时 ready 等待占用名额时，第 9 个在 0.5 s 内返回 `WAIT_PRESSURE`。放开锁后这 8 个返回 `OK` 并释放，随后同目标再次 `OK` 并释放。

owner 本轮 `test_s2_g3_offline.py`、`test_s2_g4_offline.py`、`test_s2_g5_offline.py` 合计 22 passed / 5.39 s，其中实机是上述等待测试和一条非计分短链。这不是独立复跑口径。7537b033 的 O9 不为这次产品改动代签。
