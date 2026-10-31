# OctoMap S1 产品修复

- tool_commit: 32a3267e633cc79d0e912531fabd4684212b53e0
- base_revision: f28c0cf598faf90b9bec28a3a0168f65ecc65d72
- plan_revision: c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76
- worktree_dirty: false
- scoring: not_run
- capture_certified: false
- product_acceptance: not_run

S1 实现了计划 A–D 的产品路径，并用行为测试触发了锁持有、拒绝、reset、TTL、等待超时和资源所有权。这不是正式 O5–O8 评分，也不开放 S2/S3。

## 已触发的路径

- 积分线程持有 store 锁时，已有句柄的 `surface()` 和 `read_cells()` 在 200 ms 内返回，且返回时锁仍被持有。
- 同一段持锁期间，worker 的 health 缓存仍可读。health 发布不调用 `diagnostics()`。
- reset 之后旧句柄不能再取；已经取出的快照仍能读完。
- 默认 TTL 常量是 5 秒。测试把同一过期函数的期限设为 20 ms，句柄随后失效。
- 第 9 个句柄返回 `SNAPSHOT_PRESSURE`。
- 两个句柄共享块时，按指针去重后的块字节不增加。释放句柄后字节不高于释放前。
- 资源计数把 `std::map` 节点、OcTree 内部节点和中间件缓冲标成未覆盖。
- 入口队列容量 2。第 3 条以自己的 ID 返回 `QUEUE_FULL`，弹出顺序与接纳顺序一致。
- 终态账满 4096 时拒绝新 ID，在途记录不被淘汰。未满 120 秒的终态保持，超过保留期后变为 `EXPIRED`。终态先写入账本。
- 等待板第 9 个请求是 `WAIT_PRESSURE`。epoch 不一致是 `EPOCH_MISMATCH`。只有 map revision 前进、observed revision 未包含目标时继续等待。observed revision 到达后返回 `OK`。目标拒绝返回 `REJECTED`。无新帧到时返回 `TIMEOUT`。

## 命令

`observer_repair_s1_behavior` 退出码 0，输出 `repair s1 behavior passed`。

`observer_node` 在这份 CMake 构建里已编译。用默认库路径启动时缺 FastRTPS/introspection 类型支持库，这次没有完成一次在线服务调用。

## 未做

正式评分、1 cm 新测试、Gazebo、planning、生产迁移、公平比较。历史失败和旧立即查询的 `STALE` 保留。
