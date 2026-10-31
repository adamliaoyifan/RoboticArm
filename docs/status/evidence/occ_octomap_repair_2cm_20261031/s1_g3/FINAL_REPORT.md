# OctoMap S1 gen3 资源所有权与终态补修

- base: 32a3267e633cc79d0e912531fabd4684212b53e0
- plan_revision: 0ee2da84331aa3da0c50afb00a79a307b8234a68
- scoring: not_run
- capture_certified: false
- resource_certification: NOT_EVALUATED
- s2_s3: closed

共享 Surface 缓存按对象只计一次。reset/release 之后，仍被读取者持有的块和缓存继续计入；最后一个持有者放开后，回到 reset 后的空基线（块 0、缓存 0）。topic 与 service 的完成和拒绝都先写入终态账并通知等待板，再发布。

## 已触发

- 复核 probe：同一快照，缓存 48000 保持 48000；reset/release 后读取者仍持 2 块 / 48000 缓存，报告分别为 8336 与 48000；8 个 cell 仍可读。读取者释放后块和缓存归零。
- 两版本共存时，旧版本独占的块计入 old view；旧读取者释放后只留下当前快照缓存。
- 并发响应字节相加，逐个释放后归零并保留峰值。
- 入口队列满、worker 队列满、TF 查找失败、service 成功、service epoch 不匹配，等待返回 REJECTED 及原因，不是 TIMEOUT。查询与发布的回执身份一致。NOT_SEEN 保持 NOT_SEEN。
- 重复 ID 不再次积分。账满逻辑仍拒绝新 ID。保留期过后查询为 EXPIRED。完成历史有 4096 上限，reset 丢掉其他 epoch。
- 适配暂停期间 reset，恢复后旧 epoch 消息不写入新地图（map_revision 0，known_cells 0）。
- 积分持有 store 锁时，health 仍发布，等待按截止时间返回 TIMEOUT。无等待者时不做快照。第 9 个等待是 WAIT_PRESSURE。恢复等待在 2 秒内返回 OK，年龄小于 2 秒。
- 单线程积分、全部有效射线和概率模型未改。

## 构建

- 编译器：`/usr/bin/c++` Ubuntu 11.4.0
- 离线库：`/tmp/observer-s1-g3-core`，`OBSERVER_WITH_ROS=OFF`，Release
- ROS 安装：`/tmp/s1g3-install`，先 source `/opt/ros/humble/setup.bash`，再 source `/tmp/s1g3-install/setup.bash`
- 节点：`/tmp/s1g3-install/uld_occ_observer/lib/uld_occ_observer/observer_node`
- 节点 sha256：`943ff5f983ca6b110b900f535d182c76d8b20914435bb4c16ec956def955ce68`
- 在线测试 `ROS_DOMAIN_ID=41`。未启动 Gazebo，未接 planning。

## 结果

- `observer_repair_s1_behavior` 退出 0，`repair s1 behavior passed`
- 复核 probe 退出 0，`failures=0`
- `test_repair_s0.py` 12 passed
- `s1_g3_node_behavior.py` 22 passed，0 failed，见 `live_checks.json`

## 未做

正式评分、1 cm、Gazebo、planning、生产迁移、公平比较。资源门未认证。S2/S3 仍关闭。
