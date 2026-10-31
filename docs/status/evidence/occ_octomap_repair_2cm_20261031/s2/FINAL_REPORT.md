# OctoMap S2 gen2 预演与 O9

- product_commit: 16dda82286b345b0a6f4208ac9c3f43621b8d8fd
- base: bfece5c32a0b4595827dd10993b88d4638c4f106
- plan_revision: c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76
- worktree_dirty_at_measurement: false
- scoring: not_run
- o5_o8: not_run
- resource_certification: NOT_EVALUATED
- s3: closed

等待检查把 8 个请求移出队列后，在途总数仍把它们算进去。交错屏障下的实测是 `checking=8 queued=0`，第 9 个返回 `WAIT_PRESSURE`。功能预演关闭行为 hook，客户端执行器 4 线程，发送线程不处理响应。

## 预演

11/11 通过，退出码 0。启动 health、TF 拒绝捕获与重放、缺字段包不能重放、2 秒内恢复等待、三个独立发送各一个终态、资源认证保持 `NOT_EVALUATED`、在线 trace 页脚可重放、去掉页脚后不能重放。

## O9

同一 `observer_profile_cli`，30 对 AB/BA。30k 帧 sha256 `ca45605ffb958a9c6e83f2ee2a3b4a55e2b3ef9f0d545fe9f6853b5cee49419a`。首次 Surface 为 cache miss，第二次为 hit。官方判官 `pass`，reasons 为空。

| 操作 | ON/OFF P95 比 |
|---|---|
| ROI | 0.9963607021704047 |
| 30k 融合 | 0.9887859269007071 |
| 首次 Surface | 1.007934487148608 |

门限是 1.10。不完整注入包 `replay_possible=false`。完整 epoch 拒绝包可以重放。性能认证只覆盖这三组干净样本。

## 环境

- 安装：`/tmp/s2-install`，先 `/opt/ros/humble/setup.bash`，再 `/tmp/s2-install/setup.bash`
- 节点 sha256：`35716c796a762c2d74c35e959861e158078364dee07b896e499b1b5139738637`
- profile CLI sha256：`19703076fb0c2fca86e5ee1b2ee6ee87423b528f633a24ac94f5cfac75388146`
- RMW：`rmw_fastrtps_cpp`
- governor：`powersave`，不绑核，逻辑 CPU 32
- 锁 `/tmp/elfin-occ-backend-perf.lock` 等待 0 秒
- 峰值 RSS 70856 KiB（样本进程峰值，不是资源门）

## 资源门阻塞

`GetResourceUsage` 仍缺 OcTree 内部节点、目录 map 节点、账本和完成历史的 map 节点、入口队列节点、中间件缓冲、TF buffer、指纹表。O6 的读视图/缓存峰值 ≤ 512 MiB 不能用这些部分字节宣布通过。S3 资源门保持阻塞。

## 未执行

O5、O6、O7、O8 正式评分，Gazebo，planning，1 cm，生产迁移，公平比较。S3 仍关闭。命令已写在 `COMMANDS.sh`，本阶段执行到 O9 为止。
