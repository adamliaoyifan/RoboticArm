# OctoMap S2 gen3 真实重放与评测入口

- tool_commit: d653c9c84f2c47dc0daa0046ac57f26b30a9bb3b
- product_cpp: 16dda82286b345b0a6f4208ac9c3f43621b8d8fd
- evidence_base: 2d42aa6a2ea57248e992b2231d7738532f3c76c1
- plan_revision: 1f9dd9284383d6b33ebfb740ea9294749dd007bb
- scoring: not_run
- o5_o8_samples: not_run
- resource_certification: NOT_EVALUATED
- s3: closed

重放会重新执行产品。JSON 读回和 trace 页脚本身不是重放。审查里的伪造失败包和 drop=999 的无身份 trace 都是 `replay_possible=false`。epoch 99 的拒绝由 profile CLI 实际跑出，再在干净目录重跑，状态仍是 `EPOCH_MISMATCH`。TF 拒绝在两个先后启动的节点上各执行一次，原因一致。

## 本代 O9

目标是 2 cm chamfer，`config_chamfer_r0p02.yaml`，生成器 `o6-load-v1`，组 `r02_b00_chamfer_c30000_o50_x00`，outside 0.5，cross 0。ROI 4096 cell。首次 Surface 为 miss。30 对 AB/BA。每个 ON trace `capture_complete` 且 drop 为 0。OFF/ON 内容 hash 存在且相同。

| 操作 | ON/OFF P95 比 |
|---|---|
| ROI | 1.0007700033478406 |
| 30k 融合 | 0.9959667471663929 |
| 首次 Surface | 0.9551624793742002 |

资格是延迟门、捕获完整和 epoch 故障重执行的合取，结果 `pass`。旧 5 cm cuboid 比值 0.9963607021704047 / 0.9887859269007071 / 1.007934487148608 留在 `s2/`，不给本候选代签。

## 入口

O5 是 4 组 2 cm chamfer、0/20 箱、30k/100k、outside 0.5、cross 0、4096 ROI。O6 是 96 组 2/5 cm，每组 30 次冷启动。O7 是 ROS 九个正常窗、600 条 10 Hz 过载和 60 条恢复，不是 cold CLI。O8 是三轮 180 秒。O5 非 pass 则不开始 O6/O7。缺样和 `NOT_EVALUATED` 资源不通过。最终输入是可校验生成 manifest，点云字节 `not_emitted`，索引 sha256 `3b485878cbf4608bf30ca91984fb0689930412848e51b256e0690c5926c255d8`。本阶段没有执行这些正式样本。

## 环境与时间

- profile CLI sha256 `19703076fb0c2fca86e5ee1b2ee6ee87423b528f633a24ac94f5cfac75388146`
- 节点 sha256 `35716c796a762c2d74c35e959861e158078364dee07b896e499b1b5139738637`
- RMW `rmw_fastrtps_cpp`，governor `powersave`，频率 600000 kHz，不绑核
- 逻辑 CPU 32，不把它当作利用率。负载 0.41 0.20 0.14。上下文切换计数 141021696808
- 锁等待 0 秒。样本峰值 RSS 95624 KiB
- 本代预演约 4 秒，O9 22.5 秒，离线测试不到 1 秒。加上 gen2 的约 10 秒 O9，远低于 2 小时预演预算。正式 8 小时预算未使用

## 资源

读视图/缓存 512 MiB 门仍被未覆盖分配阻塞。S3 关闭。
