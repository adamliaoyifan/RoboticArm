# A0/A'/B 批次版本清单（2026-10-19）

批 1（run 1-9）执行时与两轮指标推导所用 harness/配置版本对照。
run 1-4 的 per-run 资产哈希见各 `ab_run_NN/asset_record.json`；冻结输入
与判据见 `AB_RUN_ORDER.json`（批 1 commit 5f5a0ea4 的父提交即执行时点）。

## 批 1（run 1-9，2026-10-19 执行）

| 项 | 版本 |
|---|---|
| worktree HEAD（执行时） | 5f5a0ea4 前一棵（判定书提交 5f5a0ea4；runner/指标/资产生成脚本自 AB_RUN_ORDER 冻结后未改动直到本批执行完毕） |
| run_one.sh | 执行时版本：clean-room 仅 reap parameter_bridge ＋ 占用门（boot 前不主动杀 sim）；本轮已改（见下），**批 1 未含** |
| metrics.py（原始推导 run_metrics.json） | 执行时版本：停止规则=仅横向速度（后证会在零初速生成期误判）；无 verdict_scope/landing 字段；final 字段名 final_xy_m |
| record_box_stream.py | 执行时首版：argv 解包 3 槽 vs 传参 2 → 全程未运行，box_contacts.jsonl 九次全部缺席（缺陷记录见 AB_RESULTS.md） |
| scene_tf | 每 arm 变体 /tmp/ab_scene_N.yaml（容器模型唯一变量；基线=scene_tf_deploy_update.yaml） |
| 碰撞资产 | asset_record.json 逐文件 sha256（A0=install 内生产模型；A′/B=ab_generation_manifest 产物） |

## 批 1 的指标重导出（v2，2026-10-19 判定修正）

| 项 | 版本 |
|---|---|
| metrics.py | 本提交（停止规则=落地锚定+末次超阈下一采样；verdict_scope/landing_t_s/stop_method 字段；final_xy_displacement_from_spawn_m 更名；delivery-reorder 增跨实例拼接护栏） |
| 输出 | `ab_run_NN/run_metrics_v2.json`（NN=01..04，仅有效运行）；原始 run_metrics.json 原样保留 |
| 判定书 | AB_RESULTS.md 追记版（三修正：接触合同未评估、停止重锚定、A0→A′ 联合效应+n=1/1 边界） |

## 批 2（骨架，未授权执行）

见 `AB_RUN_ORDER_BATCH2_SKELETON.json`。触发条件：接触桥非计分预检
通过（`preflight_contact_bridge.sh`）＋用户一次性确定额度与顺序。
**批 2 不与批 1 数据合并**（分批判定，不拼"三轮"）。

**预检状态（2026-10-19）**：三跑后 **PASS**（preflight_contact_162308：
25,063 接触行、伙伴 airport_container_real、teardown 零残留；spawn ack
正常）。链条缺陷两枚已修并入库（Contact 消息形状错、回调抛异常杀整个
记录器）；五次 infra-abort 的机制根因（孤儿参与者＋/dev/shm fastrtps
锁）已由 boot 纪律 v2 关闭。批 2 剩余前置仅：用户一次性冻结额度与
顺序。
