# 2026-11-01 -- OctoMap S2 g8 复核

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

复核产品/工具提交 `abc41dd5`（owner工作树干净）。认可上一轮ROI缺样、真实调用方目标覆盖、WaitSnapshot持锁截止及即时ready名额修复；不等于完整S2或产品验收通过，S3继续关闭。本轮未修改产品、未启动ROS或正式样本。

## Findings

- 独立离线复跑20 passed / 2 live deselected。覆盖短ROI、身份错误、缺列/重复列/越界列与离线内容oracle等；owner 22 passed包括两项live，不能写成本轮独立22项。
- 实际调用方从完成回执传入observed_revision，从文件固定geometry/config/clock。ROI要求4096状态及等长log_odds、done和cursor=4096；Surface检查完整列集合。保存content_material，独立复算SHA为c53bb531ebfa562d61110495cf9ed2adccf02590483c8e313dd83a97ec1c90e5，与owner一致。
- 实机内容证据是可复算指纹与身份/完整性校验；错误格值拒绝来自离线提供的oracle。实际调用方未提供逐格oracle，不能扩大为实机每个log-odds已独立验证。
- 源码确认store试锁失败重排队，租约尝试计入8个名额，登记后复核截止并释放超时句柄。owner持锁测试包含300ms等待TIMEOUT、9th WAIT_PRESSURE、释放锁后8个成功及再次获取。接受为owner行为证据，本轮未独立重跑该测试。测试使用sleep等待而非显式持锁确认屏障，未来正式回归建议记录锁已持有事件和实际耗时，避免仅依赖调度时序；不把0.8s/0.5s测试上界写成测量值。

## Next

1. 补正式live runner及O5–O8真实适配器/采集。当前live路径仍返回official_long_window_not_started；短链和标签不能替代整窗入口。先非计分验证累计profile、独立发送、完整查询和失败保存，缺样严格拒绝。
2. 补资源所有权覆盖并复核512MiB门；当前NOT_EVALUATED继续保留。
3. 产品与工具冻结后重测适用O9；7537b033只保留历史。达到S2完整关闭条件后再审核S3开放，不启动正式样本调试。
4. placement转换单独形成明确任务：坐标/矩阵轴序、unknown/free、同租约全页、epoch与observed revision语义。当前接口提供读链，但未形成placement可直接消费的适配。现有官方prune为等概率八兄弟合并，保留，不能视作新近似合并或资源认证。

## Pointers

- `/tmp/elfin-occ-octomap-repair-2cm-g1/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g8/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g8_review/RESULTS.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g8_review/pytest.log`
- `docs/agents/discuss/2026-10-31_1956_octomap-2cm-repair-s2-g5.md`
