# OctoMap 2 cm S0 合同与判官

- plan_revision: `c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76`
- product_base: `4c8f1862ec357cb01616f4b04be636227461cf44`
- tool_base: `57d8c0bfe23fee54920c89c06a2b6bdcb97de976`
- scoring: not_run
- capture_certified: false

本阶段冻结 96 组 2/5 cm 身份、O5/O7/O8/O9 清单和逐门判官。点云字节没有生成，观察器没有启动，产品没有修改。

历史 1 cm 失败 `r01_b00_cuboid_c30000_o50_x50` 的 P95 517.297808 ms 留在矩阵外，不改判。

判官排练 35 条。每条重放后原因码与直接判定一致。合成故障不是实机捕获资格。

| 用例 | 状态 | 原因 |
|---|---|---|
| healthy_normal | pass | none |
| missing_frame | fail | missing_frame |
| duplicate_terminal | fail | duplicate_terminal |
| all_rejected | fail | all_rejected |
| pre_callback_gap | fail | pre_callback_gap |
| health_gap | fail | health_gap |
| single_window | fail | normal_b00_r0:surface_ms |
| recovery_timeout | fail | recovery_timeout |
| wrong_snapshot | fail | wrong_snapshot |
| resource_missing | fail | resource_missing |
| pre_callback_overload | fail | pre_callback_gap |
| overload_not_observed | not_evaluated | overload_not_observed |
| perturbation_ratio | fail | perturbation_ratio |
| injected_fault | fail | injected_fault |
| o6_one_group | fail | group:r02_b00_chamfer_c30000_o50_x00 |
| o5_campaign | pass | none |
| o8_growth | fail | rss_growth |
| healthy_recovery | pass | none |
| healthy_o8 | pass | none |
| healthy_o9 | pass | none |
| overload_all_rejected | pass | none |
| recovery_one_rejected | fail | input_rejected |
| normal_duplicate_query | fail | duplicate_query_id,missing_query |
| o6_repeated_group | fail | duplicate_group |
| normal_wrong_snapshot | fail | wrong_snapshot |
| recovery_nan_latency | fail | nonfinite_value |
| healthy_o6 | pass | none |
| healthy_o7 | pass | none |
| o5_repeated_group | fail | duplicate_group,missing_group |
| o7_repeated_window | fail | duplicate_window |
| o5_parameter_mismatch | fail | parameter_mismatch |
| o5_unknown_group | fail | unknown_group,missing_group |
| normal_missing_identity | not_evaluated | missing_identity |
| health_exact_second | fail | health_gap |
| overload_one_reject | pass | none |

资源字节门只接受调用方已经按所有权去重的计数。缺归属时该门是 not_evaluated，整组不能通过。

S1 需要 reviews 绑定本次工具提交后才能领取。S0 的合成通过不能代替 S2 的真实捕获资格。

## gen3

- tool_commit: f28c0cf598faf90b9bec28a3a0168f65ecc65d72
- worktree_dirty: false
- pytest: 12 passed, including a 208-case numeric and missing-field matrix
- review reproduction: 9/9 matched, including 3 healthy controls
- illegal RSS and timing samples fail by name; missing samples are not_evaluated
- replay does not fill a missing timing field
- capture_certified: false
- product scoring: not_run
- S1 stays closed until reviews binds this commit
- s0, s0_g2, and s0_g2_review were not overwritten
