# S2 非计分 live 接线

- product_change: none in the observer; the runner is `LiveCampaign.wire`
- historical_o9: 7537b033daf20561c247bb7e09b6ee8805358a93
- o9_signs_this_candidate: false
- official_windows: not_run
- placement_bridge: not_implemented
- resource_certification: NOT_EVALUATED
- s3: closed

`LiveCampaign.wire` 实际调用累计 profile、ROS 发送线程和短稳定性采样。正式 30 样本、96 组、240 输入和 180 秒窗口没有启动。四段正式判官都拒绝这条短样本，`formal_pass` 为 false。

O5 和 O6 各用一个进程跑 2 帧累计 profile，三次 Surface 时刻都有数。O5 为 4.617776 / 0.000181 / 2.392208 ms，O6 为 4.633545 / 0.0001 / 2.386367 ms。判官理由是缺组。

O7 发送线程与执行线程不同。计划发送时刻是 0 和 0.5 秒，实测 0.000214 和 0.500616 秒，间隔 0.500402 秒。同一次查询耗时 0.717092 秒，没有把第二次发送拉到查询结束之后。两条查询都是 `WaitSnapshot` 句柄 `1:1`、`1:2`，15 页、7425 列、ROI 4096、释放 OK、`identity_stable` true。窗口判官仍拒绝：只有 2 帧、没有节点 callback 进入时刻、查询数量不足。资源服务返回 `NOT_EVALUATED`。

O8 只在约 1 秒内采了两次 health 和 RSS。官方 180 秒和 hold 没有跑。判官理由是 run_count、缺观测和 hold_count。

启动失败会写入 `failure/<stage>/manifest.json`。这次接线没有失败目录。owner 本轮相关 pytest 是 25 passed / 9.14 s，其中包含这条接线和此前的短链、持锁测试，不是独立复跑口径。
