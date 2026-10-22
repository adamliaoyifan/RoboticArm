# Direct portal first：实现与两轮闭环验证

## 结论

用户要求的直达优先策略已落地，并完成两轮冻结版本闭环尝试。
两轮都选择 direct、通过路线认证并执行完 transit/traverse/insert/descend；
均停在释放确认 KINEMATIC_RELEASE(kinematic presence refused (no_message))。
两轮完整闭环通过数0/2，各尝试1箱、装入0箱；不能报告n=3闭环通过。
没有启动第三轮，也未在两轮之间修改代码或裕量。

## 实现与验证

- 计划a5cd0c3d，产品版本b7ebea8cdf30d691a009b7ac5b4aa74b67e9af97。
- DirectFirstRoutePlanner统一graph/lattice的拓扑尝试顺序。首先直接home→portal；
  失败才生成staging IK并做携箱/空载端点检查。直达成功时不生成staging。
- 有staging基准时分配剩余候选预算60%给直达，其余与staging准备/建路共享原截止时间；
  无staging基准则全部给直达。重试不能延长候选原截止时间。
- staging检查未得到明确有效结果即拒绝该备用分支；每条完整路线仍用原认证入口。
  保存topology_attempts的预算、耗时、结果和selected_topology。
- 同步修改placement_prior规范。保守包络、碰撞许可、Attachment60s时效不变。
- 80项针对性行为测试通过：包括真实graph管线的直达成功不碰staging、直达失败后
  合法备用可用、无效/未知端点不连接，lattice直达优先、共享截止与重试不扩预算。
  见pytest.log。luggage_planning构建成功，两个修改模块安装字节与源码一致。
- startup_01非功能启动预演通过：单clock、相机正常、URDF保存、gain0.8；随后清理。

## 两轮结果

相同seed0、inch20、A-prime、180s Select诊断预算、prior off、kinematic release；
每轮目标3箱，首个功能失败停止。两个manifest均为b7ebea8c，dirty仅既有未跟踪
pydeps-frozen链接。两轮配置哈希一致。启动失败0次，功能运行2次。

| 项目 | live_01 | live_02 |
|---|---:|---:|
| 尝试/抓取/装入箱数 | 1/1/0 | 1/1/0 |
| 已尝试槽位 | 3 | 1 |
| 选择槽位/拓扑 | 11/direct | 13/direct |
| Select总时长 | 38.740s | 12.731s |
| 成功direct建路耗时 | 8.26s | 11.18s |
| 已认证路线/段数 | 1/8 | 1/8 |
| 实际执行成功段数 | 4 | 4 |
| descend fraction | 1.0 | 1.0 |
| 首段transit名义时长 | 240.224s | 196.436s |
| suite wall | 321.3s | 253.1s |
| 终止 | 释放确认no_message | 释放确认no_message |

live_01槽13、12先直达失败，再进入staging备用检查，分别耗时约0.20s、0.15s后
拒绝；槽11直达成功。live_02首槽直达成功，无staging端点生成。说明目标顺序已经
在真实运行中生效，不是仅靠测试桩。备用成功路径仅有离线行为测试覆盖。
与先前G4两轮选路预算耗尽相比，本轮观测到选路与前四段执行通过；样本及随机
路径不同，不声称一般成功率提升或用耗时差估算稳定性能收益。

## 释放失败证据

两轮trial的release_kinematic均保存set_pose=ok，随后presence_evidence记录
verdict=refused、reason=no_message以及model/epoch_floor_ns。
两轮状态ABORT_RELEASED，未进入正常退臂、最终verify或占用提交，所以0 packed。
这只能定位到释放位姿确认缺失，尚未证明根因是消息发布、订阅、时序或执行器。
不能用set_pose返回成功替代新鲜位姿确认，也未降低验证门。

失败dump保存trial、release字段、submitted_legs、joint_ring、last_boundary、
perception_refs、surface和scene摘要。manifest标capture_complete=true，但stage=descend；
它不是释放消息原始流的完整回放证明。Select前完整scene CDR、原始请求/响应、
expanded URDF、物理follow/plugin流另在run目录保存，可继续诊断。
本轮没有独立完成物理几何的连续碰撞复核，因此不把执行成功等同于实机安全证明。
此前发现的初始scene附着位姿不一致、名义高度锚点输入未在本次混入修复。

## 产物与复现

- live_01/metrics.json、live_02/metrics.json：结果及逐拓扑统计。
- 各run的selection_boundary/*/response_rows.json保留完整精度拒绝/成功行。
- 各run的driver_out/dumps/*/trial.json保留失败边界和已提交轨迹。
- 各run的manifest.json、startup_preflight.json、teardown.log；总teardown_audit.json。
- artifact_manifest.json对所有本地证据记录长度和SHA256；大型采样二进制保存在
  集成worktree及主工作区同路径，Git仅收紧凑证据（不将缺省未入Git误称为缺失）。
- summarize_run.py只读原始输入并派生metrics/response_rows；在source ROS及workspace后
  python3 <此目录>/summarize_run.py <run目录>。

两轮按原计划已完成，无完整装箱周期样本：cycle latency not_evaluated(n=0)。
不以2轮结果推断总体可靠性。最终进程核对无遗留Gazebo/bridge/domain7产品进程。
