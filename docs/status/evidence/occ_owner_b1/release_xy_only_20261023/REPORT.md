# 释放位置仅判定XY与n=3验证 — 2026-10-23

## 结果

按用户要求修复：计划a8f14d35，产品冻结8afbec19d1f0f60a89f519320826d30bfb12c23f。kinematic释放位置门使用XY欧氏距离<=5mm；Z偏差包括超过50mm均仅记录，不参与本项拒绝。原有身份、有效位姿、freshness、set_pose、包络、路径和最终settle/verify门保持原逻辑。新verdict=declared_xy_confirmed；新增XY/Z/3D误差、精确观测pose7、未舍入声明值和set_pose请求/响应主机/仿真时间。

68项相关测试通过，无skip；构建、安装源码一致性、启动预演通过。正式一轮n=3套件在首件异常后停止：attempted1、packed0、未尝试2。该件8段轨迹均执行完成，释放检查通过，日志进入COMMITTED、VERIFIED、HOME；随后_home_arm→goto_observe→typed plan_motion的readiness检查抛出PEER_LEASE_EXPIRED placement_planner。HOME仅是进入状态，不表示回观察位已完成。suite计数0表示未完成整个单件流程；不得把中间提交/验证状态改记为packed1。

套件292.0s，退出1，n=3未通过。只有一轮正式诊断，没有再次执行或修改冻结产品。没有现场Z>50mm样本的门限覆盖证明；正/负Z>50mm放行由离线行为测试验证。不得声称可靠性改善。

## 验证

- offline_tests.log：test_kinematic_release、test_release_pose_stream、test_pose_text、test_release_capture、test_payload_attachment_gate、test_release_failure_contract_driver，共68passed。覆盖正负150mm高度差放行、XY斜向组合>5mm拒绝及既有服务/消息拒绝。
- build.log：colcon build --packages-select luggage_gazebo --event-handlers console_direct+成功。
- installed_identity.json：已安装driver与源码SHA256一致。
- rehearsal_01：单clock、相机11.53Hz、gain0.8，启动到teardown正常，无箱子尝试。
- 正式live_01：2026-10-23T11:09:13+08:00开始，11:09:26driver开始，11:14:20driver退出，11:14:23清理结束。ROS_DOMAIN_ID7，seed0/inch20/A-prime/180s Select/prioroff，geometry hash及包络配置与上轮一致。无产品脏修改；仅预存未跟踪pydeps-frozen链接。
- 命令：PHASE2_RUN_OUT=<本目录>/live_01 ELFIN_RELEASE_CAPTURE_DIR=<本目录>/live_01/release_capture bash scripts/g3/n3_functional_run.sh。
- slot12、direct路线，Select27.567s，8段认证。原始selection request/response/planning_scene CDR及解码rows保留。

## 失败边界

直接观测：driver.log先记录return成功、COMMITTED→VERIFIED→HOME，接着1792725258.414044487主机秒记录placement_planner租约2.003704871982336s，随后增加到2.25s；traceback落在runtime.require_graph_ready。没有观测到本轮kinematic XY mismatch、Z拒绝或包络碰撞终止。不能由租约过期推断节点崩溃或根因是CPU/采集负担；原始心跳序列未采集，精确调度/心跳根因not_evaluated。

源码机制：run_place_from_carry在_home_arm成功后才调用_dump_place。当前异常抛出之前，release_kinematic/verify等完整记录还在内存；pack_eval_driver外层捕获并写DRIVER_EXC_ContractError，但record=None，所以只有fallback的box/slot/placement等文件，没有trial.json与原有joint-ring/replay manifest。新增完整释放证据也因此没有耐久落盘。这是本次证据明确的缺口，不应宣称已补齐该异常路径或可精确回放释放判定/租约事件。下次同门试验前应先加异常路径的耐久落盘及identity心跳窗口，不应盲目重跑。

## 保存与覆盖

- live_01/metrics.json、exception_boundary.json明确capture_complete=false、replay_possible=false及缺项。
- 原始位姿poses.raw达到8MiB上限，离线解出827个目标样本；末尾1个partial-block拒绝对应字节截断。raw不等于检查使用的精确缓存样本。
- contacts.raw为空，contact_absence_proven=false；不作没有碰撞的证明。
- plugin recorder关闭原因为CTL_STOP；queue drop为0、faults为空；write produced300996/written300995，存在环形覆盖计数，不作全部内部记录完整声明。
- suite、ledger、fallback box/slot/commit、selection CDR、原始事件/字节索引、URDF/config、launch/driver日志、插件bin完整保留；大bin不入Git但记录hash，并复制到主工作区。
- teardown.log记录只停止owned process group765956；最终审计无Gazebo/parameter_bridge残留。
