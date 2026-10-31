# 2026-10-31 -- OctoMap未提交接线与placement消费接口复核

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

实现树/tmp/elfin-occ-octomap-repair-2cm-g1，HEAD=4b71073d850db16194d8c4fa9dd94c7ca6d1f2c3，3个测试/执行器文件未提交。本次保存working_changes.patch。placement消费端读取主工作区当前代码（共享工作区也有其他未提交改动），不是一次冻结版本认证。

结论：OctoMap具备低层读取原语，不满足现有placement直接替换使用条件。topic短接线还有确定的客户端状态判定缺陷；live正式分支仍只输出标签并返回。S3保持关闭，7537b033既有O9保留。本轮不接规划、不启动ROS/Gazebo、不改owner工作区，不执行正式样本。

## Verification

加载/opt/ros/humble/setup.bash及/tmp/s2-install/setup.bash后，三个离线套件17 passed/1 deselected。独立status_probe从当前源码AST抽取smoke_ros.Adapter，注入真实接口形状的COMPLETED/receipt.INTEGRATED，在虚拟时钟下执行ros_query与ros_collect：输出unknown、receipt_received=false、parseable=false。只证明客户端误判，不证明历史点云已积分或没积分。

## Current wiring findings

1. GetObservationStatus返回账本状态COMPLETED，内层receipt.status才为INTEGRATED。smoke_ros却轮询顶层INTEGRATED，并据此判成功。新增等待observation_id不能修复这个错误。应同时校验完整(epoch,sensor_id,observation_id)与原始终态；REJECTED/EXPIRED/NOT_SEEN分别保存，不补成功。
2. session.query实际是GetObservationStatus，不是GetSnapshot/WaitSnapshot。将它的输出标op=immediate并不能证明快照查询成功。必须另做等待目标输入→取快照→ReadSurface/ReadCells→ReleaseSnapshot，记录身份、年龄和内容。
3. 查询找回终态不等于收到topic回执；目前receipt_received由query_status派生，不能用于正式接收率。需分别记录订阅回执与查询终态。当前重复发布同一stamp十次是接线手段，不是正式每ID一次发送；先等订阅发现/TF就绪，再独立发送并保留边界事件。硬编码stamp=1秒不适用于当前ROS时钟的新鲜度验证。
4. LiveCampaign目前仅字符串标签，stage开放分支无实例化/调用，返回official_long_window_not_started和exit2。认可避免启动正式样本，但不能称完整live入口已交付。
5. cumulative profile helper已有同进程多帧路径；_run_profile仍调用profile_frame，未使用累计helper。短接线与正式调度路径尚未统一。
6. OfflineAdapter.ros_send把传入deadline当记录时刻，_run_ros先提交全部发送再查询；smoke适配器直接丢弃deadline。0/0.5证明计划时刻记录不受虚拟查询advance影响，尚不能证明真实独立发送按时完成。需记录planned与actual两套时间，并行为验证真正发送调度。

## Placement usage and gaps

本次“placement policy”按架构规定的现有Humble placement_planner_node→placement_solver及运动占用读取链审查；不是ROS1离线FreeSpaceModel，也未发现消费端直接使用uld_occ_observer服务。

| 使用要求 | 当前接口/调用 | 结论与补项 |
|---|---|---|
| 获取目标观测之后的一致读视图 | WaitSnapshot、GetSnapshot、identity含epoch/map_revision/observed_revision/hash/config/clock | 基础能力具备；先正确闭合终态，再验证等待返回年龄与observed_revision。不能只看map_revision，饱和更新可能不改内容。 |
| 读取整幅支撑面 | ReadSurface分页给x/y、active_count、known_count、has_occupied、top_z | 消费端需要surface_2d或SurfaceContractV2；目前无桥接调用。所有页用同一句柄，核对identity，完整后一次发布，finally释放。TTL/pressure/epoch变化时整次失败，不发布半幅图。 |
| 坐标、形状、高度基准 | top_z为domain origin_z+(z+1)*resolution并裁至ceiling；Surface不自带完整domain元数据 | 配套geometry/config必须按hash/digest验证并提供分辨率、origin、shape、floor/ceiling、frame/scene绑定。不可按默认5cm解释2cm索引。OctoMap输出y*nx+x，planner内部网格第一维x；按显式x/y重建避免转置。 |
| unknown/free/occupied与来源 | Surface仅列计数和最高occupied | has_occupied=false不等于free；只有active_count>0且known_count==active_count且无occupied才可将整有效列判全free。部分未知保留unknown；active_count=0不能当自由空间。unknown的V2顶面为null/来源unknown，free顶面为floor；不能把默认top_z=0作为观测。occupied上方未知/局部空洞若影响安全需同句柄ReadCells，计数不足以给出竖直分布。sensor来源需由合法输入链证明，不伪造verified_geometry。 |
| placement身份接纳 | SurfaceContractV2要求scene/frame/geometry/revision与map-rev-N关联；现planner只接纳严格递增map_revision | OctoMap epoch重置与map_revision重复不能直接映射；需定义reset使旧视图失效、epoch到scene/消费版本的绑定。饱和后observed_revision/年龄变化也不能被仅map_revision去重吞掉。 |
| 算法行为 | solver按高度选支撑、unknown高于floor拒绝；floor prior只代表地板存在 | 保留fail-closed：拒绝/过期/错hash不可回退为空仓。完整候选足迹、间隙、顶高/支撑与轨迹占用读同一身份。禁止把计划放置框写成实测占用。 |
| 性能/资源可用性 | 2cm列数与分页开销、TTL/8句柄、未覆盖分配 | 尚无placement整链延迟样本，S3未跑，资源NOT_EVALUATED；接口有字段不等于目标负载可用。 |

## Next todo and acceptance

先修S2客户端状态与真实适配器，原owner提交后复核；不以本次接口审查擅自扩大为规划接入。

后续独立桥接任务应冻结上述映射与reset合同，再做离线oracle：非零floor、非对称nx/ny、倒角域外、全free/全unknown/部分known/occupied上方unknown、跨页一致性、最后release、TTL/reset/STALE/压力、同map_revision不同observed_revision。输出须通过现有surface_from_message严格校验，支撑候选与同快照ROI一致；拒绝地图不得变成floor-prior成功。

非计分接线通过条件：一个唯一ID有匹配终态；分别记录receipt订阅/状态查询；WaitSnapshot返回OK且目标已包含；同句柄ROI/Surface全页身份相同，oracle一致并释放。之后才能讨论完整runner交付与S3开放。资源门未覆盖分配仍不可pass。

## Pointers

- docs/architecture/placement.md
- src/luggage_packing/scripts/placement_planner_node.py (_on_surface_v2、_surface_view_from_record)
- src/luggage_description/luggage_description/ros_frame_bindings/surfaces.py
- src/luggage_description/luggage_description/frame_contracts.py (validate_surface)
- src/luggage_planning/luggage_planning/occupancy_place_paths.py
- /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/srv/ReadSurface.srv
- /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark/campaign_executor.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/placement_interface_review/
