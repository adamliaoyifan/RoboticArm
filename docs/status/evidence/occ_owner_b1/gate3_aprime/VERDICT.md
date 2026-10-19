# Gate-3（A′ 候选修复）n=2 完整验收 · 判定书（2026-10-19）

## 判定：**PASS**（产品结果门首次通过；40 mm 门原样、合同未改、无现场改规则）

计分运行：`gate3_aprime_r5`（score 模式，max_boxes=2，seed 0，冻结
HEAD `7377959a`，dirty=2 为运行期生成文件）。

### 两箱结果

| 箱 | 链路 | verify XY | verify z | drift | settled | committed | 交接 |
|---|---|---|---|---|---|---|---|
| 0 | select→place→release→verify→占用确认 | **36.5 mm** ≤40 | 1.25 mm | 1.4 mm | ✓ | ✓ commit_index 0 | — |
| 1 | 同上 | **0.0115 mm** ≤40 | 1.25 mm | 0 | ✓ | ✓ commit_index 1 | 规划时地图已含箱 0（committed_box_count=1）|

- `boxes_attempted=2, boxes_packed=2`，`termination=MAX_BOXES`（额度正常
  耗尽），`goto_failed_count=0`，全程 277.9 s wall。
- 对照 r10 见证（同门）：箱 1 verify XY 42.8 mm FAIL → 本轮 36.5/0.01 mm。
- `capacity_claim_valid=false` 属预期（n=2 非容量主张；容量/效率按用户
  指令在验收稳定后再做）。

### 接触合同（首次全程有数据）

- 箱 0：188,868 行——容器（airport_container_real_aprime）188,846 行
  ＋ **S20（机器人）22 行**，后者集中于 sim 102.841–102.861 s（真空关断
  102.439 s 后 0.4 s、持续 20 ms）＝退臂起始工具擦过箱体。按登记条款记
  为**观察项**（已知伙伴、非未知接触、不拦判定）。箱 0 的 36.5 mm 残余
  方向 (+X world) 与该时窗相容，但**因果未证**（n=1，未做单机制归因）。
- 箱 1：64,519 行——全部为容器接触，**释放后零工具接触**，零穿透深度。

### 冻结版本（manifest.json）

- 容器：`airport_container_real_aprime`（dir sha256
  `aa4f11fa…`；src=install 哈希一致，colcon 可部署）。
- scene_tf_used.yaml（container_model 覆盖）md5 `fa2af81a…`；
  robot_poses_used.yaml（identity+逐 pose scene 戳重绑，关节值未动，
  先例同源）sha `699e0767…`。
- 其余同 r10 冻结：atlas_deploy_update / YOLO 权重 / CLIP vendor /
  pydeps-frozen（manifest 全哈希）。

## 前置失败记录（r1–r4，全部保留；判门不曾放宽）

| run | 中止点 | 根因 | 处置 |
|---|---|---|---|
| r1 | driver 前置图洁净审计（boxes_attempted=0） | 接触桥成为第 4 个 parameter_bridge，超过冻结 driver 允许的 3 | 捕获改 gz-echo（tmpfs），不占桥 |
| r2 | goto SCENE_STALE（0 箱） | container_model 覆盖改变 md5(scene_tf)→场景指纹，pose 文件 identity 戳过期 | 派生 pose 文件重绑 identity（关节不动，既有先例） |
| r3 | goto POSE_NOT_VALID（0 箱） | 逐 pose `scene_revision_at_capture` 也要随场景重绑 | 同上，逐 pose 重绑 |
| r4 | 箱 0 释放后 DYNAMIC_BODY："no release pose" | `_gz_box_pose` 的 ign CLI 连续 4 次无位姿返回（输出零数值三元组）；**r5 未复现**（探针 30/30 健康）；代码注释记载 multi_run01 曾一次同类瞬态 | 探针+诊断入库常驻 runner；落地箱按合同保留、地图 pending 持障碍（合同行为正确） |

r5 之前无任何计分数据被消费（r1-r3 零箱，r4 中止即停）；单一计分运行，
无择优重跑。

## 边界

- 已证：冻结版本下两箱全链（放置/退出/几何 verify/占用确认/交接）通过
  原 40 mm 门。
- 未证：箱 0 残余 36.5 mm 与退臂 20 ms 擦碰的因果（单次观察）；容器
  资产三缺陷（缺洞/碎片/漏水）各自贡献（单缺陷臂已按收缩指令撤销）。
- 部署：A′ 经 colcon 安装路径运行于本验收；生产工作区部署为后续
  merge 动作（本 worktree 已版本化全部资产与 runner）。
