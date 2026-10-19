# Gate-3（A′ 修复候选）n=2 完整验收 · 预登记（2026-10-19）

## 目标与不变项

产品结果门（与 r10 见证同一条，不改规则）：同一冻结版本下**两箱**依次完成
放置、退出、几何 verify、占用确认，并正确交接下一箱。**40 mm XY verify
门原样**；规划预算（goto 60 s / plan 180 s / auth 8 次 8 s）原样；占用
合同（B1-B3 栈）原样。失败保存证据，不现场改规则，不自动补跑。

## 冻结版本

- 候选修复：`airport_container_real_aprime`（A′=冻结 YAML 基准的 148 三角
  水密网格；预门 v3 对基准双向 0.000 mm；sha256 见 manifest
  container_model_dir_sha256 与 asset_record）。
- 部署方式：`G3_CONTAINER_MODEL=airport_container_real_aprime`，runner
  生成 `scene_tf_used.yaml`（launch 与 driver 共用）——gz 物理网格、规划
  腔体、占用栅格、TF 同一 YAML 单源。
- runner：`scripts/g3/run.sh`（本提交 HEAD；boot 纪律 v2=孤儿参与者清扫
  ＋SHM 清锁；接触捕获=bridge+记录器全程跟随 driver 窗口）。
- 其余冻结资产同 r10：atlas_deploy_update、robot_poses.baseline_update、
  YOLO 权重、CLIP vendor、pydeps-frozen（manifest 全哈希）。

## 判据（预登记）

1. 两箱 driver 全链 COMPlit：select→place→release→verify→占用确认→交接。
2. 每箱几何 verify XY ≤ 40 mm（z 门同 r10）。
3. 接触证据（首次具备）：box_contacts.jsonl 伙伴集 ∈ {container,
   真空/夹具}（搬运期工具接触合法；**释放后**不应再有工具伙伴）；穿透
   深度记录留档，异常（深度突增/未知伙伴）记 invalid 观察不拦判定。
4. 占用确认：每箱 PLACED 后 cargo map 确认态翻转、账本一致（B2/B3 门）。

## 支撑证据（已完成，非本门一部分）

- A′ 集成检查 aprime_integration_01：0.013 mm 残余 n=2/2 逐位复现、
  落地即静止、释放后零工具接触、48k 接触点零穿透。
- 三臂 round-1（已收缩为支撑证据）：A0 23.005mm vs A′ 0.013mm。

## 执行

`G3_CONTAINER_MODEL=airport_container_real_aprime bash scripts/g3/run.sh score gate3_aprime_r1`
