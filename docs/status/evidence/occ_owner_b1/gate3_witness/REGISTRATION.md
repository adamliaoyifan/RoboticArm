# Gate-3 n=2 功能见证 · 预登记（执行前落盘）

- **登记时间**: 2026-10-16（用户当轮明示授权"现在授权执行"）
- **代码冻结**: HEAD `4bee002351a52283ccb5483bac49a8859cf8338b`
  （分支 agent/eng/master-pack-baseline-g3；B1→B3+B4Gate2 全部提交之后，
  唯一追加为本登记所需的 --frame-runtime 运行开关）
- **安装**: 同工作树 colcon 全量重建（6 包 0 失败，/tmp/freeze_build.log）
- **运行命令**: scripts/pack_to_full_run.sh --out <本目录>/run01
  --max-boxes 2 --sequence-ids carryon,carryon
- **链路**: v2 typed（--frame-runtime）；B3 旗标生产默认开
  （--verify-selected-route、--await-map-confirmed）
- **通过判据（全部满足才算链路通过；只证一次流程，不签稳定性）**:
  1. 两箱 ledger committed=true，fail_code 空；
  2. 每箱含放置、退出（retreat/unloaded 段执行）与 verify 通过；
  3. 箱 1 提交后地图确认（occupancy_confirmed）到达后箱 2 才开始规划
     （B3 await-map-confirmed 门，get_stats/日志可证）；
  4. 箱 2 交接（spawn→detect→compute→select→commit）完整；
  5. v2 commit 带 verify 锚（ledger verify_anchor 有 request_id/delta），
     admission 生效（validate_selected_route 调用记录）。
- **见证额度**: 1 次。失败保留完整证据束离线归因，不重跑、不临时改规则。

## 执行记录

- **尝试 0（中止，不计额度）**: 2026-10-16T14:00 启动即在 launch 段失败——
  `SEMANTIC_VENDOR_MISSING`（worktree install 的 CLIP vendor 为空，冻结内容
  不入 git）。驱动未启动、无运动、无箱生成：见证未发生。
  修复为环境层：`LUGGAGE_CLIP_VENDOR_DIR` 指向主工作区 src vendor
  （semantic_segmenter._setup_clip_vendor 的正统覆盖，代码零改动）。
- **尝试 1（正式见证）**: 同一冻结 HEAD `4bee0022`，附加上述 env，执行同一命令。
- **尝试 0b（中止，同因续）**: 依次揭出三道同类冻结内容门——YOLO 权重、
  pydeps-frozen 前缀（均 gitignored、主工作区持有）。修复同为安装/环境层
  符号链接（install share models/yolov8s-world.pt、ws 根 pydeps-frozen），
  代码零改动。逐门失败均为 launch 即断，无运动、无箱、见证未发生。
- **尝试 0c（中止）**: 栈已起但驱动启动即拒（MISSING_PEER——栈侧心跳未开：
  launch 缺 frame_runtime:=true）。无运动、无箱。修复为运行脚本补
  `frame_runtime:=true`（与驱动 --frame-runtime 同一接线类，代码零改动）。
  冻结定义更新为包含此两处运行开关的 HEAD（见最终哈希）。
- **尝试 0d（中止）**: pack_to_full_run.sh 栈起来后首 goto 即 SCENE_STALE——
  该脚本的 launch 未携带部署匹配的 scene_tf/atlas/robot_poses 三件套
  （G3 计分运行的既有接线）。无运动、无箱。**执行载体改为
  scripts/g3/run.sh score**（score_run01 同源：n=2、inch20、全链路参数、
  worktree install 身份校验内置）——车辆更换，非规则变更。
- **正式见证（尝试 1，计额度）**: `bash scripts/g3/run.sh score gate3_*`，
  冻结代码 = worktree HEAD（含两处运行开关提交），MAIN 侧冻结资产按
  run.sh 既有指向。
- **r3 发现（产品缺陷，修复后重冻结）**: 箱 0 死于 STALE_MAP_REVISION，根因
  是 B1 快照冻结量化把边界列 z 推到 floor 之下 1e-16 → v2 发布自拒
  SURFACE_HEIGHT_OUT_OF_RANGE → planner 收不到 v2 面。修复 7391f460
  （冻结不量化+部署 hull 回归测试）。**冻结 HEAD 更新为 7391f460**；
  r1–r3 均未产生任何箱放置，见证额度仍未消耗。
- **正式见证（r4，计额度）**: 同命令，冻结 7391f460。
- **r4 发现（部署缺口，非产品缺陷）**: st09_contracts 为独立 colcon 包，
  B2 的 PlanningSnapshot epoch 字段改了源码未重建 → 运行时旧拷贝报
  unexpected keyword 'epoch'。已重建（含级联）。冻结仍为源码 HEAD
  7391f460；install 与之同步。
- **r7 发现（B3 缺口，修复 a19760fd）**: place_smoke 缺 _leg_trajectory_digest
  （历史上 _leg_execution_admission 静默降级无 scope；B3 接线显式拒
  LEG_DIGEST_UNAVAILABLE——fail-closed 正确行为暴露了存量缺口）。已按
  motion_occ 原版移植。**冻结 HEAD 更新为 a19760fd**；r5–r7 仍零箱放置，
  额度未耗。
