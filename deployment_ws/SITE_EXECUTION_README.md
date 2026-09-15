# 实机执行文档：`master` vs 当前 deployment（`ros2_humble`）

- 编写日期：2026-09-15
- 现场机：ThinkPad，无 NVIDIA GPU，仅 `/opt/ros/jazzy`
- 当前工作树：`ros2_humble` @ `3b83de2`，路径 `/home/adamliao/work/RoboticArm`
- 对照目标：`origin/master` @ `8ac1fca`（无 GPU 密封抓取 SOP 所在分支）
- 本轮验收上限：**密封 pick（observe → detect → plan → attach → DI0 密封 → retreat）**
- 本轮明确不做：place、packing、cargo map、生产 orchestrator、完整 HB-1/2/3 重跑、Livox 进检测链路
- 当前先测 master preprocessor A/B：数字门槛（帧率 / YOLO 2 Hz / RSS）见 [`docs/PREPROCESSOR_AB_ACCEPT.md`](docs/PREPROCESSOR_AB_ACCEPT.md)（与 master 树 `docs/status/site_preprocessor_ab_accept.md` 同步）
- **进度快照（2026-09-15 19:57）：** 已做到 Gate 4 平面接触密封；**Gate 5 未跑**。正文见下节。证据：[`docs/status/evidence/site_no_gpu_verify/PROGRESS.md`](docs/status/evidence/site_no_gpu_verify/PROGRESS.md)

---

## 当前测试进度（2026-09-15 19:57 +08）

**执行树：** master worktree `/home/adamliao/work/RoboticArm-master`（本轮主跑；现场 `ros2_humble` 工作区未当主路径）。
**Domain：** `ROS_DOMAIN_ID=7`。证据根：`docs/status/evidence/site_no_gpu_verify/`。
**本轮上限仍是密封 pick。完整周期 Gate 5 还没开始。**

### 硬约束：真空必须贴无高低差的平面

当前真空（控制柜 box IO：DO0=泵、DO1=吹放、DI0=密封）**只有杯口压在一张没有高低差的平面上才能密封**。这是现场实测，不是建议。

| 接触面 | DI0 | 判定 | 证据 |
|---|---|---|---|
| 空中（杯不贴箱） | 一直 0 | 针脚 PASS；接触密封不算（`VACUUM_SEAL_TIMEOUT`） | `20260915_1948_g4` |
| 贴箱，但盖面**有高低差**（凸起 / 凹陷 / 台阶 / 折痕 / 拉杆区） | 泵开满 8 s 仍为 0 | 接触密封 **FAIL** | `20260915_1952_g4_contact`、`20260915_1954_g4_contact2` |
| 换成**无高低差的平面** | 1.87 s 置 1 | 接触密封 **PASS**（hold 1 s 仍吸；release 0.21 s 掉 0） | `20260915_1956_g4_flat` |

**明确禁止：**

1. 不要在有高低差的箱盖上做 Gate 4 接触密封或 Gate 5 pick。
2. 杯口必须整面落在同一平面，不要压棱、锁扣、缝、拉杆、软包鼓包。
3. 漏气时加长 `seal_timeout_sec` **过不了**：非平面两次 8 s 都是 DI0=0。
4. 平面历史参考约 6.3 s；本次平面实测 **1.87 s**。门限仍是 **≤ 8.0 s**。

Gate 5 的箱子同样必须是**无高低差的平面顶**。换箱或换盖面后先重做一次 `/vacuum/command` enable，DI0 置 1 才允许发 pick 轨迹。

### 闸门状态

| 闸 | 结果 | 证据目录 | 说明 |
|---|---|---|---|
| 0 / 0.5 | PASS | `20260915_1815_g0_g05` | Jazzy、ping 臂/D555/Livox、CPS、YOLO/CLIP 权重。两树工作区仍脏，未按清单拷 `site/unpushed-*` 当干净备份。 |
| 1a D555 | PASS（旁证） | `20260915_1825_g1c_g1d_pp` | 640×360@15，host-stamp。无单独 20 s hz 文件，live 流稳定。 |
| 1b 关节只读 | PASS（旁证） | 同上 + Gate 2/3 | `/joint_states` 非零。无单独 telemetry-only 目录。 |
| 1c Livox overlay | PASS | `20260915_1825_g1c_g1d_pp` | 不进 detect。 |
| 1d detect-only | PASS | `20260915_1905_g1d_yolo_crop`；A 见 `20260915_1936_g3` | **B 3/3**。检测 ROI 跟 YOLO bbox→对齐深度，**不**跟 `scene_tf` pickup 方框（`crop_to_workspace=false`）。早期 FAIL 是误用 workspace 裁剪。 |
| 2  2° FJT | PASS | `20260915_1929_g2` | `--duration 0.4`（默认 3 s 会 20070，低于 Huayan 约 1 deg/s）。现场 observe 关节已记下。 |
| 3 detect+plan-only | PASS（observe CONDITIONAL） | `20260915_1936_g3` | A/B 都 detect 3/3 + plan-only 4 段（含 attach）、**零 FJT**。observe 用 `--skip-observe`（Gate 2 现场位，不是仿真 `pickup_observe`）。 |
| 4 真空 | 针脚 PASS；接触密封 **仅平面 PASS** | `20260915_1948_g4`、`20260915_1956_g4_flat` | 无 11。有高低差的盖面 FAIL，见上表。 |
| **5 密封 pick 5/5** | **未跑** | — | 下一闸。必须无高低差平面顶。 |

### 本轮已改、后续必须保持的行为

- `hardware_pick`：`semantic_device:=cpu`，`workspace_accept_enabled:=false`，`crop_to_workspace:=false`。
- YOLO 选框：最高 conf 的紧凑货物框（`cargo_min_confidence`），不是最大面积。
- CPS 单客户端：`hardware_pick` 的 executor 与独立 `jazzy_real` / `cps_telemetry` 互斥。
- `scene.measured: false`：可视化货柜仍是 yaml 仿真几何，不是实测。

---

## 0. 先读这一节

两套代码**没有共同祖先**，不能 `git pull`、rebase 或把 `master` 直接 checkout 到脏的现场树。

| | `origin/master` | 当前 `ros2_humble`（本机） |
|---|---|---|
| 历史 | 独立仓库形态：根目录 `src/` + `deployment_ws/` | 嵌套：`elfin_humble_ws/src/` + `deployment_ws/` |
| 相对对方独有提交 | 202 | 58 |
| 实机主线 | 无 GPU sealed pick SOP + 锁定 EEF TF + preprocessor A/B | 现场 bring-up、D555/Livox overlay、CPS executor、hardware_pick 初版 |
| 仿真主线 | POS-1 place、PF-R7/R10 评测（本轮不跑） | Humble Gazebo 装箱闭环（本轮不跑） |

**推荐执行策略（不要二选一混用）：**

1. 把当前脏树备份成 `site/unpushed-YYYYMMDD`（或至少保留完整 `git diff`）。
2. 用 **第二 worktree** 检出 `origin/master`，只在那棵树里跑 master 的 `hardware_pick.sh`。
3. 把本机已经量过、必须保住的现场数 **按文件白名单**拷进 master 树；不要整树覆盖。
4. 未提交的 `scene_tf.yaml` 左右镜像 **禁止**直接当实机几何用（见 §3.4）。

若现场暂时不能开第二 worktree，则留在 `ros2_humble` 上跑，但必须先补齐 §4 的阻塞缺口，否则 plan/pick 会在 Jazzy MoveIt、CPU YOLO 或错误的 preprocessor 上失败。缺口未补时，只允许跑到 **Gate 1 的 2° FJT** 和 **`--detect-only`**。

---

## 1. `master` 读后：它实际交付了什么

`master` 把「现场 runtime」和「仿真 packing」分家，现场目标写在 `docs/status/site_no_gpu_verify.md`。

### 1.1 现场闭环（本轮要验）

```text
WAIT (operator on e-stop)
  jazzy_real 独占 CPS TCP
  D555 640x360@15 + host-stamp JPEG/PNG
  preprocessor A 或 B（wall clock，禁止 replay yaml）
  YOLO-World CPU，max_rate_hz=2.0
  DetectLuggage
  BuildMotionSequence(pick)
  PlanMotion: approach → attach → (vacuum DO0, wait DI0) → pick_retreat
  默认保持吸力；--release 或 Ctrl-C 才放
```

入口：

```bash
# master 树（根目录就是 luggage 工作空间）
./deployment_ws/scripts/hardware_pick.sh semantic_device:=cpu
# 另一终端，同一 ROS_DOMAIN_ID=7
ros2 run luggage_planning hardware_pick_driver.py --detect-only
ros2 run luggage_planning hardware_pick_driver.py --plan-only
ros2 run luggage_planning hardware_pick_driver.py
ros2 run luggage_planning hardware_pick_driver.py --release
```

### 1.2 `master` 认为已经在本单元做过（只算 smoke，不算本轮验收）

- D555 PoE `192.168.11.55`，官方 librealsense **2.58.4**（`LD_LIBRARY_PATH=/lib/x86_64-linux-gnu` 优先）
- 稳定流：`640x360@15` 彩色 + aligned depth；**禁止** `896x504@30`
- HB-3 安装残差约 2°（不要重跑完整 HB 计划）
- Mid-360 overlay + pendant bag `record_site_pendant_20260911_220406`
- 真空针脚 2026-09-09 实测：仅 DO0=1 / DO1=0 能密封，DI0 上升约 **6.3 s**；DO0 与 DO1 同时为 1 永不密封
- **2026-09-15 现场：** 接触密封还要求杯口贴**无高低差的平面**；有高低差则 8 s 内 DI0 不置 1。平面盖实测 `sealed in 1.87 s`（`20260915_1956_g4_flat`）
- CC600 手眼锁定：`T_end_optical`（`elfin_end_link ← d555_color_optical_frame`）

### 1.3 `master` 明确从未在本单元验证

- 密封的 `hardware_pick_driver.py` **完整周期**（Gate 5：approach / attach / 真空 / retreat / release × 5）仍未跑
- preprocessor A 的 **运动中** live gate（示教器晃臂时 pair 被闸住）本轮只有袋回放旁证，没有新的 live 运动闸表

已在本单元补上、不再算「从未验证」：静止时 A/B detect-only 与 plan-only；真空针脚；无高低差平面上的接触密封。

### 1.4 `master` 另有、但本轮禁止带上实机的内容

- POS-1 place（ACM settle-retry、链可达性门、P2 堆叠、10 Hz stall sampler）
- PF-R7 G4 / PF-R10 C1–C2 仿真评测门槛
- 生产 orchestrator 的 `WAIT_START` → `EXPLORE_CONTAINER` → pick/place/commit
- Livox 进 preprocessor 的 deskew / 检测融合

这些是仿真或下一阶段实机工作。本轮任何 place 轨迹、任何 `/placement_planner`、任何 `operator_control.py start` 都算出界。

---

## 2. 当前 deployment（`ros2_humble`）实际状态

### 2.1 布局与运行时

| 项 | 当前树 |
|---|---|
| 行李栈 | `elfin_humble_ws/`（Humble 源码，现场用 Jazzy overlay 跑） |
| 实机 executor | `deployment_ws/src/elfin_trajectory_executor` |
| Domain | `ROS_DOMAIN_ID=7` |
| 臂 | `192.168.0.10:10003` |
| D555 | PoE `192.168.11.55`（不是 USB D435） |
| Livox | `192.168.1.120`，host `192.168.1.5`（见 `deployment_ws/config/MID360s_config.json`） |
| GPU | 无 `nvidia-smi`；语义必须 `device:=cpu` |
| CPS | `jazzy_real` 与 `cps_telemetry` **互斥**（单 TCP 客户端） |
| Ctrl+C | 停节点 / 掉 TCP，**不断 48 V、不 BlackOut** |

`deployment_ws/README.md` 里 2026-09-02 的 Gate 0 记录（`enp0s31f6 DOWN`、ping 失败）是旧快照。现场网口后来已用于 D555/Livox bring-up，**每次开跑必须重测 ping**，不要用那份 `site_check_report.yaml` 当现状。

`site_vars.yaml` 仍写着 `scene.measured: false`，几何仍是仿真默认，不能当验收过的现场测量。

### 2.2 当前已经有、且和 master 对齐的部分

- `jazzy_real.launch.py` + `FollowJointTrajectory` 握手（下一步只认 result SUCCEEDED）
- `send_joint_trajectory --delta-deg 2 --and-back` → `READY_FOR_NEXT`
- D555 host-stamp + compressed JPEG/PNG
- `overlay_livox_d555.launch.py`
- `hardware_pick.launch.py` / `hardware_pick_driver.py`（observe → detect → pick 骨架）
- 真空：box DO0=泵、DO1=吹放、DI0=密封；executor 提供 `/elfin/vacuum/set_do{0,1}` 与 `/vacuum/di0`
- `hardware_pick` 里 YOLO `max_rate_hz:=2.0`、`cloud_max_age_sec:=8.0`（防 CPU 导致 `DETECT_STALE_CLOUD`）
- `ompl_planning.yaml` **已经是** Jazzy 的 string-array 插件格式（master 则额外拆了 `ompl_planning.jazzy.yaml` overlay）

### 2.3 当前工作区脏文件（开跑前必须处理）

```
M  elfin_humble_ws/src/luggage_description/config/scene_tf.yaml
?? elfin_humble_ws/src/luggage_description/config/backups/20260911_2127_livox_d555_icp/
?? elfin_humble_ws/src/luggage_description/config/backups/20260911_2148_livox_mount_z/
?? elfin_humble_ws/src/luggage_description/config/backups/20260911_2151_livox_mount_yz/
?? elfin_humble_ws/src/luggage_description/config/backups/20260911_2153_livox_mount_yz/
```

备份目录可以保留，不要当运行时配置。`scene_tf.yaml` 的未提交改动见 §3.4。

---

## 3. 逐项差异（只列会影响实机对错的）

### 3.1 工作空间形状

| | master | 当前 |
|---|---|---|
| luggage 包 | `/src/luggage_*` | `/elfin_humble_ws/src/luggage_*` |
| `hardware_pick.sh` 找 Humble overlay | 根 `install/`，否则回退 `elfin_humble_ws/install/` | **只认** `elfin_humble_ws/install/` |
| 第二 worktree 命令 | `colcon build` 在仓库根 | 必须 `cd elfin_humble_ws && colcon build` |

现场脚本路径不能照抄 master SOP 的 `/home/adamliao/work/RoboticArm-master/src/...`，除非真的在 master worktree 里。

### 3.2 Preprocessor A/B（高优先级缺陷）

master 合同：

| Profile | 文件 | motion gate | RGB-D pair 容差 | 用途 |
|---|---|---|---|---|
| A | `preprocessor_d555_live.yaml` | **on**（0.02 rad/s，settle 0.5 s） | **5 ms** | master 严格现场 |
| B | `preprocessor_d555_site.yaml` | off | 50 ms | 当前 humble 现场阈值 |
| 禁止 | `preprocessor_d555_replay.yaml` | — | — | `use_sim_time: true`，只给 bag replay |

当前树：

- **没有** `preprocessor_d555_site.yaml`
- `elfin_humble_ws/src/luggage_perception/config/preprocessor_d555_live.yaml` 的内容其实是 **B**（gate off，pair 50 ms）
- `hardware_pick.launch.py` **写死**这份 live yaml，没有 `preprocessor_config:=` 参数
- `hardware_pick.sh` **不会**在没有 GPU 时自动注入 `semantic_device:=cpu`

结论：按文件名跑「A」会得到 B；master 要求的 A 在当前树里不存在。CPU 主机若忘记显式传 `semantic_device:=cpu`，segmenter 会按 launch 默认 `cuda` 起不来。

### 3.3 末端 TF：这是两套几何，不是调参差

同一条链 `elfin_end_link → suction_panel → eef_mount_adapter → {camera_link, mid360_mount_frame}`。法兰 CAD（`suction_flange_*`）两边锁定一致。后面三节不一致。

| Joint | 当前 `ros2_humble` | `master` 锁定（2026-09-14） |
|---|---|---|
| `suction_panel → eef_mount_adapter` | `0.0168 -0.0156 0.0702` / GUI | `0.021734 -0.033926 0.082591`（为闭合 Livox ICP 反解） |
| `eef_mount_adapter → camera_link` | CC600 Layer 3 原表达 `-0.028833 0.107910 -0.077120` | **改写后的** Layer 3 `-0.023249 0.099580 -0.052059`（`T_end_optical` 不变） |
| `eef_mount_adapter → mid360_mount` | **把桌面 ICP 写进 mount**：`0.010 0.130 0.015` + 非 CAD rpy | **CAD 方槽** `0.022 0.103 0.038` / `0, π/2, π/2` |
| D555 壳体碰撞/可视 | 仍是 D435 `90×25×25 mm` | D555 datasheet `167×42×48 mm` |

master 硬规则（`docs/architecture/eef_sensor_frames.md`）：

- 禁止把桌面 ICP 写进 `mid360_mount_xyz/rpy`
- 禁止改 `cam_mount_*`，除非重新解 ChArUco 再重推 Layer 3
- 禁止用 D435 壳体当腕部相机
- `camera_link` 与 `d555_link` 必须 identity

当前树 2026-09-11 的 Livox 备份（`backups/20260911_*`）正是「把 ICP 写进 mount」这条被 master 否定的路径。检测/抓取主链路只用 D555，Livox 本轮不进 detect；但 MoveIt 碰撞体会用错误的 D435 盒子，接近行李箱时可能误碰或漏碰。

**实机 pick 对相机外参的要求：** 用 CC600 的 `T_end_optical`，不要用 09-11 写进 `mid360_origin.xacro` 的 ICP。若在当前树跑 detect，Layer 3 仍是 09-11 原表达，光学外参可用；不要为了对齐 Livox 去改 `cam_mount_*`。

### 3.4 未提交的 `scene_tf.yaml` 左右镜像（阻塞级风险）

HEAD（与 master 仿真默认同类）：

- `world → container_link`：`[1.5, 0, 0]`，yaw 0
- pickup platform：`[-1.0, 0, 0]`

工作区未提交：

- container：`[-1.5, 0, 0]`，yaw `π`
- pickup platform：`[+1.0, 0, 0]`

`pickup_observe` 关节是按相机在 **world `(-1.0, 0.0, 1.9)`**、朝下看旧平台解的。镜像后平台在 +X，observe 会看空地。

**本轮规则：** 实机 `hardware_pick` 不要用这份未提交镜像，除非现场卷尺确认平台确实在机器人 +X，并重新解 `pickup_observe`。仿真左右对调与实机抓取验收分开做。

`site_vars.yaml` 的 `scene.measured` 仍是 false。容器/平台/开口的仿真数不能写进验收报告当「已标定现场」。

### 3.5 MoveIt / pick 图

| | master | 当前 |
|---|---|---|
| Jazzy OMPL overlay | launch 里强制叠 `ompl_planning.jazzy.yaml`，并剥掉 Humble 标量 `planning_plugin` | 主文件已是 Jazzy 数组；launch **没有** overlay，依赖 `MoveItConfigsBuilder.to_dict()` 不把标量塞回去 |
| `preprocessor_config` 参数 | 有，可切 A/B | 无 |
| `start_aligned_depth_cloud` | 无（canonical 路径不传 PointCloud2） | 有，默认 false |
| `hardware_pick.sh` 无 GPU 默认 cpu | 有 | 无 |
| driver 本体 | 与当前几乎相同（~380 行） | 可用 |

当前 `ompl_planning.yaml` 已是 Jazzy 格式，但 `MoveItConfigsBuilder` 仍可能注入标量。若 `move_group` 报 `ParameterTypeException: expected [string_array] got [string]`，按 master 方式 overlay，不要改 Humble 仿真 yaml 去迁就。

### 3.6 生产编排 vs 现场 driver

当前 `luggage_bringup` 已有 ROS 2 生产合同：

```text
WAIT_START → RESET_CARGO_MAP → EXPLORE_CONTAINER → RETURN_PICK_OBSERVE
  → WAIT_PICKUP_READY → DETECT → COMPUTE_PLACEMENT
  → PLAN_PICK → EXEC_PICK → PLAN_PLACE → EXEC_PLACE
  → COMMIT_AND_VERIFY → …
```

启动后停在 `WAIT_START`，必须 `operator_control.py start` 才许运动。这是 **装箱单元** 合同，不是本轮验收对象。

本轮只跑 `hardware_pick_driver.py`。不要 launch `production_orchestrator.launch.py`。

---

## 4. 开跑前必须补齐的代码/配置缺口

在 **master worktree** 跑时，这些多数已经在 master 里。在 **当前 `ros2_humble` 树**跑时，按序补：

| ID | 缺口 | 不做的后果 | 本轮是否阻塞 |
|---|---|---|---|
| C1 | `hardware_pick.sh` 无 GPU 时默认 `semantic_device:=cpu` | segmenter 要 CUDA，detect 起不来 | 阻塞 |
| C2 | launch 增加 `preprocessor_config`；补 `preprocessor_d555_site.yaml`（现 live 内容即 B）；把真正的 A（gate on，5 ms）写回 `preprocessor_d555_live.yaml` | 无法做 A/B 对比；误把 B 当 A 签字 | 做 A 验收时阻塞；只做 B 可暂缓 |
| C3 | 确认 `move_group` 不被 Builder 注入标量 OMPL 参数 | `--plan-only` / pick 无规划器 | 一出现就阻塞 |
| C4 | 不要用未提交镜像 `scene_tf.yaml` | observe 看错台 | 阻塞 |
| C5 | D555 壳体改 167×42×48（master URDF） | 近箱碰撞体错误 | pick 接触前建议修；detect-only 不阻塞 |
| C6 | 不要把 09-11 Livox ICP 写进 `mid360_origin.xacro` | 违反锁定 TF；污染 RViz | 本轮 Livox 不进 detect，不阻塞 pick；禁止再改 camera Layer 3 |
| C7 | `site_vars.yaml` 填实测 IP/几何；Gate 0 重跑 | 用过期 DOWN 报告放行 | 运动前阻塞 |

现场量过、需要带进 master worktree 的白名单：

```text
deployment_ws/config/site_vars.yaml
deployment_ws/config/MID360s_config.json          # Livox 192.168.1.120 / host 192.168.1.5
elfin_humble_ws/.../handeye_cc600_20260911_21-38.json
# 若仍在 humble 树跑 detect：保留 09-11 camera_mount_origin.xacro（Layer 3 原表达）
# 不要拷贝 09-11 的 mid360_origin.xacro ICP 版到 master
```

不要拷贝：未提交镜像 `scene_tf.yaml`、ICP backups、bag、`*.pt`、密钥。

---

## 5. 硬性现场规则（全程）

1. 只 `source /opt/ros/jazzy/setup.bash`。同一 shell 禁止 Humble。
2. `export ROS_DOMAIN_ID=7`；`unset ROS_LOCALHOST_ONLY`。
3. 无 Gazebo、无 `zero_joint_state_publisher`、无 EtherCAT。
4. CPS 单客户端：`jazzy_real` XOR `cps_telemetry`。
5. D555 只用 `640,360,15`。
6. 真空禁止 DO0=1 且 DO1=1。
7. 语义：`semantic_device:=cpu`。YOLO `max_rate_hz:=2.0`。
8. 禁止把 `preprocessor_d555_replay.yaml` 喂给 live pick。
9. 失败即停：吸住则先 `--release` 或 Ctrl-C；不自动重试进工作空间。
10. 有人盯急停。Ctrl+C 不等于断电。

网络（本单元历史实测，仍要当场 ping）：

| 设备 | 地址 |
|---|---|
| 臂 CPS | `192.168.0.10:10003` |
| D555 PoE | `192.168.11.55` |
| Livox Mid-360S | `192.168.1.120` |
| Livox host | `192.168.1.5` |
| 臂网口（历史） | `enp0s31f6`，MTU 9000 |

---

## 6. 实机 TODO（按闸门，可打勾）

每闸失败就停，把 §7 的证据打包装回，不要跳闸。命令以 **当前嵌套树** 为默认；若已切 master worktree，把 `HUMBLE_WS` 换成仓库根，`src` 前缀去掉 `elfin_humble_ws/`。

约定环境（每个新终端都做）：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
export PYTHONPATH=/home/adamliao/work/RoboticArm/third_party/huayan_python_sdk:${PYTHONPATH}
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

REPO=/home/adamliao/work/RoboticArm
HUMBLE_WS=$REPO/elfin_humble_ws
source "$HUMBLE_WS/install/setup.bash"
source "$REPO/deployment_ws/install/setup.bash"
```

证据根目录：

```text
docs/status/evidence/site_no_gpu_verify/<RUN_ID>/
```

当前 `ros2_humble` 可能不跟踪 `docs/status/`。现场仍按这个相对路径写；不要写进 `docs/agents/`。

`RUN_ID` 格式：`YYYYMMDD_HHMM_<gate>_<profile>`，例如 `20260915_1400_g1_fjt`。

---

### Gate 0 — 代码与备份（禁止使能伺服）

- [ ] `git rev-parse --abbrev-ref HEAD` 与 `git rev-parse HEAD` 记入 `git.txt`
- [ ] `git status -sb`、`git log -15 --oneline`、`git diff --stat`、`git stash list`
- [ ] 未提交工作进 `site/unpushed-$(date +%Y%m%d)` 或把完整 diff 归档（不要 add bags / weights / secrets）
- [ ] 决定执行树：`master` 第二 worktree **或** 当前树 + §4 补丁
- [ ] 若用 master worktree：

```bash
git fetch origin
git worktree add /home/adamliao/work/RoboticArm-master origin/master
# 在 master 树：
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  luggage_msgs luggage_description luggage_perception luggage_planning \
  elfin_description elfin_moveit_config
cd deployment_ws
colcon build --packages-select elfin_trajectory_executor
```

- [ ] 若留在当前树：补 C1（cpu 默认）后再 build `luggage_planning` + `elfin_trajectory_executor`
- [ ] `python3 -c "import moveit_msgs"`；失败则安装 SOP 中的 `ros-jazzy-moveit-*`（`--detect-only` 可暂缓）

**通过：** 旧树干净或已备份；选定树的 overlay 能 source；未混 Humble。

---

### Gate 0.5 — 网络与资产（仍禁止运动）

```bash
cd "$REPO/deployment_ws"
python3 scripts/check_site.py          # 必须 ICMP 通 192.168.0.10
python3 scripts/check_gate1.py         # CPS import + TCP 10003
python3 scripts/check_gate2.py         # Livox 软件栈（pick 不阻塞）
python3 scripts/check_gate3.py         # YOLO/CLIP 文件
```

- [ ] `enp0s31f6`（或当前臂网口）UP，MTU 确认
- [ ] ping `192.168.0.10` 成功
- [ ] `yolov8s-world.pt` 与 CLIP `ViT-B-32.pt` 在 `luggage_perception`（CLIP ≥ 300 MB）
- [ ] **不要**用 `realsense_d435.launch.py` 当腕部相机；腕部是 D555 PoE

**通过：** Gate 0 ping 绿；CPS import ok。Livox / YOLO 失败只降级后续可选步骤，不挡 Gate 1。

---

### Gate 1a — D555 流（executor 不要占 CPS）

```bash
ros2 launch elfin_trajectory_executor d555_rgbd.launch.py \
  color_profile:=640,360,15 depth_profile:=640,360,15
```

另开终端：

```bash
ros2 topic hz /camera/d555/color/image_raw/compressed
ros2 topic hz /camera/d555/aligned_depth_to_color/image_raw/compressed
ros2 topic echo /camera/d555/aligned_depth_to_color/camera_info --once
```

- [ ] 分辨率 640×360，约 15 Hz
- [ ] `frame_id` 为 `d555_color_optical_frame`（aligned depth / 其 camera_info）
- [ ] 停掉该 launch，再进入下一互斥驱动

**通过：** 两路 compressed ≥ 10 Hz 持续 20 s，无 DDS 掉设备。

---

### Gate 1b — 关节只读（仍不要 `jazzy_real`）

示教器停在安全位，最好已接近 observe。

```bash
ros2 launch elfin_trajectory_executor cps_telemetry.launch.py
ros2 topic echo /joint_states --once
```

- [ ] 六个 `elfin_joint*`，数值非全零
- [ ] `/vacuum/di0` 能读到（0 或 1 均可，此时应无负载）
- [ ] **杀掉 telemetry**，否则 Gate 2 的 `jazzy_real` 连不上

**通过：** 一帧非零 `/joint_states` 入库。

---

### Gate 1c — 可选 Livox overlay（不挡 pick）

```bash
source "$REPO/deployment_ws/livox_ws/env.sh"
ros2 launch elfin_trajectory_executor overlay_livox_d555.launch.py
```

- [ ] `/livox/lidar` 有云
- [ ] RViz 里 Livox 与 D555 相对腕部大致同侧；**不要**现场改 `cam_mount_*` 去「坐实」壳体

失败则记 overlay skip，继续。

---

### Gate 1d — Detect only，臂不动

CPS 必须空闲。`start_executor:=false`。箱子放在 **当前 observe 能看到的** 平台上。若臂不在 observe，先用示教器挪到俯视平台，再用 `--skip-observe`。

当前树（补齐 C1 后）：

```bash
cd "$REPO/deployment_ws"
./scripts/hardware_pick.sh start_executor:=false semantic_device:=cpu
```

第二终端：

```bash
ros2 topic echo /luggage/preprocessed/status --once
ros2 run luggage_planning hardware_pick_driver.py --detect-only --skip-observe
```

然后（仅当 A 文件已按 master 合同存在时）再启一次 A。当前树若只有 B，本闸只签 B。

- [ ] `/luggage/preprocessed/status` 有 accepted pair，无持续 `DETECT_STALE_CLOUD`
- [ ] `DetectLuggage` success，打出 `id / xyz / size / top_z / valid / src`
- [ ] `top_surface_valid=true` 才允许后面运动；否则停

**通过：** B 上 detect 成功 ≥ 3/3（同一只箱子，不挪）。A 若可跑，同样 3/3；A 失败不自动改阈值，先查 motion gate 是否因示教器微抖一直开着。

---

### Gate 2 — 小运动（必须有人盯急停）

只允许一个 CPS：`jazzy_real`。

```bash
cd "$REPO/deployment_ws"
ros2 launch elfin_trajectory_executor jazzy_real.launch.py
```

另一终端：

```bash
ros2 run elfin_trajectory_executor send_joint_trajectory --delta-deg 2 --and-back
```

- [ ] 日志出现 `READY_FOR_NEXT`
- [ ] 臂回到起点，无伺服报错
- [ ] **停掉这个独立 executor**（后面 `hardware_pick.sh` 会自己起 `jazzy_real`）

**通过：** 一次 2° 往返 SUCCEEDED。这是本单元第一条允许的程序运动。

---

### Gate 3 — Observe + detect + plan-only（有运动，无真空）

```bash
cd "$REPO/deployment_ws"
./scripts/hardware_pick.sh semantic_device:=cpu
```

```bash
ros2 run luggage_planning hardware_pick_driver.py --detect-only
ros2 run luggage_planning hardware_pick_driver.py --plan-only
```

- [ ] `GoToRobotPose(pickup_observe)` SUCCEEDED（除非书面记录为何 `--skip-observe`）
- [ ] detect 仍 `top_surface_valid=true`
- [ ] `--plan-only` 打印 pick 段名（预期 `approach` / `attach` / `pick_retreat` 一类）及目标 xyz
- [ ] **没有**发出 `PlanMotion` 执行（plan-only 必须零 FJT 段运动）
- [ ] 若用了镜像 `scene_tf` 导致 observe 看空：立即停，回退 HEAD 的 `scene_tf.yaml`，重解或示教 observe

A/B 各做一轮 plan-only（能提供 A 时）。

**通过：** detect-only 与 plan-only 退出码 0；plan-only 日志可证明未执行段轨迹。

---

### Gate 4 — 真空 IO（接触后，仍不抓完整周期也可先做）

`jazzy_real` 必须是 CPS 主人。杯口离开人或障碍。

**接触密封的盖面必须是无高低差的平面。** 有凸起/凹陷/台阶/折痕/拉杆则漏气，DI0 不会在 8 s 内置 1（已 FAIL 两次）。空中短测只验针脚，不验密封。

在安全接触（或空中短测，接受 DI0 可能不置位）后：

```bash
# 观察
ros2 topic echo /vacuum/io
ros2 topic echo /vacuum/di0
```

通过 `/vacuum/command` 或 driver 的真空路径：enable → 等 DI0 → disable。禁止双手动把 DO0/DO1 同时置 1。

- [x] enable 后 DO0=1、DO1=0（`20260915_1948_g4` 空中 + `20260915_1956_g4_flat` 平面）
- [x] 有密封条件时 DI0 在 **≤ 8.0 s** 升到 1（平面实测 1.87 s；历史约 6.3 s。空中 / **有高低差盖面** 记 `VACUUM_SEAL_TIMEOUT`，不算抓取失败，但**不得**当接触密封 PASS）
- [x] release：DO0=0，DO1 脉冲，DI0 在约 0.21 s 掉 0
- [x] 全程无 DO 11

**通过：** 针脚方向与 2026-09-09 地图一致；无 11 状态；接触密封仅在无高低差平面上签字。

---

### Gate 5 — 密封 pick（本轮终点）

箱子：单件、已知大概尺寸、**顶面是无高低差的平面**、顶面可见、平台稳定、周围无人员。有高低差的盖面不要上 Gate 5（Gate 4 已证明密封失败）。速度保持 launch 默认（FJT `default_velocity_deg:=10`，`max:=20`；named pose `max_vel:=0.25`）。

```bash
ros2 run luggage_planning hardware_pick_driver.py
# 吸住后目视确认再
ros2 run luggage_planning hardware_pick_driver.py --release
```

失败：急停或 Ctrl-C → driver 应 `shutdown_vacuum`；确认 DI0=0、箱子安全。

- [ ] 段 `approach` / `attach` / `pick_retreat` 均 FJT SUCCEEDED
- [ ] attach 后真空 `sealed in T s`，T ≤ 8 s，DI0=1
- [ ] retreat 后箱子仍吸在杯上（目视 + DI0=1）
- [ ] `--release` 后箱子落下或被托住，DI0=0
- [ ] 无碰撞、无掉件、无 DO 11

重复 **5 次**（同一尺寸、同一放置区，允许人工把箱子放回）。

**通过：** 5/5 密封抓取 + 受控释放。任何一次失败整闸 FAIL，不平均掉。

---

### 明确不做的 TODO（写在计划里以免现场加戏）

- [ ] ~~place / insert / descend~~
- [ ] ~~`ComputePlacement` / cargo map integrate~~
- [ ] ~~`operator_control.py start`~~
- [ ] ~~完整 HB-1/2/3、重解 ChArUco~~
- [ ] ~~把 Livox 喂进 detect~~
- [ ] ~~改 `cam_mount_*` 去对齐错误壳体~~
- [ ] ~~在未测几何上跑生产 orchestrator~~

---

## 7. 期望采集的测试数据

每个 Gate 一个目录，至少包含 `meta.json`：

```json
{
  "run_id": "20260915_1400_g5_pick_B",
  "branch": "ros2_humble|master",
  "commit": "<sha>",
  "tree": "/home/adamliao/work/RoboticArm",
  "preprocessor": "A|B",
  "preprocessor_file": "<abs path>",
  "semantic_device": "cpu",
  "scene_tf": "<abs path>",
  "scene_tf_git": "HEAD|dirty-mirror|measured",
  "d555_profile": "640,360,15",
  "operator": "<name>",
  "estop_person": true,
  "result": "PASS|FAIL|SKIP",
  "fail_reason": ""
}
```

### 7.1 日志与 topic（最小集）

| 数据 | 来源 | 用途 |
|---|---|---|
| `git.txt` | Gate 0 | 可复现 |
| `check_site.yaml` | `check_site.py` 新报告 | 不要用 09-02 DOWN 旧文件 |
| `hz_d555.txt` | `ros2 topic hz` 20 s | 流稳定 |
| `joint_states_once.yaml` | echo --once | 非零关节 |
| `preprocessed_status.json` | `/luggage/preprocessed/status` | pair / stale / gate |
| `detect.log` | driver stdout | 尺寸与 `top_surface_valid` |
| `plan_only.log` | driver stdout | 段名与 xyz，证明无执行 |
| `fjt_2deg.log` | `READY_FOR_NEXT` | Gate 2 |
| `vacuum_io.jsonl` | `/vacuum/io` | DO/DI 时序 |
| `pick.log` | 完整 driver | 段成功与 sealed 时间 |
| `tf_eef.txt` | `ros2 run tf2_ros tf2_echo elfin_end_link d555_color_optical_frame` | 与 CC600 对照 |

可选但高价值（磁盘允许时）：

```bash
./scripts/record_site.sh pendant   # 无 FJT 时
./scripts/record_site.sh real      # Gate 5，须 e-stop 人在
```

Bag 至少含：`/joint_states`、`/tf`、`/tf_static`、D555 compressed 两路、`/vacuum/*`、`/luggage/preprocessed/status`、`/luggage/perception/detection/latest`、`/trajectory_executor/events`。不要把完整点云默认进 bag。

### 7.2 必须量化的指标

| 指标 | 符号 | 采集方法 | 期望（本轮） |
|---|---|---|---|
| 彩色帧率 | `f_rgb` | `topic hz` 20 s | 12–16 Hz |
| 对齐深度帧率 | `f_d` | 同上 | 12–16 Hz |
| RGB-D 接受率 | `pair_accept` | preprocessor status 计数 | B：无明显持续拒；A：静止时接受、运动时闸住 |
| 检测成功率 | `detect_ok` | 连续 N 次 driver | Gate 1d：3/3；Gate 5 前每次 pick 都要成功 |
| 顶面有效 | `top_surface_valid` | detect 日志 | 运动前必须 true |
| 位置（平台系或 world） | `xyz` | detect | 记下来，本轮无 GT 门；看是否在台面上 |
| 尺寸 | `w,d,h` | detect | 与卷尺比，记误差；本轮不设毫米门，但 h 应用平台高约束 |
| 2° FJT | `fjt_2deg` | READY_FOR_NEXT | 1/1 |
| observe 到位 | `goto_observe` | GoToRobotPose | 1/1 或书面 skip |
| 规划段数 | `n_seg` | plan-only | ≥ 3，含 attach |
| 密封时间 | `t_seal` | vacuum 日志 / DI0 上升 | 接触密封 ≤ 8.0 s，历史中心约 6.3 s |
| 释放时间 | `t_release` | DI0 下降 | 通常 < 0.2 s（有吹放） |
| 密封抓取 | `pick_sealed` | 5 trial | **5/5** |
| 掉件 | `drop` | 目视 | 0 |
| 碰撞 | `collision` | 目视 + 伺服 | 0 |
| CPU 过期云 | `DETECT_STALE_CLOUD` | detect 日志 | 非持续；偶发重试可接受（driver 默认 5 次） |

仿真里 Phase 6a 的尺寸门（高 p95 0.29 mm 等）**不是**本轮实机门。实机只要求：顶面有效、箱子在视野内、误差记入报告，为下一轮定门提供样本。

### 7.3 建议的表格模板（Gate 5）

| trial | preprocessor | detect xyz | size w×d×h | top_valid | t_seal_s | DI0 after retreat | release ok | notes |
|---|---|---|---|---|---|---|---|---|
| 1 | B | | | | | | | |
| 2 | B | | | | | | | |
| 3 | B | | | | | | | |
| 4 | B | | | | | | | |
| 5 | B | | | | | | | |

A 若跑，另表 5 行。A 与 B 不要混在同一张表里平均。

---

## 8. 验收流程与标准

### 8.1 流程

```text
Gate 0 代码备份            ← 2026-09-15 PASS（备份手续未做干净）
  → Gate 0.5 网络          ← PASS
  → Gate 1a D555           ← PASS（旁证）
  → Gate 1b 关节只读       ← PASS（旁证）
  → Gate 1d detect-only    ← PASS（B+A；YOLO bbox 裁剪）
  → Gate 2 2° FJT          ← PASS（第一条程序运动）
  → Gate 3 observe + plan-only  ← PASS（--skip-observe 书面）
  → Gate 4 真空针脚 + 平面接触密封  ← PASS（有高低差盖面禁止）
  → Gate 5 密封 pick 5/5   ← **未跑**（必须无高低差平面顶）
  → 出报告，停止
```

评审只看证据目录 + 上表，不看口头「好像行」。

### 8.2 判定

| 等级 | 含义 | 条件 |
|---|---|---|
| **FAIL** | 本轮未通过 | 任一阻塞闸失败；或 Gate 5 < 5/5；或发生碰撞/掉件/DO11 |
| **CONDITIONAL** | 有限通过 | Gate 5 = 5/5，但只有 B、没有 A；或 observe 用了 `--skip-observe` 且书面说明示教器已到位；或 Livox overlay skip |
| **PASS** | 完整通过 master 无 GPU SOP | Gate 5 = 5/5 **且** A、B 都能 detect-only 3/3 **且** plan-only 在两 profile 退出 0 **且** 用的是锁定/合法 TF（未用镜像 scene_tf，未改 Layer 3 去坐壳体） |

当前树若无法提供真正的 A，最高只能 **CONDITIONAL**。要 PASS 必须 master worktree 或把 A 文件按合同补回。

### 8.3 单项硬门槛

1. `ROS_DISTRO=jazzy`，未混 Humble。
2. ping `192.168.0.10` 绿之后才允许 FJT。
3. D555 640×360@15，aligned depth 在 `d555_color_optical_frame`。
4. 运动步只认 `/elfin_arm_controller/follow_joint_trajectory` 的 SUCCEEDED，不认 `/trajectory_executor/status=idle`。
5. `top_surface_valid=false` 禁止运动。
6. 真空密封：接触面必须是**无高低差的平面**；此时 `t_seal ≤ 8.0 s` 且 DI0=1（历史 ~6.3 s，2026-09-15 平面 1.87 s）。有高低差 → `SEAL`，不是超时能救的。
7. Gate 5：5/5 密封 + 受控释放，掉件 0，碰撞 0。
8. 证据里能指出 preprocessor 文件的绝对路径和 git sha。

### 8.4 不构成通过的现象

- 「RViz 里箱子看着齐」但没有 detect 日志
- 示教器手动吸住（未走 `hardware_pick_driver`）
- 旧 `site_check_report.yaml`（09-02 NIC DOWN）
- 仿真 Phase 5/6/8 的 passed（那是 Gazebo）
- Livox 与 D555 叠上了但 detect 失败
- 5 次里 4 次成功（本轮不接受多数决）

### 8.5 失败分类（报告里必填）

| 代码 | 含义 | 下一步 |
|---|---|---|
| `NET` | ping/TCP | 查网口 MTU、示教器网络页 |
| `CAM` | 无 640×360 流 | librealsense 2.58.4、禁止 896×504 |
| `STALE` | `DETECT_STALE_CLOUD` | 确认 cpu + 2 Hz；不要加大云超时来掩盖 |
| `DETECT` | 无箱 / 顶面无效 | 光照、FOV、observe 是否看台 |
| `SCENE` | 几何错边 | 未提交镜像 scene_tf / 错误 observe |
| `MOVEIT` | 标量 OMPL / 无 IK | C3 overlay |
| `FJT` | 轨迹被拒或超时 | 速度、急停、CPS 双客户端 |
| `VAC_PIN` | DO 映射错或 11 | 停，对照 09-09 |
| `SEAL` | 接触但不置 DI0 | **先查盖面高低差**；再查杯口是否整面贴合、泄漏、超时 |
| `DROP` | retreat 后掉件 | 停，不要加加速 |
| `COLLIDE` | 碰撞 | 停，查 D435 壳体 vs 真实 D555、ACM |

---

## 9. 本轮之后（不在本次验收，避免遗忘）

完成 CONDITIONAL/PASS 后，下一阶段才允许排期：

1. 卷尺测量 `scene_vars`：基座、平台、容器开口；`scene.measured: true`；重解 `pickup_observe`。
2. 若现场几何确实左右对调，**先**测再改 `scene_tf`，不要用现在这份未提交仿真镜像直接上臂。
3. 把 master 锁定 EEF 树（CAD Livox 槽 + 改写 Layer 3 + D555 壳体）迁到现场树；用同一 CC600 `T_end_optical` 做回归，而不是重跑手眼。
4. 实机尺寸误差门：用卷尺对 Gate 5 的 `w,d,h` 定 p95，再决定是否沿用仿真 20 mm commit margin。
5. 单次 place smoke（POS-1 的硬件裁剪版）：低速、预计算槽、禁止自动堆叠。
6. 生产 orchestrator：`WAIT_START` + request-id 的 `pickup-ready`，仍禁止启动即动。

仿真侧 POS-1 / PF-R7 与本单元 sealed pick **解耦**；不要用仿真 BIN_FULL 当实机通过。

---

## 10. 命令速查（当前嵌套树）

```bash
# Gate 0.5
cd /home/adamliao/work/RoboticArm/deployment_ws
python3 scripts/check_site.py
python3 scripts/check_gate1.py

# Gate 1a
ros2 launch elfin_trajectory_executor d555_rgbd.launch.py

# Gate 1b（与 jazzy_real 互斥）
ros2 launch elfin_trajectory_executor cps_telemetry.launch.py

# Gate 2
ros2 launch elfin_trajectory_executor jazzy_real.launch.py
ros2 run elfin_trajectory_executor send_joint_trajectory --delta-deg 2 --and-back

# Gate 3–5（停掉独立 jazzy_real 之后）
./scripts/hardware_pick.sh semantic_device:=cpu
ros2 run luggage_planning hardware_pick_driver.py --detect-only
ros2 run luggage_planning hardware_pick_driver.py --plan-only
ros2 run luggage_planning hardware_pick_driver.py
ros2 run luggage_planning hardware_pick_driver.py --release
```

Master worktree 把 `hardware_pick.sh` 与 `colcon` 放在仓库根；并可用：

```bash
./scripts/hardware_pick.sh preprocessor_config:=<abs>/preprocessor_d555_site.yaml semantic_device:=cpu
```
