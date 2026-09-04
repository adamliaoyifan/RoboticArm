# Todo 2 — 检测 vs GT 精度门

对应 plan todo `detect-gt-gate`。这是进入运动规划之前的**门禁**：检测精度不过线，
pick/retreat 的失败无法归因，做了也白做。

## 范围

新增一个无 ROS 的 `DetectionAccuracy` 类，配一套采样流程，在 `pickup_observe`
位姿下把 `DetectLuggage` 的结果和 `GetCurrentBox` 的 GT 比对 N 次，产出报表。

**不做**：改估高算法、改检测器内部、把对比逻辑塞进检测器节点。

## 交付物

| 文件 | 性质 |
|---|---|
| `luggage_perception/luggage_perception/box_geometry.py` | 新，无 ROS：IoU / 长宽比 / yaw 折叠 |
| `luggage_perception/luggage_perception/eval/detection_accuracy.py` | 新，无 ROS |
| `luggage_perception/test/test_box_geometry.py` | 新 |
| `luggage_perception/test/eval/test_detection_accuracy.py` | 新 |
| `docs/status/detection_gt_gate.md` | 新，实测报表 |

采样脚本本身不单独做，并入 todo 4 的评测驱动；本 todo 先把类和判据定死，
必要时用一次性脚本先取一组数据。

---

## 1. 已有的基线：别重复劳动，也别误读

[preprocessor_baseline.md](../status/preprocessor_baseline.md) 里已有一组
`DetectLuggage` vs `GetCurrentBox`，但那是在 launch 默认的 **`observe`** 位姿、
且 `use_semantic:=false`（整片点云跑 RANSAC）下测的：

| | width | depth | height | center xyz |
|---|---|---|---|---|
| GT | 0.729 | 0.459 | 0.288 | (-1.000, 0.000, 1.004) |
| Detect | 0.222 | 0.111 | 0.216 | (-0.683, -0.197, 0.968) |

读法：

- **高度这条链是通的**。另一次采样 height 0.252 vs GT 0.252，说明
  "平台已知 Z + RANSAC 箱顶" 在预处理之后没问题。
- **XY 与 footprint 偏差大是视角伪影**，不是回归。`observe` 不是俯视取货箱的
  位姿，腕部相机看到的是箱子侧面的一角，PCA 拟出来的矩形自然又小又偏。

所以本 todo 的重测**必须**在 `pickup_observe` 且开 `use_semantic` 之后做。
判断"是否变差"时要和上面这组数字区分开，不要拿 `observe` 的数当基准线。

---

## 2. `DetectionAccuracy` 类

无 ROS。驱动把两份 msg 转成普通数值传进来，类不认识 `DetectedLuggage`。

```python
@dataclass(frozen=True)
class BoxObservation:
    x: float; y: float; z: float      # 世界系箱中心，m
    yaw: float                        # rad
    width: float; depth: float; height: float   # m

@dataclass(frozen=True)
class AccuracyResult:
    ok: bool
    err_xy: float          # 水平距离，m
    err_z: float           # 有符号，m
    err_xyz: float         # 3D 距离，m
    err_width: float       # 有符号，已按 swap 对齐后
    err_depth: float
    err_height: float
    err_yaw: float         # rad，已按对称性折叠；近方箱为 0
    swapped: bool          # 是否发生了 W/D 互换匹配
    reason: str            # "ok" / "near_square" / 第一个越界项
    iou: float             # 俯视 footprint IoU，主分数
    near_square: bool      # GT max/min < MIN_ASPECT_FOR_YAW (1.15)
```

```python
class DetectionAccuracy:
    def __init__(self, tol_xy=0.03, tol_z=0.02, tol_size=0.05, tol_yaw=0.15,
                 min_aspect=1.15, tol_iou=0.60): ...
    def compare(self, measured: BoxObservation, gt: BoxObservation) -> AccuracyResult
    def summarize(self, results: list[AccuracyResult]) -> dict   # P50/P95/通过率
```

实现在 `luggage_perception/eval/detection_accuracy.py`，几何原语在
`luggage_perception/box_geometry.py`。检测器**不得** import 本模块；评测驱动只调
`compare`。

### 关键细节：IoU 主分数、yaw 对称性、W/D 互换

主分数是**俯视 footprint IoU**（每个箱子都算，不只近方箱）。180° 翻转和
90°+W/D 互换会得到同一占用，IoU ≈ 1；长箱真的转了 90° 但尺寸没换，IoU 会掉到
~0.33，门禁靠 IoU 拦住，不必单独「找回真 yaw」。

1. **180° 歧义**：主轴方向可正可负，`yaw` 与 `yaw + π` 描述同一个箱子。
   标量 yaw 误差先 wrap 到 `(-π, π]`，再折叠到 `(-π/2, π/2]`（`fold_yaw_pi`）。

2. **90° + W/D 互换**：主轴可能选中短边。此时 `yaw` 差 ~90°，且 width/depth
   互换。IoU 已经把这种情况判成匹配；尺寸诊断仍算两套候选，取
   `|Δw|+|Δd|+|Δyaw|` 更小的，选中 b 时 `swapped=True`。

3. **近似正方形**：用 **GT** 的 `max(W,D)/min(W,D) < 1.15`（不是 `|W−D|<2cm`）。
   近方时令 `err_yaw = 0`，通过则 `reason="near_square"`。运行时是否信任 PCA
   yaw 由估计器的特征值比（阈值 1.2）决定，与门禁用的 GT 长宽比是两套阈值。

4. **height 不参与 swap**：z 轴由平台高度定义，没有歧义。

### 其他细节

- `err_z` 保留符号：系统性偏高/偏低（标定或平台 z 错）和随机噪声要能区分开。
- `compare` 不做单位换算，输入统一 m / rad。
- 不订阅任何 topic，不 import `luggage_msgs`。

---

## 3. 门槛

这就是检测指标（Detect vs GT），不是另起一套数。`DetectionAccuracy` 的
`tol_*` / `tol_iou` / 通过率就是门禁；评测驱动只调 `compare`，检测器不算 GT。

**已确认：现在不必改数字，也不把它们写成契约。** 表里是带推导理由的初值，
代码默认与此一致。真正要定的是第一轮 N=20 跑完之后：用实测 P95 回填，
决定维持、放宽还是收紧，依据写进 `docs/status/detection_gt_gate.md`。
没数据之前没有第二套「官方门槛」。

| 项 | 初值 | 依据 |
|---|---|---|
| `tol_xy` | 0.03 m | 吸盘接触面比箱面小得多，3 cm 偏心仍在面内 |
| `tol_z` | 0.02 m | 高度直接决定 attach 的 Z，误差直接撞箱或悬空 |
| `tol_size` | 0.05 m | 尺寸只用于 catalog snap 和可视化，不进运动学 |
| `tol_yaw` | 0.15 rad (≈8.6°) | 折叠后；近方箱不参与 yaw 门槛 |
| `tol_iou` | 0.60 | 3 cm 平移仍过；长箱真 90° 无 swap 不过 |
| `min_aspect` | 1.15 | GT 长宽比低于此则不算标量 yaw |
| 通过率 | ≥ 90% (N=20) | 留出 spawner 抽到极端长宽比的余量 |

**门禁规则**：不过门 → 记 fail 并 **skip 该轮的 motion**，不是终止整个评测。
要统计的是"检测过门率"和"过门后规划成功率"两个独立指标；混在一起就没法归因。

---

## 4. 采样流程

每轮：

1. `SpawnNextBox`（continuous 模式，每次随机尺寸）
2. `GoToRobotPose(pickup_observe)`——M3 未就绪时用现成 FJT 脚本发同样的关节目标，
   不阻塞本 todo
3. **等预处理 `geometry_ok=true`**：订 `/luggage/preprocessed/status`
   （RELIABLE + TRANSIENT_LOCAL），确认 `flags.geometry_ok` 且
   `motion_gate.state == "stable"`。跳过这步测到的是运动模糊，不是检测精度。
4. `DetectLuggage`
5. `GetCurrentBox`
6. `DetectionAccuracy.compare`
7. `ClearCurrentBox`

重复 N=20。

### 时序细节

- 预处理输出实测 **4–6 Hz**（raw 相机 30 Hz），瓶颈是全分辨率点云在 Python 里
  解码 + 变换。第 3 步的等待超时按这个量级设，别按 30 Hz 假设"下一帧马上到"。
- 开语义后还要再串一级 YOLO 推理，CPU backend 下更慢。等待超时给到 10 s 以上。
- 箱子 spawn 后有物理沉降，`GetCurrentBox` 拿到的是 spawn 参数还是实际位姿要
  确认；上一轮实测 GT 的 z 是稳定的，但换更高的箱子后值得复查。

### 失败码

检测器的 `_last_failure_reason` 已有这套取值，报表直接复用，不要另造一套：

`not_run` / `DETECT_NO_CLOUD` / `DETECT_STALE_CLOUD` / `DETECT_TOO_FEW_POINTS` /
`DETECT_TF_FAILED` / `DETECT_ESTIMATION_FAILED` / `DETECT_LOW_CONFIDENCE` / `ok`

从 `~/diagnostics_json`（transient_local）读。

### 两套口径的问题

检测器自己有 `evaluation_compare_gt` 参数（默认 false），开了会在节点里写对比
日志。**报表以 `DetectionAccuracy` + 驱动为准**，检测器那套只作调试。两边同时
出数会导致对不上时不知道信谁。要么保持关闭，要么在报表里明确标注哪份是权威。

---

## 5. 测试

`test_detection_accuracy.py`，纯 numpy：

- 完全一致 → `ok=True`，各项误差 0，`iou ≈ 1`
- 位置差 `tol_xy` 边界内外各一例
- yaw 差 π → `err_yaw ≈ 0`（180° 歧义），`iou ≈ 1`
- yaw 差 π/2 且 W/D 互换 → `ok=True`，`swapped=True`，`iou ≈ 1`
- yaw 差 π/2 但尺寸没换 → `reason="iou"`（真的转错了）
- 近方箱 90°（同一占用）→ `err_yaw == 0`，`reason="near_square"`
- 尺寸超标 → `reason` 指向 size 而不是 position
- `summarize` 的 P50/P95 与通过率算得对（构造已知分布）

## 6. 验收

- N=20 全部有 `AccuracyResult`，无异常中断
- 报表含：每轮 GT/测量/误差、P50/P95、通过率、失败码分布
- 结论明确写出是否放行 todo 3
- 若不过门，报表要指出主因是 XY、Z、尺寸还是 yaw，以及是否与
  `DETECT_TOO_FEW_POINTS`（语义裁得太狠）相关
