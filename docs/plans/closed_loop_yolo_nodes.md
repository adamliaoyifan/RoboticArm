# Todo 1 — YOLO 语义节点与 cargo 点云

对应 plan todo `yolo-nodes`。上游背景见仓库外的
`~/.cursor/plans/closed-loop_gap_analysis_4401de44.plan.md`；规范以
[perception_architecture.md](../architecture/perception_architecture.md) 与
[sensor_data_pipeline.md](../architecture/sensor_data_pipeline.md) 为准。

## 范围

把 `SemanticSegmenter` / `SemanticPointFilter` 两个已有算法类接进 Humble 图，让
`luggage_detector` 吃语义 cargo 点云而不是整片点云。

**不做**：新分割算法、新估高算法、SAM2 实例分割上线、`robot_self_point_filter.py`
的 rospy 清理、把 ROS 1 节点整文件搬过来。

## 交付物

| 文件 | 性质 |
|---|---|
| `luggage_perception/luggage_perception/semantic_segmenter.py` | 改：加 `update` / `copy_output` / `SegmenterOutput` |
| `luggage_perception/scripts/semantic_segmenter_node.py` | 新 |
| `luggage_perception/scripts/semantic_point_filter_node.py` | 新 |
| `luggage_perception/luggage_perception/ros_message_adapters.py` | 改：加 mask / instance mask 编码 |
| `luggage_perception/config/semantic_segmenter.yaml` | 改：清理 `sync_slop` |
| `luggage_perception/CMakeLists.txt` | 改：`install(PROGRAMS ...)` 两个新节点 |
| `luggage_gazebo/launch/sim_world.launch.py` | 改：起两个节点，detector `use_semantic:=true` |
| `luggage_perception/test/test_semantic_segmenter.py` | 改：补 `update`/`copy_output` 用例 |

---

## 1. 算法类改造：`SemanticSegmenter`

### 当前不合规之处

`semantic_segmenter.py` 是一次性 `segment(rgb) -> (label_map, detections)`：

- 没有 `update`，不保存"当前帧"，无 stamp / frame_id
- `instance_map` 属性（L158–161）直接 `return self._instance_map`，调用方可改内部数组
- `last_stats` 拷了 dict，但 mask / detections 没有拷贝出口

这两条在 [architecture/README.md](../architecture/README.md) 的 known deviations 里已登记。

### 目标 API

基类实现一次，四个 backend 仍然只写 `segment()`，签名不动。

```python
@dataclass(frozen=True)
class SegmenterOutput:
    stamp: float
    frame_id: str
    label_map: np.ndarray        # HxW uint8
    detections: tuple            # tuple of dict, 已深拷贝
    instance_map: np.ndarray     # HxW uint16 或 None
    stats: dict

    def copy(self): ...          # 深拷贝所有可变字段
```

```python
class SemanticSegmenter:
    def update(self, rgb_uint8, stamp, frame_id) -> None:
        """跑 segment()，把结果深拷贝进 self._output。"""

    def copy_output(self) -> SegmenterOutput | None:
        """唯一对外读取口，再拷一份出去。没结果返回 None。"""
```

**细节**

- `update` 内部调 `self.segment(rgb)`，然后 `np.copy(label_map)`、
  `copy.deepcopy(detections)`、`np.copy(instance_map)`（若非 None）。
  backend 返回的数组可能是内部复用 buffer，不拷会串帧。
- `detections` 存成 `tuple`，元素是新 dict；SAM2 的 `mask` 字段是 `HxW bool`，
  也要拷。
- `instance_map` 属性保留但改为返回拷贝，或直接标 deprecated 只留 `copy_output`。
  推荐后者，少一个出口少一处泄漏。
- `update` 不发布、不做 I/O。
- stamp / frame_id 一律从输入 header 继承，**禁止 `now()`**。

### backend fallback 不许静默

`build_segmenter` 在 `ImportError` / 任意异常时会退回 stub，并把
`stats["backend"]` 写成 `"stub(fallback:yolo_world:ultralytics)"`（L483–490）。
这个行为保留，但节点侧要能拒绝：

- 节点参数 `require_backend`（默认空）。非空且实际 `stats["backend"]` 不以它开头
  时，**启动即 `RuntimeError`**，不要等跑完一轮评测才发现测的是 stub。
- 评测运行必须设 `require_backend: yolo_world`。

---

## 2. `semantic_segmenter_node`

### 订阅

`/luggage/preprocessed/camera/color/image`，**不是 raw**。

原因：mask 要继承 RGB 的 `primary_stamp`，下游 point filter 才能和
`/luggage/preprocessed/camera/depth/points` 做精确 stamp join。订 raw 会拿到
预处理没有采纳的那些帧，stamp 对不上，join 永远空。

**QoS 陷阱**：gz `ros_gz_bridge` 在 raw 相机话题上提供 **RELIABLE**，此 RMW 下
用 BEST_EFFORT 订 raw 能连上但几乎收不到数据（预处理节点就是因此改成 RELIABLE
订 raw 的，见 `sensor_preprocessor_node.py` 里的注释）。而 preprocessed 输出是
BEST_EFFORT，所以这里用 `BEST_EFFORT, depth=1` 正确。别把两种情况抄混。

### 发布

| 话题 | 类型 | 条件 |
|---|---|---|
| `/luggage/semantic/mask` | `Image` `mono8`，像素=label id 0–4 | always |
| `/luggage/semantic/overlay` | `Image` `bgr8` | `publish_overlay` 且 cv2 可用 |
| `/luggage/semantic/instance_mask` | `Image` `mono16` | 仅当 `instance_map is not None` |
| `~/stats_json` | `std_msgs/String` | transient_local |

header 的 stamp / frame_id 原样继承输入。

**不做**：不发 3D 点（那是 1C）、不做 TF、不估高、不写
`rospy.set_param`（ROS 1 节点用 `/luggage/semantic/detections` 等三个全局参数
传 detections，ROS 2 不复制这套，走 `~/stats_json`）。

### 节点主体

```
订阅回调:
    frame = adapters.rgb_frame_from_msg(msg)   # None 则丢帧 + 限频告警
    segmenter.update(frame.image, frame.stamp, frame.frame_id)
    out = segmenter.copy_output()
    publish(mask/overlay/instance)
```

`max_rate_hz` 参数（ROS 1 有，默认 0=不限）建议保留：YOLO 在 CPU 上远慢于
4–6 Hz 的输入，不限频会让回调堆积。ROS 1 那个 `self._latest_image = None`
声明后从不写入的死代码不要抄。

### 适配器要补两个函数

`ros_message_adapters.py` 目前只有 rgb8/mono8 解码和 `16UC1` 深度编码，缺
mask 编码。补：

```python
def mask_msg_from_array(label_map, stamp, frame_id)      # mono8
def instance_mask_msg_from_array(instance_map, stamp, frame_id)  # mono16
```

不要拿 `depth_msg_from_frame` 凑数（它硬编码 `encoding="16UC1"`），也不要拿
`RgbFrame(encoding="mono8")` 绕过去——类型名会误导后来的人。补完记得在
`test_ros_message_adapters.py` 里加往返用例。

overlay 走 `draw_detections_overlay`，它需要 cv2（现有测试在无 cv2 时 skip）。
节点里 try/except 包住，缺 cv2 就不发 overlay 并 warn 一次，不要让整个节点起不来。

---

## 3. `semantic_point_filter_node`

**这个节点比原计划薄很多。** 预处理落地后，TF 换帧、滤 inf、slop 配对全部
不再是它的事，在这里重做即违反 `sensor_data_pipeline.md`。

### 订阅与精确 join

| 输入 | 话题 |
|---|---|
| 点云 | `/luggage/preprocessed/camera/depth/points` |
| mask | `/luggage/semantic/mask` |
| 相机内参 | `/luggage/preprocessed/camera/color/camera_info` |

join 规则：**完全相同的 `(sec, nanosec)`**。实现方式：

```python
self._clouds = {}   # stamp_key -> msg，bounded
self._masks  = {}   # stamp_key -> msg，bounded
# 任一到达时查另一边有没有同 key；命中则处理并从两边删除
# 超过 N 条（建议 10）按 stamp 淘汰最旧的
```

不要用 `message_filters.ApproximateTimeSynchronizer`。容差配对只允许出现在
预处理里。

**yaml 里遗留的 `point_filter.sync_slop: 0.05` 与精确 join 冲突，删掉。**
（若担心兼容，改成 `0.0` 并注释说明，但删掉更干净。）

### 相机内参从哪来 —— 最容易踩的坑

`SemanticPointFilter` 需要 color 内参 + `depth_to_color` 外参把点投到像素。
`realsense_d435.yaml` 的 K 已与仿真对齐：`fx=fy=(640/2)/tan(1.5184/2)`
≈ **337.22194822727283**，来源是 Intel D435 官网 depth HFOV 87° 和
Gazebo `horizontal_fov`，与 live `/camera/depth/camera_info` 一致。
旧值 607.53 / 382.72 是未核实的真机样本，已废弃。

Humble 节点仍应优先从**在线 `camera_info`** 构造 `CameraIntrinsics`，yaml
只作离线 fallback。外参加参数 `extrinsics_source`：

- `identity`（仿真默认）：gz 的 color 与 depth 是同一个传感器，
  `rotation=I`、`translation=0`
- `config`（真机）：读 `realsense_d435.yaml` 的 `camera.extrinsics.depth_to_color`

另注：phase 1 预处理里 color 与 depth 两个 camera_info 槽都由 gz 那一个
`/camera/depth/camera_info` 喂（见 `sensor_preprocessor.yaml` 的
`input.color_camera_info: ""`），所以仿真里订 color_info 拿到的就是 depth 的
内参——这正好和 `extrinsics_source: identity` 自洽。

`SemanticPointFilter.__init__` 还要一个 `depth_intrinsics` 参数，但
`filter_points` 的投影其实只用 color 内参（L179–181），传同一个进去即可。

### 输出

| 话题 | 内容 |
|---|---|
| `/luggage/semantic/cargo_points` | `PointCloud2`，header 沿用输入点云的 stamp 与 `camera_depth_optical_frame` |
| `/luggage/semantic/obstacle_points` | 同上 |
| `~/stats_json` | `String`，`last_stats` + join 命中率 |

**点云字段**：本轮只发 `xyz`（float32, point_step=12），直接用
`adapters.cloud_msg_from_points`。ROS 1 那套 16 字节带 `label` uint8 +
`instance_id` uint16 的布局本轮不需要——下游只有 detector，它只读 xyz。等
真要做实例级抓取再扩，扩的时候连同 `ros_message_adapters` 一起加编码函数和测试。

### 不做

- TF 查询：点云已在 `camera_depth_optical_frame`，和 mask 同系，直接投影
- `isfinite`：预处理已滤（防御性保留一行无妨，但不是它的职责，也不要计入 stats 当卖点）
- `cloud_data_frame` 参数：不要引入
- **`fallback_to_raw` 保持 `false`**。无 mask 就不发 cargo 点云，在 stats 里
  写明原因。把未过滤点云当有效货物几何发出去，是验收标准明令禁止的一条。
  ROS 1 节点代码默认 `True`（L90）而 yaml 是 `false`，端口过来时别把代码默认抄成 True。

---

## 4. `luggage_detector` 改接线

只改一处：`use_semantic:=true`。节点内已有分支（L150–160）会自动从
`cargo_cloud_topic`（默认 `/luggage/semantic/cargo_points`）订阅。

- `cloud_data_frame` 保持默认空（用 header）
- `cloud_max_age_sec=1.0` 保留：单输入消费者按 `primary_stamp` 判龄，合规
- ROI / RANSAC / 估高一律不动

**注意**：开语义后 cargo 点数量会大幅下降（只剩箱面），
`min_points`（默认 50）和 `min_confidence`（默认 0.70）可能需要重调。
先不改，在 todo 2 的重测里用数据说话。

---

## 5. launch 与打包

`sim_world.launch.py` 里的启动顺序：bridges → `depth_image_republisher` →
`sensor_preprocessor` → **segmenter → point_filter** → `luggage_detector`。

节点参数走 `semantic_segmenter.yaml`。detector 加 `use_semantic: True`。

建议加一个 launch argument `use_semantic`（默认 `false`），这样默认 launch
仍是当前这条已验收的链路，评测时显式开启，避免 YOLO 依赖缺失把日常仿真也拖挂。

`CMakeLists.txt`：

```cmake
install(PROGRAMS
  scripts/luggage_detector_node.py
  scripts/sensor_preprocessor_node.py
  scripts/semantic_segmenter_node.py
  scripts/semantic_point_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

`package.xml` 现有 exec_depend 已覆盖 rclpy / sensor_msgs / std_msgs / numpy。
若 overlay 要 cv2，加 `python3-opencv`（可选依赖，代码里要能缺省降级）。

---

## 6. 测试

**ROS-free（`test_semantic_segmenter.py` 补充）**

- `update` 后 `copy_output()` 的 stamp / frame_id 与传入一致
- 连续两次 `update`，先取的 output 不被后一次覆盖
- 改 `copy_output()` 返回的 `label_map` / `detections`，再取一次不受影响
- 没 `update` 过时 `copy_output()` 返回 `None`
- backend fallback 时 `stats["backend"]` 含 `stub(fallback:`

现有 11 条分割测试与 11 条 filter 测试全部要继续通过，`segment()` 签名不许动。

**适配器（`test_ros_message_adapters.py` 补充）**

- mono8 mask 编解码往返
- mono16 instance mask 编解码往返

**不写**需要 Gazebo 的单测。join 逻辑如果做成一个小的纯函数
（`match_by_stamp(dict_a, dict_b)`），可以脱离 ROS 单测，推荐这么切。

## 7. 验收

- `/luggage/semantic/mask` 与 `/luggage/preprocessed/camera/color/image` stamp 逐帧相同
- `/luggage/semantic/cargo_points` 的 stamp 属于预处理点云 stamp 集合，帧率不低于点云的 80%
- stats 里 `backend` 是 `yolo_world:...` 而非 `stub(fallback:...)`
- `cargo_count` 显著小于 `raw_count`（点云确实被裁到箱面）
- `DetectLuggage` 仍返回 `perception estimate`，不是 GT 回退
- 停掉 segmenter：cargo 点云停发，detector 报 `DETECT_NO_CLOUD` 或
  `DETECT_STALE_CLOUD`，**不得**出现整片点云被当 cargo 发出去
