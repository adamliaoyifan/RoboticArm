# 点云工具：房间摘箱 + 参数化箱体 Gazebo 模型

## 一、从房间 LAS 摘出 Cargo（CloudCompare / 脚本）

见 [`extract_cargo_from_room.py`](scripts/extract_cargo_from_room.py)（若存在）；配置 `config/extract_cargo.yaml.example`。

---

## 二、参数化测箱 → Gazebo（主流程）

**点云只用于测量参数**；几何由程序化 CAD 生成（六边形主体 + 开口 + 四条腿），不再依赖密集角点/棱线提取或 Poisson/BPA 表面重建。

输入：已裁剪的箱体点云（**真实 LAS / LAZ / PLY**，配置里写**绝对路径**）。  
**不要**把 CloudCompare 原生 `.bin`（文件头 `CCB2`）直接当 LAS 用；若从 CC 保存的文件扩展名是 `.las` 但打开报 `Invalid file signature CCB2`，见下方排障。

### 安装

```bash
cd ros2_ws/src/elfin_trajectory_executor/pointcloud
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

```bash
cp config/measure_cargo.yaml.example config/measure_cargo.yaml
# 编辑 input_las 或 input_cloud: "/你的路径/box_export.las"
```

### 全流程

```bash
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml --install-gazebo
```

等价步骤：`preprocess → fit → cad → gazebo`

### 分步

```bash
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml --only preprocess
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml --only fit
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml --only cad
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml --only gazebo --install-gazebo
# 或单独安装：
python3 scripts/generate_gazebo_model.py --install
```

### 输出

| 文件 | 说明 |
|------|------|
| `output/up_axis.json` | 检测到的竖直轴与高度跨度 |
| `output/z_layers.json` | `floor_top_z`、`deck_bottom_z`、空隙与各层点数 |
| `output/aligned_pre_split.ply` | 重力对齐后、分割前的点云（验收 Z 是否竖直） |
| `output/object_points.ply` | 去 outlier、去地面后的箱体点云 |
| `output/ground_points.ply` | 地面点（近水平薄层） |
| `output/body_points.ply` | 主体区域（保留稀疏/洞，不强行删细节） |
| `output/leg_points.ply` | 腿区域 |
| `output/parametric_model.json` | 六边形、开口、腿参数 |
| `output/measured_container.yaml` | 外廓/开口/腿 — 可拷到 `luggage_description/config/` |
| `output/report.txt` | 拟合摘要 |
| `output/parametric_debug.ply` | 补全后的六边形轮廓 + 腿角点（CloudCompare 验收） |
| `output/meshes/container_visual.stl` | Gazebo visual（默认来自 CAD；可用第四节点云表面重建覆盖） |
| `output/meshes/surface_debug.ply` | 分簇着色 debug mesh（仅表面重建流程） |
| `output/meshes/container_collision.stl` | 碰撞盒合并 mesh（参考用） |
| `output/collision_boxes.json` | SDF 多 box 碰撞源数据 |
| `output/gazebo/airport_container_measured/` | 本地 SDF 包 |

安装后：

`elfin_noetic_ws/src/luggage_gazebo/models/airport_container_measured/`

### Gazebo 使用

```bash
roslaunch luggage_gazebo sim_world.launch \
  spawn_suitcases:=false \
  container_model:=airport_container_measured
```

终端会打印 STL AABB；**max z 应约 2 m**（与箱体高度一致）。规划/TF 可读 `measured_container.yaml`。

### 流水线说明

1. **预处理**（`cargo_preprocess.py`）：SOR → DBSCAN → **检测 up 轴并旋到 Z-up** → **水平 RANSAC 地面** → **腿区 XY DBSCAN 柱体** → 开口 yaw  
2. **参数拟合**（`cargo_parametric_fit.py`）：六边形 XZ 轮廓、立面开口矩形、腿 DBSCAN + 四角补全缺失腿  
3. **CAD**（`cargo_cad_model.py`）：挤出 watertight visual mesh、开口三角剔除、腿 box；`collision_boxes.json`  
4. **Gazebo**（`generate_gazebo_model.py`）：visual=STL，collision=多个 `<box>`  

旧模块 [`cargo_features.py`](scripts/cargo_features.py)、[`cargo_mesh.py`](scripts/cargo_mesh.py) 仍保留，**默认不再调用**。

### 自测（无真实 LAS）

```bash
python3 scripts/make_measured_cargo_las.py -o /tmp/measured_cargo_test.las
# config/measure_cargo.yaml 中 input_las 指向该文件
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml
```

### 验收（计划对照）

1. `object_points.ply`：远离箱子的 outliers 已删，箱体和腿仍完整  
2. `body_points.ply`：仅主体，洞/稀疏保留  
3. `leg_points.ply`：3~4 条腿；`measured_container.yaml` 中缺失腿 `inferred: true`  
4. `parametric_debug.ply`：六边形主体 + 四条腿示意  
5. Gazebo：六边形箱体、开口、四条腿，非贴地薄片或长方体占位  

### CloudCompare BIN（`CCB2`）排障

| 现象 | 处理 |
|------|------|
| `Invalid file signature "b'CCB2'"` 或脚本提示 CCB2 | 文件是 **CloudCompare 原生 BIN**，不是 LAS。在 CC 中 **File → Save As → LAS 1.4 / PLY**，或：`scripts/convert_cloudcompare_bin.sh box.las box_export.las`（需 `sudo apt install cloudcompare`），再把 `input_las` 指向 `box_export.las` |
| 仅有 `.bin` / 误命名 `.las` | 同上；支持格式：`.las` `.laz` `.ply` `.pcd` `.xyz` |

### 地面 / 腿 / 坐标系排障

| 现象 | 处理 |
|------|------|
| `ground_points` 像箱底大平面（十几万点） | 确认 `preprocess.ground.mode: z_min_gap`；看 `z_layers.json` 中 `floor_top_z` 应 **< 0.06 m**，`deck_bottom_z` **~0.5 m** |
| 真实地面/桌角未进 `ground_points` | 减小 `floor_max_thickness`（如 0.04）；或略增 `floor_clearance` |
| `ground_points` 像竖墙、`leg_points` 像地毯 | **up 轴错**：看 `aligned_pre_split.ply`；试 `preprocess.up_axis: z` / `y` |
| 箱底高度不对 | 手填 `segmentation.body_bottom_z: 0.53`（锚定 Z=0 后约为 0.53） |
| 箱底大平面进了 `leg_points` | 默认 `segmentation.method: plane_and_vertical_obb`；或 `deck_exclude_band: 0.04`；`z_band_only` 作回退 |
| 竖直腿进 `body`、腿点很少 | 减小 `leg_dbscan_eps`；检查 `deck_bottom_z`；看 `leg_obb_debug.ply` |

**分割验收（`box.las`）**：`ground_points` ~3 万；`body_points` 含 z≈0.53 箱底；`leg_points` 为 3–4 条竖柱且左右高度可不同；`planes.json` / `z_layers.json` / `leg_obb_debug.ply` 在 `output_dir`。

### 调参提示

| 现象 | 处理 |
|------|------|
| 远离箱子的飞点仍在 | 收紧 `preprocess.dbscan.eps` 或 `keep_largest_components: 1` |
| 腿被并进主体 | `deck_exclude_band`；`method: z_band_only`；或 `body_bottom_z` |
| 地面削掉腿底 | 增大 `preprocess.ground.ground_clearance` |
| 开口位置不对 | `fit.opening.manual` 指定 `x_min/x_max/z_min/z_max` |
| 缺腿未补全 | 检查 `leg_points.ply`；腿 DBSCAN 减小 `fit.legs.dbscan_eps` |

### 旧 Poisson/BPA 排障（仅在使用旧脚本时）

若仍手动跑 `cargo_mesh.py` 且 Gazebo 显示贴地薄片：检查对齐后高度、`mesh.min_height_span`；优先改用本参数化流程。

---

## 四、点云 Visual STL（独立）

从预处理产物重建**贴近扫描外形**的 `container_visual.stl`（平面分簇 BPA + 整云回退、保留开口），**不修改** `measure_cargo_from_las.py` 主流程。碰撞仍用 `collision_boxes.json`（参数化 fit/cad）。

### 前置

```bash
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml --only preprocess
python3 scripts/measure_cargo_from_las.py -c config/measure_cargo.yaml --only fit
# fit 生成 parametric_model.json，供开口三角剔除
```

### 配置与运行

```bash
cp config/surface_visual.yaml.example config/surface_visual.yaml
python3 scripts/build_surface_visual.py -c config/surface_visual.yaml
```

输出覆盖 `output/meshes/container_visual.stl`；终端打印 STL AABB 与点云–mesh 覆盖率（默认要求 ≥85% 点在 4 cm 内）。

安装到 Gazebo（沿用现有 SDF，仅替换 visual STL）：

```bash
python3 scripts/generate_gazebo_model.py --install
```

### 算法要点

| 步骤 | 说明 |
|------|------|
| 输入 | `body_points.ply` + `leg_points.ply`，可选 `planes.json` 分簇 |
| 重建 | 每簇 Ball Pivoting，失败则 Alpha Shape；`full_body_fallback` 再建整云 BPA |
| 开口 | 读 `parametric_model.json` 的 opening，+Y 面删三角；可 `opening.manual` 覆盖 |
| 禁止 | 整云 Poisson 封门（旧 `cargo_mesh.py` 问题） |
| QA | 高度/AABB + 点云覆盖率；mesh 底面 Z 锚定后与参考点云对齐再算距离 |

### 调参

| 现象 | 处理 |
|------|------|
| QA Coverage FAIL | 减小 `mesh.voxel_size`（如 0.015）；确认未开 `mask_under_deck` |
| 开口太小/偏 | `opening.manual` 或修正 fit 后再跑 |
| mesh 太密/慢 | 降低 `mesh.visual_triangles` |
| 外形仍不像 | CloudCompare 叠加 `body_points.ply` 与 `surface_debug.ply` |

模块：[`cargo_surface_mesh.py`](scripts/cargo_surface_mesh.py)、[`build_surface_visual.py`](scripts/build_surface_visual.py)、[`cargo_mesh_opening.py`](scripts/cargo_mesh_opening.py)。

---

## 五、点云几何补全（独立）

CC 中**本来没有点**的盲区（例如立面约 50×30 cm、某条腿少 15 cm）需在点云阶段用**平面栅格 / 柱体延伸**补合成点，再跑第四节表面重建。

### 前置

与第四节相同：`preprocess` + `fit`（`measured_container.yaml`、`parametric_model.json`）。

### 配置与运行

```bash
cp config/cloud_completion.yaml.example config/cloud_completion.yaml
# 默认 auto_hole 在竖直主面上找上部缺口；若选错，在 CC 中点选缺口中心后填 center_xyz
python3 scripts/complete_cargo_cloud.py -c config/cloud_completion.yaml

# surface_visual 使用 completed PLY（见 surface_visual.yaml.example）
python3 scripts/build_surface_visual.py -c config/surface_visual.yaml
```

| 输出 | 说明 |
|------|------|
| `body_points_completed.ply` | 原体 + 墙面补丁点 |
| `leg_points_completed.ply` | 原腿 + 延伸柱点（如 front_left 向下 15 cm） |
| `completion_debug.ply` | 灰=原点，洋红=自动补丁，红=手动补丁，橙=腿补丁 |

### 配置要点

- **立面补丁**：默认 `mode: auto_hole` + `target_face: auto_vertical`；若补错，改为 `mode: manual_center` 并填 `center_xyz: [x, y, z]`。
- **补丁尺寸**：`u_half=0.25`、`v_half=0.15` → 50×30 cm；`detect_v_min` 可限制只搜索上半墙。
- **腿延伸**：`leg_extends[].id: front_left`，`length_m: 0.15`，`direction: down`。
- **开口**：合成点若落在 `parametric_model.json` 门洞盒内会自动剔除（勿补到 +Y 门上）。
- **高级模式**：仍支持 `plane_point + u_center/v_center`，但不建议作为默认定位方式，容易像示例补丁一样放偏。

碰撞与 `measured_container.yaml` 仍由参数化 fit/cad 负责；补全仅改善 **visual STL**。

模块：[`cargo_cloud_completion.py`](scripts/cargo_cloud_completion.py)、[`complete_cargo_cloud.py`](scripts/complete_cargo_cloud.py)。

---

## 三、CloudCompare GUI 摘箱（简要）

1. **File → Open** → **F** 聚焦  
2. **Segment / Crop → Box**（约 2.8×2.4×2.6 m）  
3. 必要时 **Plane** 去地、**Connected Components** 选簇  
4. 导出 **LAS 1.4 或 PLY**（勿仅保存 CC 原生 `.bin` 并改名为 `.las`）作为 `measure_cargo` 的输入  

---

## 参考尺寸

| 参数 | 值 (m) |
|------|--------|
| 外廓长 × 宽 × 高 | 2.4 × 2.0 × 2.2 |
