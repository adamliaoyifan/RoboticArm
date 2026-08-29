# Suitcase 候选模型（仅下载，未转 DAE/OBJ）

本地目录：`third_party/suitcase_candidates/`  
用途：先肉眼挑选，再决定转 DAE/OBJ 进 Gazebo Fortress。当前 **没有** 做格式转换。

现代带轮旅行箱的免费直链很少（Poly Pizza / Sketchfab / CGTrader 多数要登录或付费）。下面 4 包都是可公开直下、许可清楚的。

---

## 怎么看

| 编号 | 最快预览 | 建议 |
|------|----------|------|
| 01 | 打开 `01_oga_cases_loafbrr/preview.png` | **最像旅行箱**（带轮、拉杆）。预览是**打开**的；glTF 里有独立 `SuitCaseLid`，可在 Blender 里合上。 |
| 02 | 打开 `02_oga_suitcase_bomb_yughues/preview.jpg` | 开盖公文箱 + 钱/道具电路，不太像托运行李。已有 OBJ。 |
| 03 | 只有 `brunner_case.blend`，需 Blender 打开 | 低模道具箱，不像带轮旅行箱。 |
| 04 | 打开 `04_zenodo_vintage_suitcase/preview.jpg` | **合上的矩形箱**，纹理最好，适合吸盘/RANSAC 的箱体外形。复古帆布，不是硬壳带轮。 |

建议优先看 **01（旅行箱外形）** 和 **04（合上的箱体 + 纹理）**。02/03 更像游戏道具，可以当对照丢掉。

---

## 01 — OpenGameArt “Cases”（Loafbrr）

- 页面：https://opengameart.org/content/cases
- 文件：`01_oga_cases_loafbrr/suitcase_fbx_gltf_blend_textures.zip`
- 许可：**CC0**（包内 `README.txt` 写明无需署名）
- 内容：
  - `extract/FBXs/SuitCase.fbx` — 带轮直立旅行箱
  - `extract/FBXs/BriefCase.fbx` — 公文箱
  - `extract/GLTF/Cases.gltf` — 两者合一
  - PBR 贴图（Base Color / Normal / Metallic / Roughness / AO）
- 注意：官方预览是开盖状态。转 Gazebo 前需要合盖，否则顶面不是平面，对深度/RANSAC 不友好。

## 02 — OpenGameArt “Suitcase Bomb Pack”（Yughues / Nobiax）

- 页面：https://opengameart.org/content/suitcase-bomb-pack （页面标 **CC0**）
- 文件：`02_oga_suitcase_bomb_yughues/suitcase.zip`
- 包内 `readme.txt` 仍写 **CC BY-SA 3.0**。OGA 后来把一批 Nobiax 包改成 CC0；若要保守，按 BY-SA 署名 + 相同协议衍生。
- 内容：`suitcase.obj`、`suitcase.FBX`，1024 贴图（diffus / normal / specular / glow）
- 外形：开盖硬壳公文箱，内有美钞和发光面板。**已有 OBJ**，不必再转，但视觉上不像行李。

## 03 — OpenGameArt Blade Runner case

- 页面：https://opengameart.org/content/brunner-case
- 文件：`03_oga_brunner_case/brunner_case.zip`
- 许可：页面标 **CC0**
- 内容：仅 `extract/brunner_case.blend`（无 OBJ/FBX/glTF）
- 需要 Blender 才能预览和导出。低模道具箱，不是旅行箱。

## 04 — Zenodo Vintage Suitcase [derivative]

- 记录：https://zenodo.org/records/10389120 （DOI `10.5281/zenodo.10389120`）
- 文件：`04_zenodo_vintage_suitcase/vintage_suitcase.glb`（约 4.3 MB）
- 许可：Zenodo 元数据为 **CC BY-SA 2.5**。衍生自 Santa Cruz MAH 的 Sketchfab 扫描 [Suitcase](https://sketchfab.com/models/90f9619a6ce44c949bd6df2b77cdcacd)（CC BY 4.0）。用的话需要署名。
- 外形：合上的复古帆布箱，圆角 +  rivet + 前把手。箱体接近长方体，碰撞仍可用 `<box>`。

---

## 未下载（需要登录或没有直链）

- Poly Pizza J-Toastie *Suitcase*（CC BY）：https://poly.pizza/m/041xs8FnZZ
- Poly Pizza Don Carson *Simple Suitcase*、get wilde *Carry-on Luggage*
- CGVista vintage suitcase（标 CC0，站点 403）
- Sketchfab 原扫描（需账号）

若 01/04 都不满意，可以说要再找带轮硬壳、且默认合盖的模型。
