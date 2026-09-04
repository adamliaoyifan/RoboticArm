#!/usr/bin/env python3
"""Generate Gazebo model from a real CAD STL (visual full mesh + simplified collision mesh)."""

from __future__ import annotations

import argparse
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
import yaml


@dataclass
class RealContainerResult:
    model_dir: Path
    visual_path: Path
    collision_path: Path
    visual_triangles: int
    collision_triangles: int
    z_shift: float
    xy_shift: tuple[float, float]
    watertight: bool


def _clean_mesh(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def _anchor_floor_z(mesh: o3d.geometry.TriangleMesh) -> tuple[o3d.geometry.TriangleMesh, float]:
    verts = np.asarray(mesh.vertices)
    if verts.size == 0:
        return mesh, 0.0
    z0 = float(verts[:, 2].min())
    if z0 >= -1e-6:
        return mesh, 0.0
    verts = verts.copy()
    verts[:, 2] -= z0
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.compute_vertex_normals()
    return mesh, z0


def _anchor_bottom_center(
    mesh: o3d.geometry.TriangleMesh,
    z_tol: float = 0.01,
) -> tuple[o3d.geometry.TriangleMesh, tuple[float, float]]:
    """Translate XY so the bottom-face bbox center sits at the link origin."""
    verts = np.asarray(mesh.vertices)
    if verts.size == 0:
        return mesh, (0.0, 0.0)

    z_min = float(verts[:, 2].min())
    bottom = verts[verts[:, 2] <= z_min + z_tol]
    if bottom.size == 0:
        bottom = verts

    cx = float((bottom[:, 0].min() + bottom[:, 0].max()) * 0.5)
    cy = float((bottom[:, 1].min() + bottom[:, 1].max()) * 0.5)
    if abs(cx) <= 1e-6 and abs(cy) <= 1e-6:
        return mesh, (0.0, 0.0)

    verts = verts.copy()
    verts[:, 0] -= cx
    verts[:, 1] -= cy
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.compute_vertex_normals()
    return mesh, (cx, cy)


def _rpy_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _rotate_mesh_rpy(
    mesh: o3d.geometry.TriangleMesh,
    rpy: list[float] | tuple[float, float, float],
) -> o3d.geometry.TriangleMesh:
    """Rotate mesh about the link origin (bottom-center anchor point)."""
    roll, pitch, yaw = [float(v) for v in rpy]
    if abs(roll) <= 1e-9 and abs(pitch) <= 1e-9 and abs(yaw) <= 1e-9:
        return mesh
    rot = _rpy_rotation_matrix(roll, pitch, yaw)
    mesh.rotate(rot, center=(0.0, 0.0, 0.0))
    mesh.compute_vertex_normals()
    return mesh


def _mesh_aabb(mesh: o3d.geometry.TriangleMesh) -> tuple[np.ndarray, np.ndarray]:
    verts = np.asarray(mesh.vertices)
    if verts.size == 0:
        zeros = np.zeros(3)
        return zeros, zeros
    return verts.min(0), verts.max(0)


def _suggest_container_dims(mn: np.ndarray, mx: np.ndarray) -> dict[str, Any]:
    size = mx - mn
    length = float(size[0])
    width = float(size[1])
    height = float(size[2])
    opening_xyz = [0.0, round(width * 0.5, 2), round(height * 0.5, 2)]
    inner_scale = 0.95
    return {
        "outer": {
            "length": round(length, 2),
            "width": round(width, 2),
            "height": round(height, 2),
        },
        "inner": {
            "length": round(length * inner_scale, 2),
            "width": round(width * inner_scale, 2),
            "height": round(height * inner_scale, 2),
        },
        "opening_frame_xyz": opening_xyz,
    }


def _decimate(mesh: o3d.geometry.TriangleMesh, target: int) -> o3d.geometry.TriangleMesh:
    n = len(mesh.triangles)
    if n <= target or n == 0:
        return mesh
    try:
        return mesh.simplify_quadric_decimation(target_number_of_triangles=target)
    except Exception:
        span = float(mesh.get_max_bound().max() - mesh.get_min_bound().min())
        return mesh.simplify_vertex_clustering(voxel_size=max(span * 0.01, 0.01))


def build_mesh_sdf(
    model_name: str,
    friction_mu: float,
    link_pose: str = "0 0 0 0 0 0",
) -> str:
    mu = float(friction_mu)
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="container_link">
      <pose>{link_pose}</pose>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/container_visual.stl</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.25 0.35 0.45 1</ambient>
          <diffuse>0.35 0.45 0.55 1</diffuse>
        </material>
      </visual>
      <collision name="collision_mesh">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/container_collision.stl</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu}</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""


CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>real CAD STL pipeline</name>
  </author>
  <description>Real container from CAD STL with simplified mesh collision</description>
</model>
"""


def _log_mesh_stats(path: Path, label: str) -> tuple[np.ndarray, np.ndarray]:
    mesh = o3d.io.read_triangle_mesh(str(path))
    verts = np.asarray(mesh.vertices)
    if verts.size == 0:
        print(f"WARN empty mesh: {path}")
        return np.zeros(3), np.zeros(3)
    mn, mx = verts.min(0), verts.max(0)
    print(
        f"{label} {path.name}: tris={len(mesh.triangles)} "
        f"min={mn.round(4).tolist()} max={mx.round(4).tolist()}"
    )
    return mn, mx


def _default_install_dir(model_name: str) -> Path:
    pointcloud_dir = Path(__file__).resolve().parents[1]
    src_dir = pointcloud_dir.parent
    return src_dir / "luggage_gazebo" / "models" / model_name


def generate_real_container_model(cfg: dict[str, Any]) -> RealContainerResult:
    pointcloud_dir = Path(__file__).resolve().parents[1]
    input_stl = Path(cfg.get("input_stl", "./realcontainer.STL")).expanduser()
    if not input_stl.is_absolute():
        input_stl = (pointcloud_dir / input_stl).resolve()

    output_dir = Path(cfg.get("output_dir", "./output/real_container_gazebo")).expanduser()
    if not output_dir.is_absolute():
        output_dir = (pointcloud_dir / output_dir).resolve()

    model_name = str(cfg.get("model_name", "airport_container_real"))
    collision_triangles = int(cfg.get("collision_triangles", 8000))
    friction_mu = float(cfg.get("friction_mu", 0.6))

    if not input_stl.exists():
        raise FileNotFoundError(f"Input STL not found: {input_stl}")

    mesh = o3d.io.read_triangle_mesh(str(input_stl))
    if len(mesh.triangles) == 0:
        raise ValueError(f"Empty STL: {input_stl}")

    mesh = _clean_mesh(mesh)
    watertight = bool(mesh.is_watertight())
    z_shift = 0.0
    if cfg.get("anchor_floor_z", True):
        mesh, z_shift = _anchor_floor_z(mesh)

    xy_shift = (0.0, 0.0)
    if cfg.get("anchor_bottom_center", True):
        mesh, xy_shift = _anchor_bottom_center(mesh)

    align_rpy = cfg.get("mesh_align_rpy", [0.0, 0.0, 0.0])
    if align_rpy and any(abs(float(v)) > 1e-9 for v in align_rpy):
        mesh = _rotate_mesh_rpy(mesh, align_rpy)
        if cfg.get("anchor_floor_z", True):
            mesh, z_shift_extra = _anchor_floor_z(mesh)
            z_shift += z_shift_extra
        if cfg.get("anchor_bottom_center", True):
            mesh, xy_shift_extra = _anchor_bottom_center(mesh)
            xy_shift = (
                xy_shift[0] + xy_shift_extra[0],
                xy_shift[1] + xy_shift_extra[1],
            )

    visual = _clean_mesh(mesh)
    collision = _clean_mesh(_decimate(mesh, collision_triangles))

    model_dir = output_dir / model_name
    mesh_dir = model_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    visual_path = mesh_dir / "container_visual.stl"
    collision_path = mesh_dir / "container_collision.stl"
    o3d.io.write_triangle_mesh(str(visual_path), visual)
    o3d.io.write_triangle_mesh(str(collision_path), collision)

    (model_dir / "model.sdf").write_text(
        build_mesh_sdf(model_name, friction_mu), encoding="utf-8"
    )
    (model_dir / "model.config").write_text(
        CONFIG_TEMPLATE.format(model_name=model_name), encoding="utf-8"
    )

    if cfg.get("install", False):
        install_dir = cfg.get("install_dir")
        if install_dir:
            install_path = Path(install_dir).expanduser().resolve()
        else:
            install_path = _default_install_dir(model_name)
        if install_path.exists():
            shutil.rmtree(install_path)
        shutil.copytree(model_dir, install_path)
        model_dir = install_path

    vis_mn, vis_mx = _log_mesh_stats(visual_path, "Visual")
    col_mn, col_mx = _log_mesh_stats(collision_path, "Collision")
    aabb_diff = np.max(np.abs(vis_mx - col_mx)) + np.max(np.abs(vis_mn - col_mn))
    print(f"Z shift applied: {z_shift:.4f} m")
    print(f"XY shift applied: ({xy_shift[0]:.4f}, {xy_shift[1]:.4f}) m")
    if align_rpy and any(abs(float(v)) > 1e-9 for v in align_rpy):
        print(f"Mesh align RPY applied: {[round(float(v), 4) for v in align_rpy]}")
    bottom_center = (vis_mn[:2] + vis_mx[:2]) * 0.5
    print(f"Bottom-face center (XY): {bottom_center.round(4).tolist()}")
    suggest = _suggest_container_dims(vis_mn, vis_mx)
    print("Suggested scene_tf container.outer:", suggest["outer"])
    print("Suggested scene_tf container_opening_frame translation:", suggest["opening_frame_xyz"])
    print("Suggested scene_tf container_link rotation_rpy: [0.0, 0.0, 0.0]  (rotation baked into mesh)")
    print(f"Watertight: {watertight}")
    print(f"Collision vs visual AABB max corner diff: {aabb_diff:.4f} m")
    if not watertight:
        print(
            "NOTE: mesh is not watertight; if Gazebo collision is unstable, "
            "increase collision_triangles or use box collision fallback."
        )

    return RealContainerResult(
        model_dir=model_dir,
        visual_path=visual_path if not cfg.get("install") else model_dir / "meshes" / "container_visual.stl",
        collision_path=collision_path if not cfg.get("install") else model_dir / "meshes" / "container_collision.stl",
        visual_triangles=len(visual.triangles),
        collision_triangles=len(collision.triangles),
        z_shift=z_shift,
        xy_shift=xy_shift,
        watertight=watertight,
    )


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "config" / "real_container_gazebo.yaml",
    )
    parser.add_argument("--install", action="store_true", help="Override config install flag")
    args = parser.parse_args()

    if not args.config.exists():
        example = args.config.parent / "real_container_gazebo.yaml.example"
        print(f"Config missing: {args.config}\nCopy: cp {example} {args.config}", file=sys.stderr)
        return 1

    cfg = load_config(args.config)
    if args.install:
        cfg["install"] = True

    try:
        result = generate_real_container_model(cfg)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"Gazebo model at: {result.model_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
