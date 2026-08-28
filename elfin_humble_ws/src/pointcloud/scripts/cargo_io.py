"""Shared point cloud I/O: LAS/LAZ/PLY/PCD/XYZ and CloudCompare BIN detection."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import open3d as o3d

try:
    import laspy
except ImportError as exc:
    raise SystemExit("laspy required: pip install -r requirements.txt") from exc

CCB2_ERROR = (
    "{path} is CloudCompare native BIN (header CCB2), not LAS.\n"
    "  1) In CloudCompare: File -> Save As -> LAS 1.4 or PLY, then point input_las at that file.\n"
    "  2) Or: scripts/convert_cloudcompare_bin.sh \"{path}\" box_export.las\n"
    "     (requires: sudo apt install cloudcompare)"
)

_O3D_EXTENSIONS = {".ply", ".pcd", ".xyz", ".pts"}


class CloudCompareBinError(ValueError):
    """Input file is CloudCompare BIN v2 (CCB2), not a supported point cloud format."""


def _read_magic(path: Path, n: int = 4) -> bytes:
    with path.open("rb") as fh:
        return fh.read(n)


def _load_laspy(path: Path) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    las = laspy.read(str(path))
    xyz = np.vstack((las.x, las.y, las.z)).T.astype(np.float64)
    rgb = None
    intensity = None
    if hasattr(las, "red") and hasattr(las, "green") and hasattr(las, "blue"):
        rgb = np.vstack((las.red, las.green, las.blue)).T.astype(np.float64)
        if rgb.max() > 1.0:
            rgb = rgb / 65535.0
    if hasattr(las, "intensity"):
        intensity = np.asarray(las.intensity, dtype=np.float64)
    return xyz, rgb, intensity


def _load_open3d(path: Path) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise ValueError(f"No points read from {path} (open3d)")
    xyz = np.asarray(pcd.points, dtype=np.float64)
    rgb = None
    if pcd.has_colors():
        rgb = np.asarray(pcd.colors, dtype=np.float64)
        if rgb.max() > 1.0:
            rgb = rgb / 255.0
    return xyz, rgb, None


def load_point_cloud(path: Path) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    """Load xyz (+ optional rgb, intensity) from LAS/LAZ/PLY/PCD/XYZ."""
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)

    magic = _read_magic(path, 4)
    if magic == b"CCB2":
        raise CloudCompareBinError(CCB2_ERROR.format(path=path))

    ext = path.suffix.lower()
    if ext in (".las", ".laz") or magic[:4] == b"LASF":
        return _load_laspy(path)
    if ext in _O3D_EXTENSIONS:
        return _load_open3d(path)
    if ext == ".bin":
        raise CloudCompareBinError(
            f"{path} has .bin extension (likely CloudCompare BIN). "
            + CCB2_ERROR.format(path=path).split("\n", 1)[1]
        )

    # Extension misleading (e.g. box.las that is actually CCB2 already caught above)
    if ext == ".las":
        return _load_laspy(path)

    # Fallback: try laspy then open3d
    try:
        return _load_laspy(path)
    except Exception:
        return _load_open3d(path)


def load_las(path: Path) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    """Backward-compatible alias for load_point_cloud."""
    return load_point_cloud(path)


def save_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray | None = None) -> None:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(np.clip(rgb, 0.0, 1.0))
    path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(path), pcd)


def numpy_to_pcd(xyz: np.ndarray, rgb: np.ndarray | None = None) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(np.clip(rgb, 0.0, 1.0))
    return pcd
