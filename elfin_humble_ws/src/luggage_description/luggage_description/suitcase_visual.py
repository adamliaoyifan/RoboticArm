"""Suitcase visuals for Gazebo.

Unit-AABB sources live in ``luggage_gazebo/models/suitcase_loafbrr`` and
``suitcase_vintage`` (lying Z-up, 1x1x1, centered). Pickup never spawns those
two: Fortress ogre2 caches GPU meshes by URI, so sharing one STL with SDF
``<scale>`` makes later boxes render at the first scale.

Pickup mesh assets are six pre-scaled models (each visual x small/medium/large)
with visual and collision using the same STL at scale 1 1 1. Spawn only
``gz create``s the matching ``model://``. Box visual stays a primitive box of
the catalog size (production default).
"""

from __future__ import division

import json
import os
import struct

# Gazebo model:// names (folders under luggage_gazebo/models).
VISUAL_LOAFBRR = "suitcase_loafbrr"
VISUAL_VINTAGE = "suitcase_vintage"
VISUAL_IDS = (VISUAL_LOAFBRR, VISUAL_VINTAGE)

# Catalog ``model`` (size slot) -> which mesh family to show.
DEFAULT_MODEL_VISUAL = {
    "suitcase_carryon": VISUAL_LOAFBRR,
    "suitcase_standard": VISUAL_VINTAGE,
    "suitcase_large": VISUAL_LOAFBRR,
}

# Gazebo Fortress ogre2 crashes on runtime OBJ import (GUI + camera render
# threads both CreateMesh → HardwareBuffer already locked). STL matches the
# container assets and loads safely. Color is SDF material, not a texture.
MESH_FILE = "meshes/suitcase.stl"
# Legacy path for tests that still exercise write_scaled_stl.
SCALED_MESH_DIR = "/tmp/luggage_gz_scaled_meshes"

VISUAL_DIFFUSE = {
    VISUAL_LOAFBRR: ("0.12 0.12 0.15 1", "0.20 0.20 0.24 1"),
    VISUAL_VINTAGE: ("0.32 0.24 0.14 1", "0.48 0.36 0.20 1"),
}

# Discrete pickup sizes. Spawn picks one of these; it does not sample the
# continuous envelope. ``catalog_id`` matches box_catalog.yaml.example.
SIZE_TIERS = (
    ("small", (0.55, 0.40, 0.25), 8.0, "carryon"),
    ("medium", (0.70, 0.45, 0.28), 15.0, "standard"),
    ("large", (0.80, 0.50, 0.32), 23.0, "large"),
)

SIZED_MANIFEST_NAME = "sized_suitcases.json"


def visual_id_for_entry(entry):
    """Return a VISUAL_* id for a catalog entry dict."""
    explicit = str(entry.get("visual") or "").strip()
    if explicit in VISUAL_IDS:
        return explicit
    model = str(entry.get("model") or "suitcase_standard")
    return DEFAULT_MODEL_VISUAL.get(model, VISUAL_LOAFBRR)


def mesh_uri(visual_id):
    visual_id = visual_id if visual_id in VISUAL_IDS else VISUAL_LOAFBRR
    return "model://%s/%s" % (visual_id, MESH_FILE)


def sized_model_name(visual_id, tier):
    """Gazebo model folder for a pre-scaled visual x size-tier pair."""
    visual_id = visual_id if visual_id in VISUAL_IDS else VISUAL_LOAFBRR
    return "%s_%s" % (visual_id, str(tier).strip().lower())


def sized_mesh_uri(visual_id, tier):
    return "model://%s/%s" % (sized_model_name(visual_id, tier), MESH_FILE)


def size_tier_name(size, atol=1e-3):
    """Return small/medium/large if *size* matches a catalog tier, else None."""
    if size is None or len(size) < 3:
        return None
    for name, spec, _mass, _catalog_id in SIZE_TIERS:
        if all(abs(float(size[i]) - float(spec[i])) <= atol for i in range(3)):
            return name
    return None


def source_stl_path(visual_id, models_root):
    """Absolute path to the unit AABB STL under a Gazebo models directory."""
    visual_id = visual_id if visual_id in VISUAL_IDS else VISUAL_LOAFBRR
    return os.path.join(models_root, visual_id, MESH_FILE)


def sized_stl_path(visual_id, tier, models_root):
    return os.path.join(
        models_root, sized_model_name(visual_id, tier), MESH_FILE)


def sized_suitcases_manifest_path(models_root):
    return os.path.join(models_root, SIZED_MANIFEST_NAME)


def scaled_stl_path(model_name, dest_dir=SCALED_MESH_DIR):
    """Unique per-spawn STL path. Kept for unit tests of write_scaled_stl."""
    return os.path.join(dest_dir, "%s.stl" % model_name)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def iter_stl_vertices(path):
    """Yield (x, y, z) vertices from a binary STL."""
    with open(path, "rb") as handle:
        header = handle.read(80)
        if len(header) < 80:
            raise IOError("truncated STL header: %s" % path)
        n_raw = handle.read(4)
        if len(n_raw) < 4:
            raise IOError("truncated STL triangle count: %s" % path)
        count = struct.unpack("<I", n_raw)[0]
        for _ in range(count):
            rec = handle.read(50)
            if len(rec) < 50:
                raise IOError("truncated STL triangles: %s" % path)
            nums = struct.unpack("<12fH", rec)
            yield nums[3:6]
            yield nums[6:9]
            yield nums[9:12]


def mesh_top_footprint(path, z_band_frac=0.01, z_band_min=0.005):
    """Axis-aligned box of the top-face vertices plus full mesh height.

    Width/depth are the XY span of vertices within ``max(frac*h, min)`` of
    zmax (what a top-down camera sees). Height is the full vertex AABB Z.
    """
    xs, ys, zs = [], [], []
    for x, y, z in iter_stl_vertices(path):
        xs.append(x)
        ys.append(y)
        zs.append(z)
    if not zs:
        raise ValueError("empty STL: %s" % path)
    zmax, zmin = max(zs), min(zs)
    height = zmax - zmin
    band = max(float(z_band_frac) * height, float(z_band_min))
    top_x = [x for x, z in zip(xs, zs) if z >= zmax - band]
    top_y = [y for y, z in zip(ys, zs) if z >= zmax - band]
    if not top_x:
        return [max(xs) - min(xs), max(ys) - min(ys), height]
    return [max(top_x) - min(top_x), max(top_y) - min(top_y), height]


def write_scaled_stl(src_path, dest_path, scale):
    """Copy a binary STL with vertices multiplied by ``scale`` (W, D, H).

    Used once when generating the six pickup models. Spawn does not call this.
    """
    sx, sy, sz = (float(scale[0]), float(scale[1]), float(scale[2]))
    with open(src_path, "rb") as handle:
        header = handle.read(80)
        if len(header) < 80:
            raise IOError("truncated STL header: %s" % src_path)
        n_raw = handle.read(4)
        if len(n_raw) < 4:
            raise IOError("truncated STL triangle count: %s" % src_path)
        count = struct.unpack("<I", n_raw)[0]
        payload = handle.read(50 * count)
    if len(payload) < 50 * count:
        raise IOError("truncated STL triangles: %s" % src_path)
    chunks = [os.path.basename(dest_path).encode("ascii", "replace")[:80].ljust(80, b" "),
              struct.pack("<I", count)]
    for i in range(count):
        rec = payload[i * 50:(i + 1) * 50]
        nums = struct.unpack("<12f", rec[:48])
        v1 = (nums[3] * sx, nums[4] * sy, nums[5] * sz)
        v2 = (nums[6] * sx, nums[7] * sy, nums[8] * sz)
        v3 = (nums[9] * sx, nums[10] * sy, nums[11] * sz)
        e1 = (v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2])
        e2 = (v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2])
        nx, ny, nz = _cross(e1, e2)
        length = (nx * nx + ny * ny + nz * nz) ** 0.5
        if length > 1e-12:
            nx, ny, nz = nx / length, ny / length, nz / length
        else:
            nx, ny, nz = 0.0, 0.0, 1.0
        chunks.append(struct.pack(
            "<12f", nx, ny, nz, v1[0], v1[1], v1[2],
            v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]))
        chunks.append(rec[48:50])
    dest_dir = os.path.dirname(dest_path)
    if dest_dir:
        os.makedirs(dest_dir, exist_ok=True)
    with open(dest_path, "wb") as handle:
        handle.write(b"".join(chunks))
    return dest_path


def file_uri(path):
    """Absolute path -> file:// URI for gz mesh loading."""
    return "file://%s" % os.path.abspath(path)


def cuboid_inertia(size, mass_kg):
    """Solid-cuboid inertia about the AABB center (ixx, iyy, izz)."""
    length, width, height = [float(v) for v in size]
    mass = float(mass_kg)
    return (
        mass * (width * width + height * height) / 12.0,
        mass * (length * length + height * height) / 12.0,
        mass * (length * length + width * width) / 12.0,
    )


def pickup_box_pose(source_xyz, source_rpy, size, yaw_offset=0.0, xy_jitter=(0.0, 0.0)):
    """World pose of a lying box whose AABB is centered on the link.

    ``source_xyz`` is the platform-top pickup point (box bottom). Link origin
    is the geometric center, so ``z = source_z + height/2``. Roll/pitch come
    from the source; yaw is source yaw plus ``yaw_offset``.
    """
    height = float(size[2])
    xyz = (
        float(source_xyz[0]) + float(xy_jitter[0]),
        float(source_xyz[1]) + float(xy_jitter[1]),
        float(source_xyz[2]) + height * 0.5,
    )
    rpy = (
        float(source_rpy[0]),
        float(source_rpy[1]),
        float(source_rpy[2]) + float(yaw_offset),
    )
    return xyz, rpy


def _mesh_geometry_xml(uri, scale):
    return (
        "          <mesh>\n"
        "            <uri>%s</uri>\n"
        "            <scale>%.6f %.6f %.6f</scale>\n"
        "          </mesh>"
        % (uri, float(scale[0]), float(scale[1]), float(scale[2]))
    )


def pickup_visual_sdf(model_name, size, mass_kg, visual_id, visual_kind="box",
                      models_root=None, scaled_dir=SCALED_MESH_DIR):
    """SDF for a pickup spawn. Default visual is a primitive box.

    ``visual_kind=mesh`` references a pre-scaled ``model://`` URI. It does not
    write a new STL. *models_root* and *scaled_dir* are unused for mesh spawn
    (kept so callers do not break).
    """
    del models_root, scaled_dir
    kind = str(visual_kind or "box").strip().lower()
    if kind != "mesh":
        return suitcase_sdf(
            model_name, size, mass_kg, visual_id, visual_kind="box")
    tier = size_tier_name(size)
    if tier is None:
        raise ValueError(
            "mesh pickup visual requires a catalog size "
            "(small/medium/large), got %s" % (list(size),))
    return suitcase_sdf(
        model_name, size, mass_kg, visual_id,
        mesh_uri_override=sized_mesh_uri(visual_id, tier),
        mesh_already_scaled=True,
        visual_kind="mesh",
    )


def suitcase_sdf(model_name, size, mass_kg, visual_id, mesh_uri_override=None,
                 mesh_already_scaled=False, visual_kind="mesh"):
    """SDF: visual and collision share one geometry at the link origin.

    *visual_kind* ``box`` uses a primitive box for both. ``mesh`` uses the
    suitcase STL for both (same URI and scale). *mesh_already_scaled* sets
    mesh ``<scale>`` to 1 1 1.
    """
    length, width, height = [float(v) for v in size]
    mass = float(mass_kg)
    ixx, iyy, izz = cuboid_inertia(size, mass)
    ambient, diffuse = VISUAL_DIFFUSE.get(
        visual_id, VISUAL_DIFFUSE[VISUAL_LOAFBRR])
    kind = str(visual_kind or "mesh").strip().lower()
    if kind == "box":
        geometry = (
            "          <box><size>%.6f %.6f %.6f</size></box>"
            % (length, width, height))
        comment_kind = "box"
        collision_geom = geometry
    else:
        uri = mesh_uri_override if mesh_uri_override else mesh_uri(visual_id)
        mesh_scale = (1.0, 1.0, 1.0) if mesh_already_scaled else (
            length, width, height)
        geometry = _mesh_geometry_xml(uri, mesh_scale)
        comment_kind = visual_id
        collision_geom = geometry
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <!-- Suitcase %s visual+collision %.3f x %.3f x %.3f m, %.2f kg -->
  <model name="%s">
    <link name="suitcase_link">
      <inertial>
        <mass>%.4f</mass>
        <inertia>
          <ixx>%.6f</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>%.6f</iyy><iyz>0</iyz>
          <izz>%.6f</izz>
        </inertia>
      </inertial>
      <visual name="%s_visual">
        <geometry>
%s
        </geometry>
        <material>
          <ambient>%s</ambient>
          <diffuse>%s</diffuse>
        </material>
      </visual>
      <collision name="body_collision">
        <geometry>
%s
        </geometry>
        <surface>
          <friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
""" % (
        comment_kind, length, width, height, mass, model_name,
        mass, ixx, iyy, izz,
        model_name,
        geometry,
        ambient, diffuse,
        collision_geom,
    )


def _model_config_xml(name, description):
    return """<?xml version="1.0"?>
<model>
  <name>%s</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>RobotArm</name></author>
  <description>
    %s
  </description>
</model>
""" % (name, description)


def write_sized_suitcase_models(models_root):
    """Write the six pre-scaled pickup models and a measure_size manifest.

    Spawn must not call this. Run from the generator script when assets change.
    """
    records = {}
    for visual_id in VISUAL_IDS:
        src = source_stl_path(visual_id, models_root)
        if not os.path.isfile(src):
            raise IOError("missing unit STL: %s" % src)
        for tier, size, mass_kg, catalog_id in SIZE_TIERS:
            name = sized_model_name(visual_id, tier)
            dest_stl = sized_stl_path(visual_id, tier, models_root)
            write_scaled_stl(src, dest_stl, size)
            measure = mesh_top_footprint(dest_stl)
            sdf = suitcase_sdf(
                name, size, mass_kg, visual_id,
                mesh_uri_override=sized_mesh_uri(visual_id, tier),
                mesh_already_scaled=True,
                visual_kind="mesh",
            )
            model_dir = os.path.join(models_root, name)
            os.makedirs(model_dir, exist_ok=True)
            with open(os.path.join(model_dir, "model.sdf"), "w") as handle:
                handle.write(sdf)
            with open(os.path.join(model_dir, "model.config"), "w") as handle:
                handle.write(_model_config_xml(
                    name,
                    "Pre-scaled %s %s suitcase. Visual and collision share "
                    "this STL (scale 1 1 1)." % (visual_id, tier),
                ))
            records[name] = {
                "visual_id": visual_id,
                "tier": tier,
                "catalog_id": catalog_id,
                "size": [float(v) for v in size],
                "measure_size": [float(v) for v in measure],
                "mass_kg": float(mass_kg),
                "uri": sized_mesh_uri(visual_id, tier),
            }
    manifest_path = sized_suitcases_manifest_path(models_root)
    with open(manifest_path, "w") as handle:
        json.dump(records, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return records


def load_sized_suitcases_manifest(models_root):
    """Return the baked measure_size map, or {} if the file is missing."""
    path = sized_suitcases_manifest_path(models_root)
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as handle:
        data = json.load(handle)
    return data if isinstance(data, dict) else {}
