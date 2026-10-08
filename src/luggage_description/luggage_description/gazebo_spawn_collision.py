"""Edit only the URDF file Gazebo spawns. MoveIt keeps the full model."""
import xml.etree.ElementTree as ET


def strip_link_collision_mesh(urdf_xml, link_name, mesh_token):
    """Remove ``<collision>`` elements on ``link_name`` whose mesh filename
    contains ``mesh_token``. Visuals stay. Returns ``(xml, removed_count)``.
    """
    root = ET.fromstring(urdf_xml)
    removed = 0
    for link in root.findall("link"):
        if link.get("name") != link_name:
            continue
        for collision in list(link.findall("collision")):
            mesh = collision.find("geometry/mesh")
            filename = mesh.get("filename") if mesh is not None else ""
            if mesh_token and mesh_token in (filename or ""):
                link.remove(collision)
                removed += 1
    return ET.tostring(root, encoding="unicode"), removed


def panel_collision_absent(urdf_path, mesh_token):
    """The spawned file lost that one collision and kept the others."""
    try:
        root = ET.parse(urdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        return False, str(exc)
    link = next((item for item in root.findall("link")
                 if item.get("name") == "suction_panel"), None)
    if link is None:
        return False, "no suction_panel link"
    collisions = link.findall("collision")
    visuals = link.findall("visual")
    hit = []
    for collision in collisions:
        mesh = collision.find("geometry/mesh")
        filename = mesh.get("filename") if mesh is not None else ""
        if mesh_token in (filename or ""):
            hit.append(filename)
    visual_kept = any(
        mesh_token in ((v.find("geometry/mesh").get("filename")
                        if v.find("geometry/mesh") is not None else "") or "")
        for v in visuals)
    ok = (not hit) and len(collisions) >= 1 and visual_kept
    return ok, {
        "mesh_token": mesh_token,
        "collision_meshes_still_present": hit,
        "suction_collisions_left": len(collisions),
        "visual_kept": visual_kept,
    }
