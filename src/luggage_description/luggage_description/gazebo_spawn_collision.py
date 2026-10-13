"""Gazebo-only collision edits. MoveIt keeps the full URDF.

gz-physics 5 has no named pair filter. The dartsim engine ignores a contact
when the bitwise AND of the two shapes' masks is zero
(CollisionFilterMaskFeature). The SDF default mask is 0xFF. p04 is set to
0x01 alone, so it still hits every default shape. Only the carried box uses
0xFE (default without bit 0). A released or other box uses 0xFF and collides
with p04 again. No other collision's mask is changed.
"""
import xml.etree.ElementTree as ET

P04_MASK = 0x01
BOX_FULL_MASK = 0xFF
BOX_CARRY_MASK = 0xFE


def masks_collide(mask_a, mask_b):
    return (int(mask_a) & int(mask_b)) != 0


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


def set_collision_bitmask_for_mesh(sdf_xml, mesh_token, mask):
    """Set ``collide_bitmask`` on collisions whose mesh uri contains ``mesh_token``.

    Other collisions are left unchanged. Returns ``(xml, changed_count)``.
    """
    root = ET.fromstring(sdf_xml)
    changed = 0
    for collision in root.iter("collision"):
        uri = ""
        mesh = collision.find("geometry/mesh")
        if mesh is not None and mesh.find("uri") is not None:
            uri = mesh.find("uri").text or ""
        if mesh_token not in uri:
            continue
        surface = collision.find("surface")
        if surface is None:
            surface = ET.SubElement(collision, "surface")
        contact = surface.find("contact")
        if contact is None:
            contact = ET.SubElement(surface, "contact")
        bit = contact.find("collide_bitmask")
        if bit is None:
            bit = ET.SubElement(contact, "collide_bitmask")
        bit.text = str(int(mask))
        changed += 1
    return ET.tostring(root, encoding="unicode"), changed


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
