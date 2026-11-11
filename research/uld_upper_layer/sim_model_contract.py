"""Read the actual spawned SDF contact identity; never infer it from visuals."""
from pathlib import Path
import xml.etree.ElementTree as ET
from assets import sha256


def panel_contact_identity(sdf_file,model_name,mesh_basename='p04.stl',payload_mask=0xFF):
    root=ET.parse(sdf_file).getroot();found=[]
    for link in root.iter('link'):
        for collision in link.findall('collision'):
            uri=collision.findtext('geometry/mesh/uri','')
            if Path(uri).name!=mesh_basename:continue
            mask=int(collision.findtext('surface/contact/collide_bitmask','255'),0)
            found.append(dict(collision_name=f"{model_name}::{link.get('name')}::{collision.get('name')}",
                mesh_uri=uri,collision_mask=mask,payload_mask=payload_mask))
    if len(found)!=1:raise ValueError('EXPECTED_ONE_PANEL_CONTACT_COLLISION:'+str(len(found)))
    if not found[0]['collision_mask']&payload_mask:
        raise ValueError('PANEL_PAYLOAD_MASK_DISABLED')
    return dict(**found[0],spawned_sdf_sha256=sha256(sdf_file),
                scope='spawned_definition_contact_still_requires_physics_observation')
