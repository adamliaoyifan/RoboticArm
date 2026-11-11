"""Build a source-bound independent release world; no simulator is launched."""
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from scipy.spatial.transform import Rotation

from assets import sha256
from candidates import matrix
from release_fixture import release_carriage_model, _numbers
from release_geometry import release_geometry
from scene import text


def write_release_scene(assets, snapshot, state, config, case, policy,
                        source_scene, vacuum_plugin, output):
    """Place the initial fixture using source geometry, never command a live box.

    All original lower bodies, ULD collision meshes and material remain intact.
    The native attachment is created before the first physics step; callers must
    still verify seal, settled support/contact and attachment before detaching.
    """
    source_scene = Path(source_scene); output = Path(output)
    plugin = Path(vacuum_plugin).resolve(strict=True)
    if output.exists():
        raise ValueError('RELEASE_SCENE_OUTPUT_ALREADY_EXISTS')
    expected = release_geometry(assets, snapshot, state, config,
        case.requested_support_ratio, case.box_yaw_deg, case.repetition)
    if asdict(expected) != asdict(case):
        raise ValueError('RELEASE_CASE_INPUT_CHANGED')
    fixture = json.loads((source_scene / 'fixture.json').read_text())
    if fixture['source_hashes'] != assets['hashes']:
        # Source scene adds collision mesh/material/plugin identities beyond
        # the asset loader. Every shared entry must agree, extras are checked.
        for name, digest in assets['hashes'].items():
            if fixture['source_hashes'].get(name) != digest:
                raise ValueError('RELEASE_ASSETS_CHANGED:' + name)
    for name, digest in fixture['source_hashes'].items():
        path = Path(name) if Path(name).is_absolute() else assets['repo'] / name
        if sha256(path) != digest:
            raise ValueError('RELEASE_SOURCE_SCENE_CHANGED:' + name)
    model, ledger = release_carriage_model(assets, case, policy)
    root = ET.parse(source_scene / 'standalone.sdf').getroot()
    world = root.find('world')
    payload = world.find("model[@name='upper_layer_payload']")
    if payload is None or world.find("model[@name='release_carriage']") is not None:
        raise ValueError('INVALID_RELEASE_SOURCE_WORLD')
    box = assets['world_container'] @ matrix(case.container_T_box)
    pose = [*box[:3, 3], *Rotation.from_matrix(box[:3, :3]).as_euler('xyz')]
    payload.find('pose').text = _numbers(pose)
    world.append(model)
    for old in list(world.findall("plugin[@name='uld_upper::IdealVacuum']")):
        world.remove(old)
    vacuum = ET.SubElement(world, 'plugin', filename=str(plugin), name='uld_upper::IdealVacuum')
    parameters = dict(payload_model='upper_layer_payload', parent_model='release_carriage',
        parent_link='tool', payload_link='body', payload_collision='collision',
        service='/upper_layer/vacuum', state_topic='/upper_layer/vacuum/state',
        initially_attached='true')
    for name, value in parameters.items():
        text(vacuum, name, value)
    for name in ('release_x', 'release_z'):
        text(vacuum, 'joint_name', name)
    for body in fixture['boxes']:
        text(vacuum, 'observed_model', body['name'])
    tool_link = model.find("link[@name='tool']")
    collision_names = [part['sdf_collision'] for part in ledger['parts']]
    for name in collision_names:
        text(vacuum, 'observed_parent_collision', name)
        sensor = ET.SubElement(tool_link, 'sensor', name=name + '_contact', type='contact')
        text(sensor, 'always_on', 'true')
        text(sensor, 'update_rate', 1. / fixture['physics_step_s'])
        contact = ET.SubElement(sensor, 'contact')
        text(contact, 'collision', name)
        text(contact, 'topic', '/upper_layer/release/tool_contacts/' + name)
    output.mkdir(parents=True, exist_ok=False)
    ET.indent(root)
    ET.ElementTree(root).write(output / 'release.sdf', encoding='utf-8', xml_declaration=True)
    shutil.copyfile(source_scene / 'fixture.json', output / 'source_fixture.json')
    manifest = dict(profile='independent_release_fixture', case=asdict(case), tool=ledger,
        tool_collision_names=collision_names,
        fixture_policy=asdict(policy), configuration=asdict(config),
        source_world_sha256=sha256(source_scene / 'standalone.sdf'),
        source_fixture_sha256=sha256(source_scene / 'fixture.json'),
        native_plugin=str(plugin), native_plugin_sha256=sha256(plugin),
        world_sha256=sha256(output / 'release.sdf'),
        below_execution_policy=not case.expected_support.accepted,
        physical_stability='not_evaluated', native_attachment='not_evaluated',
        arm_reachability='not_evaluated', scored_units=0)
    (output / 'release_scene.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest
