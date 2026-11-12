"""Each visual incarnation reloads identical bytes without touching physics."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from luggage_gazebo.render_assets import isolate_visual_assets


def fixture(tmp_path):
    models=tmp_path/'models';source=models/'sample'/'meshes';source.mkdir(parents=True)
    (source/'shape.dae').write_text('<COLLADA>original textured geometry</COLLADA>')
    (source/'shape.png').write_bytes(b'original texture bytes')
    sdf='<sdf><model name="payload"><static>true</static><link name="body"><collision name="physical"><geometry><box><size>.5 .4 .2</size></box></geometry></collision><visual name="visible"><geometry><mesh><uri>model://sample/meshes/shape.dae</uri></mesh></geometry></visual></link></model></sdf>'
    return models,source,sdf


def test_distinct_render_uris_keep_source_files_and_all_physical_fields(tmp_path):
    models,source,sdf=fixture(tmp_path);results=[]
    for _ in range(2):
        result=isolate_visual_assets(sdf,asset_root=tmp_path/'assets',models_root=models)
        tree=ET.fromstring(result);uri=tree.find('.//visual/geometry/mesh/uri');path=Path(uri.text.removeprefix('file://'))
        assert path.read_bytes()==(source/'shape.dae').read_bytes()
        assert path.with_name('shape.png').read_bytes()==(source/'shape.png').read_bytes()
        results.append(uri.text);uri.text='model://sample/meshes/shape.dae'
        assert ET.tostring(tree)==ET.tostring(ET.fromstring(sdf))
    assert results[0]!=results[1]
    assert isolate_visual_assets(sdf,asset_root='',models_root=models)==sdf


def test_missing_visual_resource_cannot_return_a_create_request(tmp_path):
    models,source,sdf=fixture(tmp_path);(source/'shape.dae').unlink()
    with pytest.raises(ValueError,match='VISUAL_RESOURCE_UNAVAILABLE'):
        isolate_visual_assets(sdf,asset_root=tmp_path/'assets',models_root=models)
    assert not (tmp_path/'assets').exists()
