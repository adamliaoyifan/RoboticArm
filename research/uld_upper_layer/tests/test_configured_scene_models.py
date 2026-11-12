"""The simulator and planner instantiate the same configured static geometry."""
from pathlib import Path
import sys,tempfile,unittest
import xml.etree.ElementTree as ET
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[3]/'src/luggage_description')]
from luggage_description.configured_scene_models import configured_model,write_configured_model


class ConfiguredModels(unittest.TestCase):
    def test_size_and_height_follow_config_with_material_and_source_unchanged(self):
        repo=Path(__file__).resolve().parents[3]
        for name,key in [('robot_pedestal','pedestal'),('pickup_platform','pickup_platform')]:
            path=repo/f'src/luggage_gazebo/models/{name}/model.sdf';before=path.read_bytes()
            for size in ((.74,.5,.86),(.8,.9,.7)):
                scene={key:dict(size=size)}
                model=configured_model(path,scene,name)
                generated=Path(write_configured_model(path,scene,name))
                try:
                    loaded=ET.parse(generated).getroot().find('model')
                    for kind in ('visual','collision'):
                        part=model.find('link/'+kind);other=loaded.find('link/'+kind)
                        np.testing.assert_allclose([float(v) for v in part.find('geometry/box/size').text.split()],size)
                        self.assertEqual(part.find('pose').text,other.find('pose').text)
                        self.assertAlmostEqual(float(part.find('pose').text.split()[2]),size[2]/2)
                    self.assertIsNotNone(model.find('link/visual/material'))
                    self.assertEqual(path.read_bytes(),before)
                finally:generated.unlink();generated.parent.rmdir()

    def test_invalid_config_does_not_fall_back_to_source_geometry(self):
        repo=Path(__file__).resolve().parents[3]
        path=repo/'src/luggage_gazebo/models/robot_pedestal/model.sdf'
        for size in [(0,.5,.86),(.74,float('nan'),.86),(.74,.5)]:
            with self.assertRaises(ValueError):configured_model(path,{'pedestal':{'size':size}},'robot_pedestal')


if __name__=='__main__':unittest.main()
