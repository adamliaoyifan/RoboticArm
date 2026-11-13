"""Fail closed on a spawned model incapable of panel/payload contact."""
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sim_model_contract import panel_contact_identity


class SpawnedContact(unittest.TestCase):
    def check(self,collision):
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'robot.sdf'
            file.write_text('<sdf><model name="robot"><link name="wrist">'+collision+'</link></model></sdf>')
            return panel_contact_identity(file,'robot')

    def collision(self,mask=1,mesh='p04.stl'):
        return ('<collision name="panel"><geometry><mesh><uri>model://tool/'+mesh+
                '</uri></mesh></geometry><surface><contact><collide_bitmask>'+str(mask)+
                '</collide_bitmask></contact></surface></collision>')

    def test_correct_collision_identity_and_mask(self):
        result=self.check(self.collision())
        self.assertEqual(result['collision_name'],'robot::wrist::panel')
        self.assertTrue(result['collision_mask']&result['payload_mask'])

    def test_visual_or_other_mesh_is_not_a_contact_collision(self):
        with self.assertRaisesRegex(ValueError,'EXPECTED_ONE'):
            self.check(self.collision(mesh='p01.stl'))

    def test_disabled_mask_cannot_pass_spawn_gate(self):
        with self.assertRaisesRegex(ValueError,'MASK_DISABLED'):self.check(self.collision(mask=0))

    def test_ambiguous_collision_identity_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'EXPECTED_ONE'):self.check(self.collision()*2)


if __name__=='__main__':unittest.main()
