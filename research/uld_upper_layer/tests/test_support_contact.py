"""Distinguish a bottom/top support contact from a bottom-corner side jam."""
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import pose
from support_contact import support_contact_failure


class SupportContactContracts(unittest.TestCase):
    def check(self,normal=(0,0,1),point=(.5,0,1),inverted=False,signed=True):
        rotation=Rotation.from_euler('x',180,degrees=True).as_matrix() if inverted else np.eye(3)
        return support_contact_failure(pose([0,0,1.125],rotation),(1,1,.25),
            pose([0,0,.5]),[[-.5,-.5,-.5],[.5,.5,.5]],point,normal,
            .001,.0005,1.,.000001,normal_toward_box=signed)

    def test_same_corner_distinguishes_support_from_vertical_side_jam(self):
        for inverted in (False,True):
            self.assertIsNone(self.check(inverted=inverted))
            self.assertEqual(self.check((-1,0,0),inverted=inverted)['reason'],
                             'SUPPORT_CONTACT_NORMAL_NOT_TOP_BOTTOM')

    def test_native_direction_and_fcl_order_have_explicit_semantics(self):
        self.assertIsNotNone(self.check((0,0,-1)))
        self.assertIsNone(self.check((0,0,-1),signed=False))

    def test_geometric_face_and_normal_are_both_required(self):
        self.assertEqual(self.check(point=(.5,0,.995))['reason'],
                         'SUPPORT_CONTACT_NOT_ON_BOX_BOTTOM')
        self.assertEqual(self.check(point=(.51,0,1))['reason'],
                         'SUPPORT_CONTACT_NOT_ON_SUPPORT_TOP')


if __name__=='__main__':unittest.main()
