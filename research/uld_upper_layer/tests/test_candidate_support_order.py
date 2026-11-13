"""Lateral search uses input support, never a known fixture placement."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from candidates import support_ordered_positions
from planning_types import UpperConfig,SurfacePatch


class CandidateSupportOrder(unittest.TestCase):
    def order(self,shift=0.,gap=.04):
        patches=(SurfacePatch('left',(-1,-1+shift,1,-gap/2+shift),1.),
                 SurfacePatch('right',(-1,gap/2+shift,1,1+shift),1.))
        positions=tuple(shift+y for y in (-.4,0.,.4))
        return support_ordered_positions(positions,0.,1.125,np.eye(3),(.55,.4,.25),
                                        patches,UpperConfig.load(),(0,0,0))

    def test_gap_free_support_precedes_global_center_without_dropping_any_position(self):
        order=self.order()
        self.assertEqual(order,(-.4,.4,0.))
        self.assertEqual(set(order),{-.4,0.,.4})

    def test_translated_input_translates_the_order_and_removing_gap_restores_center(self):
        np.testing.assert_allclose(np.array(self.order(shift=2.))-2,self.order())
        self.assertEqual(self.order(gap=0.)[0],0.)


if __name__=='__main__':unittest.main()
