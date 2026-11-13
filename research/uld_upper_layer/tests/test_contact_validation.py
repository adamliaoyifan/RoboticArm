"""Dense native manifolds: every point matters, including in quiet fast mode."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
import sys
import unittest
import numpy as np
from shapely.geometry import box
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contact_validation import validate_declared_contacts
from physical_observation import ObservedContact
from planning_types import UpperConfig
from execution_policy import ExecutionPolicy


class DenseContactContracts(unittest.TestCase):
    def check(self,contacts,record=True):
        sample=SimpleNamespace(lower=(),contacts=contacts,
            container_T_box=tuple(np.eye(4).ravel()),container_T_tool=tuple(np.eye(4).ravel()))
        geometry=SimpleNamespace(contact_mode=lambda mode:mode,
            contact_polygon=lambda mode:box(-.09,-.09,.09,.09),
            push_geometry=SimpleNamespace(tool_T_push_surface=tuple(np.eye(4).ravel())))
        return validate_declared_contacts(sample,(1,1,1),UpperConfig.load(),ExecutionPolicy.load(),
            True,'push_plate','panel',geometry,record_intended=record)

    def test_late_manifold_point_is_checked_even_when_diagnostic_output_is_disabled(self):
        good=ObservedContact('panel',(0,0,0),(0,0,1),.000001)
        for bad in (replace(good,point_container=(.2,0,0)),
                    replace(good,point_container=(0,0,.002)),
                    replace(good,normal_toward_payload_container=(0,0,-1)),
                    replace(good,depth_m=.001)):
            for record in (False,True):
                failure,_=self.check((good,)*80+(bad,),record)
                self.assertEqual(failure['reason'],'NATIVE_CONTACT_OUTSIDE_DECLARED_PANEL_PATCH')

    def test_quiet_mode_changes_only_diagnostics_and_keeps_boundary_contacts_valid(self):
        good=ObservedContact('panel',(.09,0,0),(0,0,1),.000001)
        failure,intended=self.check((good,)*80,True)
        self.assertIsNone(failure);self.assertEqual(len(intended),80)
        failure,intended=self.check((good,)*80,False)
        self.assertIsNone(failure);self.assertEqual(intended,[])


if __name__=='__main__':unittest.main()
