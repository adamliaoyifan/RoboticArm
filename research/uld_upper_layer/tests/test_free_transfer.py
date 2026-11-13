"""Outside reorientation derives its clearance sphere from current tool geometry."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import axis_box
from planning_types import UpperConfig
from free_transfer import outside_reorientation


class FreeTransfer(unittest.TestCase):
    def scene(self,length):
        body=axis_box('tool',[-length,-.1,-.1],[0,.1,0])
        current=np.eye(4);current[0,3]=-.9;current[2,3]=1.7
        return SimpleNamespace(payload=dict(mode='world'),container_world=np.eye(4),
            urdf=SimpleNamespace(tcp=lambda *args:current,path_to=lambda *args:None),
            assets=dict(tool=[body],scene=dict(container=dict(opening=dict(aperture=dict(corners=[[-.745,-1,0],[-.745,1,2]]))))))

    def test_tool_size_moves_reorientation_outside_and_end_is_exact_target(self):
        target=np.eye(4);target[0,3]=-.95;target[2,3]=1.8
        offsets=[]
        for length in [.3,.6]:
            calls=[]
            def connect(scene,q,pose,config,contact):
                calls.append(pose.copy());self.assertFalse(contact)
                return [list(q),[v+.01 for v in q]],dict(reason='connected')
            with patch('free_transfer.cartesian_path',side_effect=connect):
                path,detail=outside_reorientation(self.scene(length),[0.]*6,target,UpperConfig.load())
            self.assertEqual(len(path),5);self.assertEqual(len(calls),4)
            np.testing.assert_allclose(calls[-1],target)
            offsets.append(detail['outside_x_container'])
        self.assertLess(offsets[1],offsets[0])

    def test_collision_or_attached_payload_never_returns_a_connected_route(self):
        scene=self.scene(.3)
        with patch('free_transfer.cartesian_path',return_value=(None,dict(reason='SELF_COLLISION'))):
            path,detail=outside_reorientation(scene,[0.]*6,np.eye(4),UpperConfig.load())
        self.assertIsNone(path);self.assertEqual(detail['legs'][0]['diagnostic']['reason'],'SELF_COLLISION')
        scene.payload['mode']='attached'
        with patch('free_transfer.cartesian_path') as connect:
            path,detail=outside_reorientation(scene,[0.]*6,np.eye(4),UpperConfig.load())
            self.assertIsNone(path);connect.assert_not_called()


if __name__=='__main__':unittest.main()
