"""Construct actual diagnostic nodes and exercise their registered raw callbacks."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from test_joint_operation_observer import configuration


@pytest.mark.parametrize('authorized,three_box',[(True,False),(True,True),(False,False)])
def test_each_authorized_mode_records_selected_inputs_and_readiness_does_not(tmp_path,monkeypatch,authorized,three_box):
    path = Path(os.environ.get('ELFIN_TEST_OPERATION_DRIVER_SCRIPT',
        Path(__file__).parents[1]/'scripts/joint_operation_sim_driver.py'))
    if os.environ.get('ELFIN_TEST_OPERATION_DRIVER_SCRIPT'):
        assert '/install-production/' in str(path.resolve())
    spec = importlib.util.spec_from_file_location('capture_driver_constructor',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _, geometry, config = configuration(tmp_path)
    (tmp_path/'launch.json').write_text(json.dumps(dict(map_owner=dict(
        geometry_file=str(geometry),config_file=str(config)))))
    subscriptions = []
    original = Node.create_subscription
    def subscribe(self,kind,topic,callback,*args,**kwargs):
        subscriptions.append((topic,callback))
        return original(self,kind,topic,callback,*args,**kwargs)
    monkeypatch.setattr(Node,'create_subscription',subscribe)
    args = SimpleNamespace(evidence=tmp_path/'driver',run_dir=tmp_path,view_poses=None,
        start=authorized,three_box=three_box,failure_hold_s=0.,pickup_recovery_config=None)
    node = None
    rclpy.init(domain_id=178)
    try:
        node = module.OperationSimDriver(args)
        assert (node.pickup_capture is not None) == authorized
        image = Image(height=1,width=2,encoding='16UC1',step=4,data=b'\x01\x00\x02\x00')
        image.header.stamp.sec = 17
        image.header.frame_id = 'measured_optical_frame'
        callbacks = [cb for topic,cb in subscriptions if topic=='/luggage/preprocessed/camera/depth/image']
        for callback in callbacks:
            callback(image)
        assert len(callbacks) == (2 if authorized else 1)
        if authorized:
            receipt = node.pickup_capture.write(tmp_path/'selected',17*10**9,
                required_topics=('/luggage/preprocessed/camera/depth/image',))
            assert receipt['selected_acquisition_complete']
            row = receipt['records'][0]
            raw = gzip.decompress((tmp_path/'selected'/row['file']).read_bytes())
            assert hashlib.sha256(raw).hexdigest() == row['sha256']
            assert deserialize_message(raw, Image) == image
        else:
            assert not node.pickup_capture_active
            assert not (tmp_path/'selected').exists()
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
