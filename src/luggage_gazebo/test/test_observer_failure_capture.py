"""Replay retained blocks and exercise time/memory/disk bounds."""
import gzip
import importlib.util
import json
from pathlib import Path

path = Path(__file__).parents[1]/'luggage_gazebo/observer_failure_capture.py'
spec = importlib.util.spec_from_file_location('capture_under_test',path)
module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_pretrigger_window_static_transforms_and_live_replay(tmp_path):
    now = [0.]
    capture = module.ObserverFailureCapture(tmp_path/'capture',window_s=10.,clock=lambda:now[0])
    capture.add('/tf_static','TF',b'camera',persistent='camera')
    capture.add('/tf_static','TF',b'lidar',persistent='lidar')
    capture.add('/points','PC',b'old')
    now[0] = 12.
    capture.add('/points','PC',b'new')
    capture.trigger()
    capture.add('/receipt','Receipt',b'current')
    capture.close()
    capture.add('/receipt','Receipt',b'after_close')
    rows = [json.loads(line) for line in (capture.folder/'index.jsonl').read_text().splitlines()]
    data = (capture.folder/'cdr.gzblocks').read_bytes()
    replay = [gzip.decompress(data[r['offset']:r['offset']+r['stored_bytes']]) for r in rows]
    assert replay == [b'camera',b'lidar',b'new',b'current']
    assert json.loads((capture.folder/'summary.json').read_text())['complete']


def test_caps_preserve_bounded_recent_data_and_report_disk_gaps(tmp_path):
    capture = module.ObserverFailureCapture(tmp_path/'capture',memory_limit=8,disk_limit=30)
    capture.add('/points','PC',b'12345678')
    capture.add('/points','PC',b'recent')
    assert capture.memory_bytes == 6
    capture.trigger()
    capture.add('/receipt','Receipt',b'exceeds remaining disk cap')
    capture.close()
    summary = json.loads((capture.folder/'summary.json').read_text())
    assert summary['stored_bytes'] <= 30
    assert not summary['complete'] and summary['dropped_records'] == 1
