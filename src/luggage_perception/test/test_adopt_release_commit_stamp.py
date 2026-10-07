"""Commit-clock labelling in CargoVolumeMapperNode._adopt_release (gate on).

Offline: the method is called on a stand-in object, no ROS graph. The
contract: with ``require_confirmed_map`` a commit records a pending change,
labels a map that has no stamp yet with the commit clock, and never
refreshes an existing sensor stamp.
"""
import importlib.util
import os
import unittest
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE = os.path.normpath(os.path.join(
    _HERE, "..", "scripts", "cargo_volume_mapper_node.py"))


def _load():
    try:
        spec = importlib.util.spec_from_file_location("mapper_node_t", _NODE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except ImportError as exc:
        raise unittest.SkipTest("mapper node imports unavailable: %s" % exc)
    return module


class _Pending(object):
    def __init__(self):
        self.recorded = []

    def record(self, event, now_ns, footprint=None):
        self.recorded.append((event, now_ns, footprint))


def _fake(module, *, stamp, gate=True, ledger=1, source="static_empty"):
    return SimpleNamespace(
        _mapper=SimpleNamespace(commit_ledger=lambda: [object()] * ledger),
        _map_source=source,
        _require_confirmed_map=gate,
        _last_acquisition_ns=stamp,
        _pending=_Pending(),
        _node_clock_ns=lambda: 7_000_000_000)


class TestAdoptReleaseCommitStamp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mod = _load()
        cls.adopt = staticmethod(
            cls.mod.CargoVolumeMapperNode._adopt_release)

    def test_gate_on_first_commit_labels_unstamped_map_and_stays_pending(self):
        fake = _fake(self.mod, stamp=0)
        self.adopt(fake, True, "footprint")
        self.assertEqual(fake._last_acquisition_ns, 7_000_000_000)
        self.assertEqual(fake._map_source, self.mod.MAP_RELEASE_INTEGRATED)
        self.assertEqual(
            fake._pending.recorded, [("box_placed", 7_000_000_000, "footprint")])

    def test_gate_on_commit_never_refreshes_an_existing_sensor_stamp(self):
        fake = _fake(self.mod, stamp=3_000_000_000, source="measured")
        self.adopt(fake, True, "footprint")
        self.assertEqual(fake._last_acquisition_ns, 3_000_000_000)
        self.assertEqual(len(fake._pending.recorded), 1)

    def test_gate_on_duplicate_commit_records_nothing_and_keeps_stamp(self):
        fake = _fake(self.mod, stamp=0)
        self.adopt(fake, False, "footprint")
        self.assertEqual(fake._last_acquisition_ns, 0)
        self.assertEqual(fake._pending.recorded, [])


if __name__ == "__main__":
    unittest.main()
