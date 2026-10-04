"""The B and I live ROS fixtures cannot share one interpreter."""
import importlib.util
import os


def _load_conftest():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "conftest.py")
    spec = importlib.util.spec_from_file_location(
        "st09_parallel_conftest_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_CONFTEST = _load_conftest()


class _Item(object):
    def __init__(self, names):
        self.fixturenames = names


def test_partition_keeps_valued_options_together():
    options, paths = _CONFTEST.partition_pytest_args(
        ["-q", "-p", "no:cacheprovider", "--tb=short",
         "tests/st09_parallel/B", "tests/st09_parallel/I", "-k", "scope"])
    assert options == [
        "-q", "-p", "no:cacheprovider", "--tb=short", "-k", "scope"]
    assert paths == ["tests/st09_parallel/B", "tests/st09_parallel/I"]


def test_split_only_when_both_live_fixtures_are_collected():
    owners = _CONFTEST._live_ros_owners
    assert owners([
        _Item(["mock_graph"]), _Item(["whole_chain"]), _Item([]),
    ]) == {"B", "I"}
    assert owners([_Item(["mock_graph"])]) == {"B"}
    assert owners([_Item([])]) == set()
