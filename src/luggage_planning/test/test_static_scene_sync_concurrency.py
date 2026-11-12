"""A reentrant timer must leave executor workers available for RPC replies."""
from concurrent.futures import ThreadPoolExecutor
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace


def test_parallel_sync_calls_do_not_starve_the_completion_callback():
    path = Path(__file__).parents[1] / 'scripts/scene_manager_node.py'
    spec = importlib.util.spec_from_file_location('static_scene_sync_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entered, reply = threading.Event(), threading.Event()
    def waiting_config():
        entered.set()
        # Represents the first worker waiting for a service reply. The reply
        # itself must run on this same four-worker executor, as in the node.
        assert reply.wait(2.), 'completion callback was starved'
        raise ValueError('end synthetic sync without scene writes')
    owner = SimpleNamespace(_sync_lock=threading.Lock(), _config=waiting_config)
    with ThreadPoolExecutor(max_workers=4) as executor:
        first = executor.submit(module.SceneManagerNode._sync_static, owner)
        assert entered.wait(.5)
        repeats = [executor.submit(module.SceneManagerNode._sync_static, owner) for _ in range(3)]
        completion = executor.submit(reply.set)
        try:
            completion.result(timeout=.5)
            assert all(f.result(timeout=.5) == (False, 'SYNC_STATIC_BUSY') for f in repeats)
            assert first.result(timeout=.5)[0] is False
        finally:
            reply.set()
