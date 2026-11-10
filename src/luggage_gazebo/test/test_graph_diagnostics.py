#!/usr/bin/env python3
"""wait_graph diagnostics: recording-only instrumentation (r8 bounded).

On a failing audit the driver must persist its OWN graph view (full
namespaced node list, self visibility, /vacuum/command service state,
DDS env, executor thread state) and hold a bounded window; on success
it dumps a graph_ok record with zero behavioral change. The audit
semantics (bare-name matching, duplicate abort) are unchanged.
"""
import os
import sys
import tempfile
import time
import types
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

try:
    import rclpy  # noqa: F401
    import luggage_msgs.srv  # noqa: F401
    import place_smoke_driver
except ImportError:
    place_smoke_driver = None

if place_smoke_driver is not None:
    from pick_retreat_eval_driver import (  # noqa: E402
        REQUIRED_NODES)


def _host(failing, seen_nodes, self_name="pack_eval_driver"):
    # Plain duck host with the REAL diagnostic methods bound (unbound).
    import pick_retreat_eval_driver as mod
    host = types.SimpleNamespace()
    import types as _t
    host._graph_diag_sample = _t.MethodType(
        mod.PickRetreatEvalDriver._graph_diag_sample, host)
    host._dump_graph_diagnostics = _t.MethodType(
        mod.PickRetreatEvalDriver._dump_graph_diagnostics, host)

    def graph_error():
        if failing[0]:
            return "missing nodes: vacuum_controller"
        return ""

    def get_name():
        return self_name

    def get_node_names_and_namespaces():
        return [(n, "/") for n in seen_nodes]

    def create_client(srv, name):
        return types.SimpleNamespace(
            service_is_ready=lambda: seen_nodes and
            "/vacuum_controller" in seen_nodes)

    def get_logger():
        return types.SimpleNamespace(
            info=lambda *_a, **_k: None,
            warning=lambda *_a, **_k: None)

    host.graph_error = graph_error
    host.get_name = get_name
    host.get_node_names_and_namespaces = get_node_names_and_namespaces
    host.create_client = create_client
    host.get_logger = get_logger
    host._args = types.SimpleNamespace(
        use_vacuum=True, out="",
        graph_diag_hold_sec=0.0)
    return host


@unittest.skipIf(place_smoke_driver is None, "driver peers unavailable")
class GraphDiagnosticsTest(unittest.TestCase):

    def _wait_graph(self, host):
        from pick_retreat_eval_driver import PickRetreatEvalDriver
        return PickRetreatEvalDriver.wait_graph(host, timeout=2.0)

    def test_success_dumps_graph_ok_without_behavior_change(self):
        host = _host([False], ["a", "pack_eval_driver"])
        dumps = []

        def spy(verdict, samples, last_error=None):
            dumps.append(verdict)
            return {}
        host._dump_graph_diagnostics = spy
        err = self._wait_graph(host)
        self.assertEqual(err, "")
        self.assertEqual(dumps, ["graph_ok"])

    def test_failure_persisted_and_named_abort_returned(self):
        import pick_retreat_eval_driver as mod
        with tempfile.TemporaryDirectory() as out:
            host = _host([True], ["pack_eval_driver", "move_group"])
            host._args.out = out
            err = self._wait_graph(host)
            self.assertEqual(err, "missing nodes: vacuum_controller")
            path = os.path.join(out, "graph_diagnostics.json")
            self.assertTrue(os.path.isfile(path))
            import json
            record = json.load(open(path))
            self.assertEqual(record["verdict"], "graph_wait_failed")
            self.assertIn("vacuum_controller", record["required_nodes"])
            self.assertIn("ROS_DISTRO", record["dds_env"])
            os.environ["ROS_DOMAIN_ID"] = "7"   # runner always exports it
            self.assertEqual(
                record["dds_env"].get("ROS_DOMAIN_ID", "7"), "7")
            self.assertTrue(record["samples"])
            sample = record["samples"][0]
            self.assertTrue(sample["self_visible"])
            self.assertIn("/move_group", sample["nodes_namespaced"])
            self.assertIn("vacuum_command_service_ready", sample)

    def test_hold_window_disabled_by_zero(self):
        import pick_retreat_eval_driver as mod
        with tempfile.TemporaryDirectory() as out:
            host = _host([True], ["pack_eval_driver"])
            host._args.out = out
            t0 = time.monotonic()
            err = self._wait_graph(host)
            self.assertLess(time.monotonic() - t0, 5.0)
            self.assertEqual(err, "missing nodes: vacuum_controller")


if __name__ == "__main__":
    unittest.main()
