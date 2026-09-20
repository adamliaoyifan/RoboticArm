#!/usr/bin/env python3
"""Residual-graph detection must be scoped to this ROS domain.

Another agent's stack on a different `ROS_DOMAIN_ID` shares the machine but
not the ROS graph. Counting it as a residual aborts a run that has a clean
graph, which is what happened to the first pack-to-full attempt.
"""

import importlib.util
import os
import subprocess
import sys
import time
import unittest
import uuid

_HERE = os.path.dirname(os.path.abspath(__file__))
_DRIVER = os.path.join(_HERE, "..", "scripts", "pick_retreat_eval_driver.py")


def _load_driver_module():
    spec = importlib.util.spec_from_file_location(
        "pick_retreat_eval_driver", os.path.normpath(_DRIVER))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestGraphDomainScope(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            cls.driver = _load_driver_module()
        except ImportError as exc:
            raise unittest.SkipTest("eval driver deps unavailable: %s" % exc)

    def setUp(self):
        self.tag = "elfin_domain_probe_%s" % uuid.uuid4().hex[:8]
        self.procs = []
        self.addCleanup(self._stop_probes)

    def _stop_probes(self):
        for proc in self.procs:
            proc.terminate()
        for proc in self.procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _spawn_probe(self, domain):
        """A real process carrying the tag in argv and a chosen ROS domain."""
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = domain
        proc = subprocess.Popen(
            [sys.executable, "-c",
             "import time,sys; sys.argv; time.sleep(30)", self.tag],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.procs.append(proc)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self.driver.ros_domain_of_pid(proc.pid) == domain:
                return proc
            time.sleep(0.05)
        self.fail("probe process did not start with ROS_DOMAIN_ID=%s" % domain)

    def test_reads_domain_from_a_live_process(self):
        proc = self._spawn_probe("41")
        self.assertEqual(self.driver.ros_domain_of_pid(proc.pid), "41")

    def test_unset_domain_reads_as_zero(self):
        env = dict(os.environ)
        env.pop("ROS_DOMAIN_ID", None)
        proc = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.procs.append(proc)
        self.assertEqual(self.driver.ros_domain_of_pid(proc.pid), "0")

    def test_other_domain_processes_are_not_residuals(self):
        self._spawn_probe("113")
        self._spawn_probe("113")
        self.assertEqual(
            self.driver.count_procs_in_domain(self.tag, "7"), 0,
            "a stack on another ROS domain must not count as a residual")

    def test_same_domain_duplicates_are_still_detected(self):
        self._spawn_probe("7")
        self._spawn_probe("7")
        self.assertEqual(self.driver.count_procs_in_domain(self.tag, "7"), 2)

    def test_mixed_domains_count_only_this_one(self):
        self._spawn_probe("7")
        self._spawn_probe("113")
        self._spawn_probe("113")
        self.assertEqual(self.driver.count_procs_in_domain(self.tag, "7"), 1)

    def test_missing_pattern_counts_zero(self):
        self.assertEqual(
            self.driver.count_procs_in_domain(self.tag, "7"), 0)

    def test_unreadable_process_is_counted_fail_closed(self):
        # PID 1 belongs to init; its environ is not readable as this user, and
        # an unattributable process must not silently clear the gate.
        self.assertIsNone(self.driver.ros_domain_of_pid(1))


if __name__ == "__main__":
    unittest.main()
