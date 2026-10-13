"""The startup probe must use the launch domain to see robot_state_publisher.

A client left on another domain reports service_ready false. The same
client, after adopt_launch_environment, reads robot_description from
/robot_state_publisher/get_parameters.
"""

import json
import os
import subprocess
import sys
import textwrap
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SERVER_DOMAIN = "101"
OTHER_DOMAIN = "102"
URDF = "<robot name=\"probe\"/>"


def _env(domain):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["PYTHONPATH"] = os.pathsep.join([
        os.path.join(REPO, "scripts"),
        os.path.join(REPO, "src", "luggage_gazebo"),
        env.get("PYTHONPATH", ""),
    ])
    return env


class ProbeLaunchDomain(unittest.TestCase):
    def test_wrong_domain_misses_and_adopted_domain_reads_urdf(self):
        server = subprocess.Popen(
            [sys.executable, "-c", textwrap.dedent("""
                import rclpy
                from rclpy.node import Node
                rclpy.init()
                node = Node("robot_state_publisher")
                node.declare_parameter("robot_description", %r)
                try:
                    rclpy.spin(node)
                finally:
                    node.destroy_node()
                    rclpy.shutdown()
            """) % URDF],
            env=_env(SERVER_DOMAIN),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            adopted = subprocess.run(
                [sys.executable, "-c", textwrap.dedent("""
                    import os, sys
                    from sim_startup_rehearsal import (
                        RosStartupProbe, adopt_launch_environment,
                        restore_environment)
                    import rclpy
                    saved = adopt_launch_environment({
                        "ROS_DOMAIN_ID": %r,
                        "ROS_LOCALHOST_ONLY": "1",
                    })
                    rclpy.init()
                    node = rclpy.create_node("probe_match")
                    probe = RosStartupProbe(node)
                    ok = False
                    detail = {}
                    for _ in range(40):
                        ok, detail = probe.check_robot_description()
                        if ok:
                            break
                    import json
                    print(json.dumps({
                        "probe_domain": detail.get("probe_domain"),
                        "node": detail.get("node"),
                        "service": detail.get("service"),
                        "ok": bool(ok),
                        "urdf": probe.robot_description,
                    }))
                    node.destroy_node()
                    rclpy.shutdown()
                    restore_environment(saved)
                    sys.exit(0 if ok else 1)
                """) % SERVER_DOMAIN],
                env=_env(OTHER_DOMAIN),
                capture_output=True, text=True, timeout=20)
            self.assertEqual(adopted.returncode, 0, adopted.stderr + adopted.stdout)
            report = json.loads(adopted.stdout.strip().splitlines()[-1])
            self.assertEqual(report["probe_domain"], SERVER_DOMAIN)
            self.assertEqual(report["node"], "robot_state_publisher")
            self.assertEqual(
                report["service"], "/robot_state_publisher/get_parameters")
            self.assertTrue(report["ok"])
            self.assertEqual(report["urdf"], URDF)

            missed = subprocess.run(
                [sys.executable, "-c", textwrap.dedent("""
                    import sys
                    from sim_startup_rehearsal import RosStartupProbe
                    import rclpy
                    rclpy.init()
                    node = rclpy.create_node("probe_miss")
                    probe = RosStartupProbe(node)
                    ready = False
                    for _ in range(15):
                        ok, detail = probe.check_robot_description()
                        ready = bool(detail.get("service_ready"))
                        if ok or ready:
                            break
                    node.destroy_node()
                    rclpy.shutdown()
                    sys.exit(0 if not ready else 1)
                """)],
                env=_env(OTHER_DOMAIN),
                capture_output=True, text=True, timeout=15)
            self.assertEqual(
                missed.returncode, 0,
                "wrong domain saw the provider: %s" % missed.stderr)
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()


if __name__ == "__main__":
    unittest.main()
