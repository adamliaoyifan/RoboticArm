#!/usr/bin/env python3
"""Reproduce the geometry-mismatch select against a running stack.

Adopts the driver peer (GraphProbe), runs one placement, then one
select with a corrupted geometry_hash — mirroring the xfail test — so
the selector's server-side progress is observable in the stack log.
"""

import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "luggage_planning" / "scripts"))

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

BOX_WDH = [0.35, 0.25, 0.20]


def main():
    domain = int(sys.argv[1]) if len(sys.argv) > 1 else 69
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    executor = MultiThreadedExecutor()
    from st09_i_probe.lib_chain import ChainHandle  # type: ignore

    chain = ChainHandle(executor=executor)
    executor_thread = __import__("threading").Thread(
        target=executor.spin, daemon=True)
    executor_thread.start()
    assert chain.probe.require_graph_ready(60.0), "graph not ready"

    from tests.st09_parallel.I.test_mock_whole_chain import (  # type: ignore
        _compute_placement,
        _select_request,
    )
    placement = _compute_placement(chain, "probe-mm-1")
    print("placement:", placement is not None and placement.success)
    request = _select_request(chain, "probe-mm-1", placement)
    request["geometry_hash"] = "not-the-hash"
    adapter = chain.adapter()
    t0 = time.monotonic()
    selection = adapter.select_place_path(timeout_sec=60.0, **request)
    print("select returned in %.1fs" % (time.monotonic() - t0))
    print("success=%s reason=%s" % (
        getattr(selection, "success", None),
        getattr(selection, "reason_code", None)))
    if selection is not None and not selection.success:
        rows = json.loads(getattr(selection, "rows_json", "") or "[]")
        print("rows:", len(rows), "last:", json.dumps(rows[-1])[:200]
              if rows else None)


if __name__ == "__main__":
    main()
