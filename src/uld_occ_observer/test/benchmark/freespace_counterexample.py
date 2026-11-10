#!/usr/bin/env python3
"""The CONTROLLED counterexample proving the checker discriminates (gen4).

A deliberately retained lease must be REPORTED as residue in its owner
class -- lease block bytes and directory entries above the baseline, every
other class untouched -- and releasing it must return every class to the
baseline. The residue is real product state made deliberately: acquire a
lease, integrate NEW content so the store diverges from the leased view
(the lease then holds view-exclusive old blocks), and simply do not
release. Bounded (<2 min, a handful of frames), never a scored gate: this
proves the CHECKER has discrimination, which is why one controlled
counterexample is enough -- not an ever-growing set.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import freespace_campaign as campaign  # noqa: E402
import freespace_verdict as verdict  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames-glob", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = campaign.load_frames(args.frames_glob)
    if len(frames) < 2:
        raise SystemExit("need at least two distinct frames for divergence")

    node = campaign.NodeProcess(args.node, args.geometry, args.config,
                                out / "observer.log",
                                "/freespace_campaign_cloud")
    client = campaign.Client("/freespace_campaign_cloud")
    document = {"stage": "counterexample"}
    try:
        client.wait_services()
        tf = frames[0].get("transform") or {}
        client.broadcast_tf(tf.get("translation") or [-1.15, 0.0, 1.20])
        campaign.readiness(client, frames)
        baseline = campaign._ownership(client, max_samples=8)

        # The deliberately retained lease, made REAL by divergence: hold a
        # lease, integrate the next frame, and simply never release.
        lease, acquire_status = campaign._acquire_real_lease(client, 10_000, 8.0)
        if lease is None:
            raise SystemExit("counterexample could not acquire a lease: %s"
                             % acquire_status)
        document["acquire_status"] = acquire_status
        mark = len(client.receipts)
        campaign.send_inputs(client, frames[1:2], 1, 1.0,
                             time.monotonic_ns(), int(time.time() * 1e9))
        deadline = time.monotonic() + 15
        integrated = False
        while time.monotonic() < deadline:
            if any(receipt["status"] == "INTEGRATED"
                   for receipt in client.receipts[mark:]):
                integrated = True
                break
            time.sleep(0.05)
        document["divergence_integrated"] = integrated
        document["residue_with_lease"] = campaign._ownership(
            client, max_samples=8, settle_s=0.5)

        # Release the last holder: every class must return to the baseline.
        release = client.call(client.release,
                              campaign._release_request(lease.handle))
        document["release_status"] = release.status
        document["after_release"] = campaign._ownership(
            client, max_samples=8, settle_s=0.5)
        document["baseline"] = baseline
    finally:
        client.shutdown()
        node.stop()
    (out / "counterexample.json").write_text(json.dumps(document, indent=1))
    problems = verdict.counterexample_problems(document)
    print(json.dumps({"problems": problems[:10], "total": len(problems)},
                     indent=1))
    return 0 if not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
