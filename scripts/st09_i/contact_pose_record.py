#!/usr/bin/env python3
"""Record box/panel poses and box contacts for one diagnostic run.

Two ``ign topic -e`` subscribers. Pose rows are filtered to the pickup box
and the suction links and capped at 20 Hz per name. Contact rows are every
published message (gz publishes only when contacts exist). A status file is
rewritten every couple of seconds so a pre-motion gate can see that the
recorder is alive even while the contact file is still empty.

  contact_pose_record.py --out-dir DIR
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import contact_capture as cap  # noqa: E402

KEEP_NAME = ("pickup_box", "suction_panel", "suction_contact")
MIN_DT = 0.05
POSE_TOPIC = "/world/airport_loading/pose/info"
CONTACT_TOPIC = "/pickup_box_contact"


def _split_messages(stream, on_message, stop):
    buf = []
    for line in stream:
        if stop.is_set():
            break
        if line.startswith("header {") and buf:
            on_message("".join(buf))
            buf = [line]
        else:
            buf.append(line)
            if len(buf) > 20000:
                buf = buf[-100:]
    if buf:
        on_message("".join(buf))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--pose-topic", default=POSE_TOPIC)
    parser.add_argument("--contact-topic", default=CONTACT_TOPIC)
    args = parser.parse_args(argv)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    pose_path = out / "poses.jsonl"
    contact_path = out / "contacts.jsonl"
    status_path = out / "recorder_status.json"
    counts = {"pose": 0, "contact": 0}
    last = {}
    lock = threading.Lock()
    stop = threading.Event()

    def write_status():
        row = {"alive": not stop.is_set(), "pose_messages": counts["pose"],
               "contact_messages": counts["contact"], "wall": time.time()}
        tmp = status_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(row) + "\n")
        os.replace(tmp, status_path)

    def on_pose(text):
        stamp, rows = cap.parse_pose_v(text)
        if stamp is None:
            return
        with lock:
            handle = open(pose_path, "a", encoding="utf-8")
            try:
                for name, xyz, quat in rows:
                    if not any(tok in name for tok in KEEP_NAME):
                        continue
                    if stamp - last.get(name, -1.0) < MIN_DT:
                        continue
                    last[name] = stamp
                    handle.write(json.dumps({
                        "stamp": stamp, "name": name,
                        "xyz": xyz, "quat": quat}) + "\n")
                    counts["pose"] += 1
            finally:
                handle.close()

    def on_contact(text):
        rows = cap.parse_contacts(text)
        if not rows:
            return
        with lock:
            with open(contact_path, "a", encoding="utf-8") as handle:
                for row in rows:
                    row["klass"] = cap.classify_pair(row["a"], row["b"])
                    handle.write(json.dumps(row) + "\n")
                    counts["contact"] += 1

    def pump(topic, handler):
        while not stop.is_set():
            proc = subprocess.Popen(
                ["ign", "topic", "-e", "-t", topic],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            contact_pose_children.append(proc)
            try:
                _split_messages(proc.stdout, handler, stop)
            finally:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
            time.sleep(1.0)

    children = []

    def shutdown(_signum, _frame):
        stop.set()
        for proc in children:
            proc.terminate()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    threads = [
        threading.Thread(target=pump, args=(args.pose_topic, on_pose), daemon=True),
        threading.Thread(target=pump, args=(args.contact_topic, on_contact), daemon=True),
    ]
    # pump appends its process to ``children``; patch it closed over above.
    contact_pose_children = children
    for thread in threads:
        thread.start()
    try:
        while not stop.is_set():
            write_status()
            time.sleep(2.0)
    except KeyboardInterrupt:
        shutdown(signal.SIGINT, None)
    write_status()
    return 0


if __name__ == "__main__":
    sys.exit(main())
