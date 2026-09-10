"""Publish Huayan box DI0 / DO0 / DO1 as /vacuum/* for site bags.

Wiring (control-box general IO unless ``vacuum_io_kind:=end``):

* DI0 = 1 suction confirmed (holding), 0 not holding
* DO0 = 1 vacuum pump on
* DO1 = 1 de-vacuum / release
"""

from __future__ import annotations

import json
from typing import Any, Optional

TOPIC_IO = "/vacuum/io"
TOPIC_DI0 = "/vacuum/di0"
TOPIC_DO0 = "/vacuum/do0"
TOPIC_DO1 = "/vacuum/do1"


def declare_vacuum_io_params(node) -> None:
    node.declare_parameter("vacuum_io_kind", "box")
    node.declare_parameter("vacuum_di_bit", 0)
    node.declare_parameter("vacuum_do0_bit", 0)
    node.declare_parameter("vacuum_do1_bit", 1)


def apply_vacuum_io_params(node, iface) -> None:
    iface.vacuum_io_kind = str(node.get_parameter("vacuum_io_kind").value).strip().lower()
    iface.vacuum_di_bit = int(node.get_parameter("vacuum_di_bit").value)
    iface.vacuum_do0_bit = int(node.get_parameter("vacuum_do0_bit").value)
    iface.vacuum_do1_bit = int(node.get_parameter("vacuum_do1_bit").value)


def snapshot(iface) -> Optional[dict[str, Any]]:
    if not getattr(iface, "last_io_ok", False):
        return None
    di0 = iface.vacuum_di0
    do0 = iface.vacuum_do0
    do1 = iface.vacuum_do1
    if di0 is None or do0 is None or do1 is None:
        return None
    return {
        "schema": "elfin_vacuum_io/v1",
        "source": str(getattr(iface, "vacuum_io_kind", "box")),
        "DI0": int(di0),
        "DO0": int(do0),
        "DO1": int(do1),
        "suction_ok": bool(di0),
        "vacuum_on": bool(do0),
        "de_vacuum": bool(do1),
    }


class VacuumIoPublisher:
    def __init__(self, node) -> None:
        from std_msgs.msg import Bool, String

        self._io = node.create_publisher(String, TOPIC_IO, 10)
        self._di0 = node.create_publisher(Bool, TOPIC_DI0, 10)
        self._do0 = node.create_publisher(Bool, TOPIC_DO0, 10)
        self._do1 = node.create_publisher(Bool, TOPIC_DO1, 10)

    def publish(self, iface) -> Optional[dict[str, Any]]:
        from std_msgs.msg import Bool, String

        payload = snapshot(iface)
        if payload is None:
            return None
        self._io.publish(String(data=json.dumps(payload)))
        self._di0.publish(Bool(data=payload["suction_ok"]))
        self._do0.publish(Bool(data=payload["vacuum_on"]))
        self._do1.publish(Bool(data=payload["de_vacuum"]))
        return payload
