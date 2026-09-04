from __future__ import division

import os
import sys

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from luggage_planning.ik_diagnostics import EVENT_PREFIX, format_event


def test_format_event_is_single_line_and_machine_readable():
    line = format_event({"candidate_id": "interior_probe_00", "success": False})

    assert line.startswith(EVENT_PREFIX)
    assert "\n" not in line
    assert line.endswith('{"candidate_id":"interior_probe_00","success":false}')
