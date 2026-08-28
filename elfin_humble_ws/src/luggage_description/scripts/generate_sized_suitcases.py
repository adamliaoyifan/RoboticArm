#!/usr/bin/env python3
"""Write the six pre-scaled pickup suitcase models (run once, not at spawn)."""

from __future__ import division

import os
import sys


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    here = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    default_root = os.path.normpath(os.path.join(
        here, "..", "luggage_gazebo", "models"))
    models_root = argv[0] if argv else default_root
    from luggage_description.suitcase_visual import write_sized_suitcase_models
    records = write_sized_suitcase_models(models_root)
    print("wrote %d sized suitcase models under %s" % (
        len(records), models_root))
    for name in sorted(records):
        rec = records[name]
        m = rec["measure_size"]
        s = rec["size"]
        print("  %s size=%.3fx%.3fx%.3f measure=%.3fx%.3fx%.3f" % (
            name, s[0], s[1], s[2], m[0], m[1], m[2]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
