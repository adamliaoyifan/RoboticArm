"""Research imports must not appear in production packages."""

from __future__ import division

import os
import unittest

WORKSPACE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", ".."))
PROD_ROOTS = (
    os.path.join(WORKSPACE, "src", "luggage_perception"),
    os.path.join(WORKSPACE, "src", "luggage_planning"),
    os.path.join(WORKSPACE, "src", "luggage_packing"),
    os.path.join(WORKSPACE, "src", "luggage_bringup"),
    os.path.join(WORKSPACE, "src", "luggage_gazebo"),
    os.path.join(WORKSPACE, "src", "luggage_description"),
)
BANNED = ("research.lrf_p1", "lrf_p1", "learning_research")


class ImportIsolationTest(unittest.TestCase):
    def test_production_tree_has_no_research_import(self):
        hits = []
        for root in PROD_ROOTS:
            if not os.path.isdir(root):
                continue
            for dirpath, _dirs, files in os.walk(root):
                if "ros1_reference" in dirpath.split(os.sep):
                    continue
                for name in files:
                    if not name.endswith(".py"):
                        continue
                    path = os.path.join(dirpath, name)
                    with open(path, "r", encoding="utf-8") as handle:
                        text = handle.read()
                    for token in BANNED:
                        if token in text:
                            hits.append("%s:%s" % (path, token))
        self.assertEqual(hits, [])


if __name__ == "__main__":
    unittest.main()
