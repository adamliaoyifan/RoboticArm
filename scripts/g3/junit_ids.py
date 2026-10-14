#!/usr/bin/env python3
"""Print failing/erroring test ids of JUnit files in a dir, and counts."""
import glob, os, sys
import xml.etree.ElementTree as ET
d = sys.argv[1]
fails, total = [], 0
for f in sorted(glob.glob(os.path.join(d, "*.xml"))):
    try:
        root = ET.parse(f).getroot()
    except ET.ParseError:
        fails.append("UNPARSEABLE:" + os.path.basename(f)); continue
    for case in root.iter("testcase"):
        total += 1
        if case.find("failure") is not None or case.find("error") is not None:
            fails.append("%s::%s" % (case.get("classname"), case.get("name")))
open(os.path.join(d, "failing_ids.txt"), "w").write("\n".join(sorted(fails)) + "\n")
print("tests=%d failing=%d" % (total, len(fails)))
