#!/usr/bin/env python3
"""Confirm castFreeRay's compiled object has no per-key timer calls.

Stage spans may call steady_clock. Two nested spans are four clock sites.
More than that means a per-key timer is still in the ray loop.
"""
import subprocess
import sys


def span(obj, needle):
    text = subprocess.check_output(["nm", "-C", "-S", obj], text=True, errors="replace")
    for line in text.splitlines():
        parts = line.split(maxsplit=3)
        if len(parts) < 4 or parts[2] not in "Tt":
            continue
        name = parts[3]
        if needle in name and "[clone .cold]" not in name:
            start = int(parts[0], 16)
            return start, start + int(parts[1], 16), name
    raise SystemExit(f"missing symbol containing {needle}")


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: check_hot_path.py occupancy_grid.cpp.o")
    obj = sys.argv[1]
    dump = subprocess.check_output(["objdump", "-d", "-C", obj], text=True, errors="replace")
    relocations = subprocess.check_output(["objdump", "-r", "-C", obj], text=True, errors="replace")
    banned = ("OccupancyDiagnostic::Timer", "domain_check", "hit_erase", "release_free")
    for name in banned:
        if name in dump or name in relocations:
            raise SystemExit(f"per-key timer still present in {obj}: {name}")
    start, end, name = span(obj, "OccupancyGrid::castFreeRay")
    clocks = 0
    for line in relocations.splitlines():
        parts = line.split()
        if len(parts) < 3 or "steady_clock::now" not in line:
            continue
        addr = int(parts[0], 16)
        if start <= addr < end:
            clocks += 1
    if not 1 <= clocks <= 4:
        raise SystemExit(f"{name} has {clocks} steady_clock sites; expected the two nested spans only")
    print(f"hot_path=pass clock_sites={clocks} object={obj}")


if __name__ == "__main__":
    main()
