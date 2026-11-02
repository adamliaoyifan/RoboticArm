"""Offline contract for the unified surface / free-space final delivery.

Frozen by docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
(plan_revision 8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451, product baseline
2ec2393122481d4ad8467c85e9971e13bc0f820c). This module records the approved
gate numbers and the candidate-verification semantics both the product query
and the independent oracle implement. It does not change the observer and a
synthetic pass does not certify a live capture.

The old O5/O6/O7/O8/O9 numbers stay in repair_s0_contract.py; the capability
matrix keeps scoring against THAT module. Nothing here rewrites gen10 results.
"""
from __future__ import annotations

PLAN_REVISION = "8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451"
PRODUCT_BASE = "2ec2393122481d4ad8467c85e9971e13bc0f820c"
SCHEMA = "occ_freespace_final_v1"

# --- Target gates (formal scoring happens in OCTOMAP-2CM-REPAIR-S3 gen3) ----
UPDATE_HZ = 2.5          # formal representative target (proves >= 2 Hz)
QUERY_HZ = 2.0
INPUT_E2E_P95_MS = 350.0
INPUT_E2E_MAX_MS = 500.0
QUERY_FULL_P95_MS = 250.0
QUERY_FULL_MAX_MS = 500.0
REQUEST_DEADLINE_MS = 500.0     # every 400/500 ms period keeps its deadline
SNAPSHOT_AGE_MAX_S = 1.0
P_WINDOWS_PER_BOX = 3
P_BOX_COUNTS = (0, 5, 20)
P_WINDOW_INPUTS = 300            # at 2.5 Hz over 120 s
P_WINDOW_QUERIES = 240           # at 2 Hz over 120 s
P_WINDOW_S = 120.0
SCHED_LATE_P95_MS = 20.0
SCHED_LATE_MAX_MS = 50.0
R_OVERLOAD_INPUTS = 600          # 10 Hz x 60 s, 20 boxes
R_OVERLOAD_QUERIES = 120         # 2 Hz during the overload
R_DRAIN_S = 30.0
R_DRAIN_QUERIES = 60
R_RECOVERY_INPUTS = 75           # 2.5 Hz x 30 s
R_RECOVERY_QUERIES = 60
RECOVERY_FIRST_WAIT_S = 2.0
L_DURATION_S = 120 * 60.0
L_WARMUP_S = 10 * 60.0
L_INPUTS = 18000
L_QUERIES = 14400
L_RSS_LIMIT_KIB = 2 * 1024 * 1024
L_RSS_GROWTH = 0.05              # last-10min median vs first-10min after warmup
L_READ_RELEASE_ROUNDS = 1000
L_RESET_ROUNDS = 100
C_REPS = 30                      # each of the 9 concurrency/ownership paths
BARRIER_WRITE_LOCK_S = 1.0
BARRIER_READ_BUDGET_MS = 200.0
PRESSURE_RESPONSE_MAX_MS = 500.0
# F semantic gate: 9 classes x 10 variants x 2 box sizes x 2 yaws = 360 cases.
F_CLASSES = ("empty_tree", "observed_empty", "full_container", "partial_unknown",
             "occlusion", "floating_interlayer", "boundary", "two_version", "reset")
F_VARIANTS = 10
F_BOXES = ((0.30, 0.20, 0.20), (0.50, 0.30, 0.30))
F_YAWS = (0, 90)
F_CASES = len(F_CLASSES) * F_VARIANTS * len(F_BOXES) * len(F_YAWS)
O9_FREESPACE_PAIRS = 30

# --- Frozen placement policy (existing placement config, contract §2) -------
SUPPORT_TOL_M = 0.05
MIN_SUPPORT_RATIO = 0.6
CLEARANCE_MARGIN_M = 0.03        # top clearance to the container ceiling
SAME_LAYER_CLEARANCE_M = 0.05    # lateral clearance to occupied cells
HULL_MARGIN_M = 0.0              # voxel hull check = all volume cells active

# --- Candidate verification semantics (product and oracle both implement) --
# Discrete enumeration: per (yaw, cx, cy) footprint the candidate bottom rests
# exactly on the highest observed support below it (the existing placement
# policy's peak semantics); it is never enumerated floating within tolerance.
# Support slab depth: ceil(SUPPORT_TOL_M / resolution) cells (3 at 2 cm); the
# floor plane supports a column while the box bottom is within SUPPORT_TOL_M
# above the floor (cells 0..2 at 2 cm) -- a prior, never observed evidence.
# Rejection priority (exactly one reason per grid point):
#   out_of_domain -> collision -> clearance -> unknown -> support
# grid_total == feasible + rejected_{out_of_domain, collision, clearance,
# unknown, support} for every response page.
PAGE_LIMIT_MAX = 65536
YAW_MODES = {0: (0,), 1: (90,), 2: (0, 90)}
SUPPORT_KIND_OCCUPIED = 1
SUPPORT_KIND_FLOOR_PRIOR = 2

STATUSES = ("OK", "NO_KNOWN_FREE", "NO_DATA", "STALE", "SNAPSHOT_PRESSURE")

# Box sizes alternate per query; one box size per request; both yaws in one
# request for the P/R/L chains (yaw_mode=2).
BRIDGE_V1_KEYS = (
    "resolution", "nx", "ny", "inner_size", "floor_z", "center_base",
    "height", "state", "confidence", "geometry_hash", "map_revision",
)


def support_slab_cells(resolution: float) -> int:
    import math
    return int(math.ceil(SUPPORT_TOL_M / resolution - 1e-9))


def floor_support_cells(resolution: float) -> int:
    """Cells above the floor plane still within SUPPORT_TOL_M (inclusive)."""
    import math
    return int(math.floor(SUPPORT_TOL_M / resolution + 1e-9))


def lateral_clearance_cells(resolution: float) -> int:
    """Minimum integer cell separation whose world gap reaches the clearance.

    gap = (cx - ox - 1) * resolution >= SAME_LAYER_CLEARANCE_M for an occupied
    column left of the footprint; solve for the integer difference.
    """
    import math
    return int(math.ceil(SAME_LAYER_CLEARANCE_M / resolution - 1e-9)) + 1


def gate_table():
    """Machine-readable copy of the unified-target thresholds."""
    return {
        "plan_revision": PLAN_REVISION,
        "product_base": PRODUCT_BASE,
        "rows": [
            {"gate": "F", "workload": "9 classes x 10 variants x 2 sizes x 2 yaws",
             "repeat": "360 offline cases through the real store + query core",
             "pass": "matches the independent voxel/hull oracle; zero fake-free, "
                     "out-of-domain, or collision candidates; positive classes "
                     "return >=1 candidate and match the independent enumeration "
                     "set exactly; empty tree -> NO_DATA; no known-free volume -> "
                     "NO_KNOWN_FREE"},
            {"gate": "C", "workload": "9 concurrency/ownership paths",
             "repeat": "30 reps each, offline core + live service",
             "pass": "identity never mixes; no deadlock; barrier read under a "
                     "1 s write-lock hold finishes ROI/surface+freespace within "
                     "200 ms with the write lock still held at the end; release "
                     "returns to baseline; sequential reuse raises no peak; an "
                     "injected over-gate peak is rejected by the judge"},
            {"gate": "P", "workload": f"boxes 0/5/20, {P_WINDOWS_PER_BOX} x {P_WINDOW_S:.0f} s",
             "repeat": f"{P_WINDOW_INPUTS} inputs @ {UPDATE_HZ} Hz + {P_WINDOW_QUERIES} full queries @ {QUERY_HZ} Hz",
             "pass": f"every id exactly one terminal; input E2E P95<={INPUT_E2E_P95_MS:.0f} "
                     f"max<={INPUT_E2E_MAX_MS:.0f} ms; full query P95<={QUERY_FULL_P95_MS:.0f} "
                     f"max<={QUERY_FULL_MAX_MS:.0f} ms; 9 windows AND; no backlog growth; "
                     f"window tasks close within 0.5 s; snapshot age<={SNAPSHOT_AGE_MAX_S:.0f} s"},
            {"gate": "H", "workload": "P send/query planned vs actual call starts",
             "repeat": "same windows as P",
             "pass": f"lateness P95<={SCHED_LATE_P95_MS:.0f} ms max<={SCHED_LATE_MAX_MS:.0f} ms"},
            {"gate": "R", "workload": "20 boxes 10 Hz x 60 s + 30 s drain + recovery",
             "repeat": f"{R_OVERLOAD_INPUTS}/{R_OVERLOAD_QUERIES}+{R_DRAIN_QUERIES} then "
                       f"{R_RECOVERY_INPUTS}/{R_RECOVERY_QUERIES} same node+epoch",
             "pass": "all overload ids terminal; >=1 real queue/reject; 0 in flight at "
                     "the 30 s deadline; first recovery wait <=2 s and age <=2 s; no restart"},
            {"gate": "L", "workload": "20 boxes, 2.5 Hz + 2 Hz for 120 min",
             "repeat": f"{L_INPUTS} inputs / {L_QUERIES} queries + "
                       f"{L_READ_RELEASE_ROUNDS} read-release + {L_RESET_ROUNDS} reset rounds",
             "pass": f"no crash/deadlock; RSS<={L_RSS_LIMIT_KIB/1024/1024:.0f} GiB; "
                     f"attributed composite peak<={512} MiB; last-10min RSS median "
                     f"grows <={L_RSS_GROWTH:.0%}; fixed-map rounds return to baseline"},
            {"gate": "O8", "workload": "per repair_s0_contract O8",
             "repeat": "unchanged", "pass": "unchanged (gen10 judge)"},
            {"gate": "O9", "workload": "roi / integrate_30k / surface_first + freespace_full",
             "repeat": f"{O9_FREESPACE_PAIRS} ON/OFF pairs each, AB/BA",
             "pass": "P95 ratio<=1.10; 30/30 complete traces; whole-query chain "
                     "always-on capture is REQUIRED, not optional"},
        ],
    }
