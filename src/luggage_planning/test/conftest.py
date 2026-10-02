import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

# Small-matrix numpy fan-out (FK loops, dense certifier sampling) spins every
# core via OpenBLAS on this machine and perturbs reduction order; pin BLAS
# threads before numpy loads anywhere in the test suite.
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")
