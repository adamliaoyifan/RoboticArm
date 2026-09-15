"""Pin the workspace source tree for luggage_perception test runs.

Sourcing an install space puts ``install/<pkg>/local/lib/python3.10/
dist-packages`` on PYTHONPATH, and nothing else puts this checkout's
``src/`` on ``sys.path`` — so pytest used to exercise the install tree.
That layout never contains ``config/`` (CMake installs it to share/ only),
which broke ``__file__``-relative data files such as the frozen table-ICP
yaml, and symlink installs lack links for files added after the last
build. Importing the checkout's source makes both impossible.

Both packages nest their importable package one directory below the
package root (``src/<pkg>/<pkg>/``), so each root is pinned separately
and first — pinning ``src/`` itself would only yield a namespace package
without the real modules.
"""

import os
import sys

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.normpath(os.path.join(_TEST_DIR, ".."))


def _pin(package_root, marker):
    if (os.path.isdir(os.path.join(package_root, marker))
            and marker not in sys.modules
            and package_root not in sys.path):
        sys.path.insert(0, package_root)


_pin(os.path.join(_PACKAGE_ROOT, "..", "luggage_description"),
     "luggage_description")
_pin(_PACKAGE_ROOT, "luggage_perception")
