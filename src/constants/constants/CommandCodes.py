"""Import bridge for the shared ``rover_protocol`` constants.

Adds the ``lib/rovers-protocol`` submodule to ``sys.path`` so that ROS
nodes can do::

    from constants.CommandCodes import CONSTANTS
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SUBMODULE = _REPO_ROOT / "lib" / "rovers-protocol"
if _SUBMODULE.exists() and str(_SUBMODULE) not in sys.path:
    sys.path.insert(0, str(_SUBMODULE))

from rover_protocol import CONSTANTS, DataType, INPUT_TYPE  # noqa: E402

__all__ = ["CONSTANTS", "DataType", "INPUT_TYPE"]
