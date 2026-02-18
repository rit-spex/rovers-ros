"""Re-export CONSTANTS, DataType, INPUT_TYPE from the rovers-protocol submodule."""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[4]
_SUBMODULE = _REPO_ROOT / "lib" / "rovers-protocol"
if _SUBMODULE.exists() and str(_SUBMODULE) not in sys.path:
    sys.path.insert(0, str(_SUBMODULE))

from rover_protocol import CONSTANTS, DataType, INPUT_TYPE  # noqa: E402

__all__ = ["CONSTANTS", "DataType", "INPUT_TYPE"]
