"""Re-export CONSTANTS, DataType, INPUT_TYPE from the rovers-protocol package.

Requires ``rover_protocol`` to be installed in the environment
(``pip install lib/rovers-protocol``).
"""

from __future__ import annotations

from rover_protocol import CONSTANTS, DataType, INPUT_TYPE

__all__ = ["CONSTANTS", "DataType", "INPUT_TYPE"]
