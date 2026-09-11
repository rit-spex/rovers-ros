"""Re-export CONSTANTS, DataType, INPUT_TYPE from the rovers-protocol package.

Prefers ``rover_protocol`` from the active Python environment; when ROS
launches nodes with a different interpreter, falls back to the workspace
submodule at ``lib/rovers-protocol``.
"""

from __future__ import annotations

import sys
from pathlib import Path


def _ensure_rover_protocol_importable() -> None:
	"""Ensure the ``rover_protocol`` package can be imported."""
	try:
		import rover_protocol  # noqa: F401
		return
	except ModuleNotFoundError:
		pass

	for parent in Path(__file__).resolve().parents:
		candidate = parent / "lib" / "rovers-protocol"
		if (candidate / "rover_protocol" / "__init__.py").exists():
			candidate_str = str(candidate)
			if candidate_str not in sys.path:
				sys.path.insert(0, candidate_str)
			return


_ensure_rover_protocol_importable()

from rover_protocol import CONSTANTS, DataType, INPUT_TYPE

__all__ = ["CONSTANTS", "DataType", "INPUT_TYPE"]
