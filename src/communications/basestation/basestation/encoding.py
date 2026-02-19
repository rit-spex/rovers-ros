"""Re-export MessageEncoder and Signal from the rovers-protocol package.

Requires ``rover_protocol`` to be installed in the environment
(``pip install lib/rovers-protocol``).
"""

from __future__ import annotations

from rover_protocol import MessageEncoder, Signal

__all__ = ["MessageEncoder", "Signal"]
