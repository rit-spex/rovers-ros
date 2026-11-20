"""Side-flip action configuration (parameter block).

Hold-to-run flip: ``flip`` starts (or resumes) the move to the mirrored
side; ``flip_release`` is the key-up event that pauses it in place with
the goal remembered, so the next ``flip`` press resumes toward the same
goal.  Moving the SpaceMouse drops the remembered goal.
"""

import math

from arm_constants import (
    ACTION_ACTIVE_FLIPPING,
    MECHANICAL_LIMITS,
)
from arm_route_planner.kinematics.linkage_models import wrap2pi

from arm_action_driver.actions.base import ActionConfiguration


class FlipConfiguration(ActionConfiguration):
    """Drive the arm to the mirrored Pick-and-Place side."""

    name = "Flip"
    active_value = ACTION_ACTIVE_FLIPPING
    trigger_commands = ("flip",)
    release_commands = ("flip_release",)

    # Behaviour flags
    pause_on_release = True
    resume_on_repeat = True
    cancel_on_manual = True
    octant_requires_planar = True

    # Movement parameters
    limits = MECHANICAL_LIMITS
    safe_path_required = True  # refuse to flip without a safe path

    def target(self, driver):
        """Mirror the measured pose around home (base and wrists kept)."""
        angles = list(driver.measured)
        target = [
            angles[0],
            math.pi - angles[1],
            -angles[2],
            angles[3],
            angles[4],
        ]
        return [max(lo, min(hi, wrap2pi(a)))
                for a, (lo, hi) in zip(target, self.limits)]
