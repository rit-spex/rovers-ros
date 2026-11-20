"""Zenith action configuration (parameter block).

``zenith`` / ``zenith_mode`` / ``toggle_zenith`` enter the straight-up
holding pose (base and wrist twist stay wherever they were); sending the
command again while holding starts the controlled exit back to the
Pick-and-Place limits, and ``zenith_release`` (key-up) stops the drive and
holds the achieved pose.  The planner can also infer a zenith boot pose
and hand it over via ``adopt:zenith``.
"""

import math

from arm_constants import (
    ACTION_ACTIVE_ZENITH,
    ELBOW_DEADBAND,
    MECHANICAL_LIMITS,
    PICK_AND_PLACE_FLIPPED_LIMITS,
    PICK_AND_PLACE_LIMITS,
)

from arm_action_driver.actions.base import (
    ActionConfiguration,
    BASE,
    ELBOW,
    SHOULDER,
    WRIST_BEND,
    WRIST_TWIST,
)

# Arrival tolerance for the exit move.
_ZENITH_EXIT_ARRIVE_TOL = 1e-3


class ZenithConfiguration(ActionConfiguration):
    """Hold the straight-up pose and run the controlled exit."""

    name = "Zenith"
    active_value = ACTION_ACTIVE_ZENITH
    trigger_commands = ("zenith", "zenith_mode", "toggle_zenith")
    release_commands = ("zenith_release",)
    adopt_modes = ("zenith",)

    # Behaviour flags
    toggle_on_repeat = True
    hold_mode = True  # keep holding the pose; never auto-finish
    stop_on_release = True  # key-up stops the drive to the zenith pose
    remember_entry_side = True  # the exit returns to the side we came from

    # Movement parameters: shoulder straight up, elbow straight, wrists
    # zero; base and wrist twist are held wherever they were.
    holding_shoulder = math.pi / 2
    limits = MECHANICAL_LIMITS
    use_safe_path = False
    deadband_ok = True
    exit_arrive_tol = _ZENITH_EXIT_ARRIVE_TOL

    def target(self, driver):
        return [
            driver.measured[BASE],
            self.holding_shoulder,
            0.0,
            0.0,
            driver.measured[WRIST_TWIST],
        ]

    def exit_target(self, driver):
        """Controlled exit pose: back inside the Pick-and-Place limits."""
        angles = list(driver.measured)
        if self.entry_side:
            _, shoulder_hi = PICK_AND_PLACE_FLIPPED_LIMITS[SHOULDER]
            elbow = max(angles[ELBOW], ELBOW_DEADBAND)
        else:
            _, shoulder_hi = PICK_AND_PLACE_LIMITS[SHOULDER]
            elbow = min(angles[ELBOW], -ELBOW_DEADBAND)
        shoulder_lo = PICK_AND_PLACE_FLIPPED_LIMITS[SHOULDER][0] \
            if self.entry_side else PICK_AND_PLACE_LIMITS[SHOULDER][0]
        wrist_lo, wrist_hi = MECHANICAL_LIMITS[WRIST_BEND]
        twist_lo, twist_hi = MECHANICAL_LIMITS[WRIST_TWIST]
        return [
            angles[BASE],
            max(shoulder_lo, min(shoulder_hi, angles[SHOULDER])),
            elbow,
            max(wrist_lo, min(wrist_hi, angles[WRIST_BEND])),
            max(twist_lo, min(twist_hi, angles[WRIST_TWIST])),
        ]
