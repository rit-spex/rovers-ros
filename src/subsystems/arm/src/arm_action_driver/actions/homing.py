"""Homing action configuration (parameter block).

Hold-to-run homing: ``homing``/``home`` start it and every held-key repeat
refreshes the hold timer without interrupting the motion, and the action
releases ``HOMING_HOLD_TIMEOUT_SEC`` after the last command.  The
basestation also sends ``home_release`` on key-up, which stops the move
immediately.  ``home_joint:<n>`` homes a single joint; its key-up maps to
the same ``home_release``.

Homing has two target sets: the normal home and the mirrored flipped-side
home.  When the homing command preempts a zenith hold it always returns to
the non-flipped home (the straight-up pose has no meaningful side).
"""

import math

from arm_constants import (
    ACTION_ACTIVE_FLIPPING,
    ACTION_ACTIVE_HOMING,
    ACTION_ACTIVE_RESTING,
    ACTION_ACTIVE_ZENITH,
    ACTION_HOME_JOINT_PREFIX,
    HOMING_HOLD_TIMEOUT_SEC,
    HOMING_TARGET,
    HOMING_THRESHOLD,
    MECHANICAL_LIMITS,
)

from arm_action_driver.actions.base import ActionConfiguration


class HomingConfiguration(ActionConfiguration):
    """Drive the measured joints to the (side-aware) home target."""

    name = "Homing"
    active_value = ACTION_ACTIVE_HOMING
    trigger_commands = ("homing", "home")
    release_commands = ("home_release",)
    single_joint_prefix = ACTION_HOME_JOINT_PREFIX

    # Behaviour flags
    hold_key = True
    hold_timeout_sec = HOMING_HOLD_TIMEOUT_SEC
    stop_on_release = True
    restart_on_repeat = True
    preempts = (ACTION_ACTIVE_ZENITH,)
    blocked_reasons = {
        ACTION_ACTIVE_FLIPPING: "Homing blocked: a flip is running",
        ACTION_ACTIVE_RESTING: "Homing blocked: resting is active",
    }
    report_homing_mask = True

    # Movement parameters
    target_angles = list(HOMING_TARGET)
    limits = MECHANICAL_LIMITS
    arrive_tol = HOMING_THRESHOLD
    deadband_target = True
    safe_path_required = False  # fall back to a direct move on a planner miss

    def target(self, driver):
        """Home pose: the non-flipped home, mirrored when flipped.

        Homing has two target sets: the normal home and the mirrored
        flipped-side home.  A homing command that preempts a zenith hold
        always returns to the NON-flipped home - the straight-up pose
        has no meaningful side, so the operator gets the standard home
        regardless of which side the zenith was entered from.
        """
        target = list(self.target_angles)
        if driver.flipped and not driver.in_zenith():
            # The flipped side mirrors shoulder/elbow around home.
            target[1] = math.pi - target[1]
            target[2] = -target[2]
        return target
