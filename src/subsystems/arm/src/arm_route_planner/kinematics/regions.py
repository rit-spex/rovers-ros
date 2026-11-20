"""Observed pose regions for the 5-DOF arm.

Pure library: no ROS imports.  Regions are properties of the *measured*
pose (flipped side, zenith pose, resting pose, planar base band), not
commanded modes.  The planner uses them to report flags and to decide
which side the pose belongs to; the action driver uses them for startup
inference and resting entry.  Details: planz/ACTION_DRIVER_PLAN.md.

The tracker adds a small angular deadband so a pose hovering on a region
boundary does not make the reported flags flicker.  Limit *selection* is
latched separately by the planner (settle_side), so teleop never swaps
the active limits mid-motion.
"""

import math

from arm_constants import (
    ELBOW_DEADBAND,
    MECHANICAL_LIMITS,
    PICK_AND_PLACE_LIMITS,
    PICK_AND_PLACE_FLIPPED_LIMITS,
    RESTING_INFERENCE_TOL,
    ZENITH_ANGLE_TOL,
)
from .linkage_models import wrap2pi

# Extra angular margin a region keeps before it stops reporting.
REGION_HYSTERESIS_RAD = math.radians(1.0)

ZENITH_SHOULDER = math.pi / 2


def is_zenith_pose(angles, tol=ZENITH_ANGLE_TOL):
    """True when the measured pose is the straight-up zenith pose.

    Base and wrist twist are deliberately ignored: zenith holds them
    where they were when the action started, so a held-twist pose must
    still classify as zenith.
    """
    if angles is None or len(angles) < 5:
        return False
    if any(a is None for a in angles):
        return False
    return (
        abs(wrap2pi(angles[1] - ZENITH_SHOULDER)) <= tol
        and abs(wrap2pi(angles[2])) <= max(tol, ELBOW_DEADBAND)
        and abs(wrap2pi(angles[3])) <= tol
    )


def is_resting_pose(angles, tol=RESTING_INFERENCE_TOL):
    """True when the pose sits in the resting (collapsed) region.

    Resting is shoulder below the planar lower bound or elbow above the
    normal upper limit, by more than the inference tolerance.
    """
    if angles is None or len(angles) < 5:
        return False
    if any(a is None for a in angles):
        return False
    shoulder_below = angles[1] < 0.0 - tol
    elbow_above = angles[2] > MECHANICAL_LIMITS[2][1] + tol
    return shoulder_below or elbow_above


def in_planar_base_band(angles, band=None):
    """True when the base sits inside the planar modifier's band."""
    if angles is None or len(angles) < 1 or angles[0] is None:
        return False
    if band is None:
        from arm_constants import PLANAR_BASE
        band = PLANAR_BASE
    return abs(wrap2pi(angles[0])) <= band + 1e-9


def _fits(angles, limits, margin):
    return all(
        lo - margin <= angle <= hi + margin
        for angle, (lo, hi) in zip(angles, limits)
    )


class RegionTracker:
    """Track observed region flags with transition hysteresis."""

    def __init__(self, deadband=REGION_HYSTERESIS_RAD):
        self.deadband = max(0.0, float(deadband))
        self.zenith = False
        self.resting = False
        self.flipped = False

    def update(self, angles):
        """Recompute the flags from the measured pose and return self."""
        if angles is None or len(angles) < 5:
            return self
        if any(a is None for a in angles):
            return self

        zenith_tol = ZENITH_ANGLE_TOL + (
            self.deadband if self.zenith else 0.0)
        self.zenith = is_zenith_pose(angles, tol=zenith_tol)

        resting_tol = RESTING_INFERENCE_TOL + (
            self.deadband if self.resting else 0.0)
        self.resting = is_resting_pose(angles, tol=resting_tol)

        flipped_margin = self.deadband if self.flipped else 0.0
        normal_margin = self.deadband if not self.flipped else 0.0
        flipped_fits = _fits(
            angles, PICK_AND_PLACE_FLIPPED_LIMITS, flipped_margin)
        normal_fits = _fits(
            angles, PICK_AND_PLACE_LIMITS, normal_margin)
        if flipped_fits and not normal_fits:
            self.flipped = True
        elif normal_fits and not flipped_fits:
            self.flipped = False
        return self
