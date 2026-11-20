"""Tests for the resting-limit override in the simulated arm hardware."""

import math

from rover_simulation.arm_sim_node import (
    RESTING_TICK_LIMITS,
    TICK_LIMITS,
    angle_to_ticks,
    ticks_to_angle,
)

SHOULDER = 1
ELBOW = 2


def _inside(ticks, index, tick_limits):
    """True when *ticks* lies inside the ordered tick window."""
    low, high = tick_limits[index]
    return low <= ticks <= high


def test_normal_tick_limits_reject_the_resting_pushes():
    # The provisional ARM_MOTOR_PARAMS tick limits are tighter than the
    # resting region, so the push targets are rejected without the latch.
    shoulder_ticks = angle_to_ticks(SHOULDER, math.radians(-15.0))
    elbow_ticks = angle_to_ticks(ELBOW, math.radians(165.0))
    assert not _inside(shoulder_ticks, SHOULDER, TICK_LIMITS)
    assert not _inside(elbow_ticks, ELBOW, TICK_LIMITS)


def test_resting_tick_limits_accept_the_push_targets():
    # The elbow's negative ticks_per_rad flips the tick direction, so the
    # checks work in tick space against the ordered window.
    shoulder_ticks = angle_to_ticks(SHOULDER, math.radians(-15.0))
    elbow_ticks = angle_to_ticks(ELBOW, math.radians(165.0))
    assert _inside(shoulder_ticks, SHOULDER, RESTING_TICK_LIMITS)
    assert _inside(elbow_ticks, ELBOW, RESTING_TICK_LIMITS)
    # The shoulder stall and its 0.5 deg backoff also fit.
    assert _inside(
        angle_to_ticks(SHOULDER, math.radians(-10.0)),
        SHOULDER, RESTING_TICK_LIMITS)
    assert _inside(
        angle_to_ticks(SHOULDER, math.radians(-9.5)),
        SHOULDER, RESTING_TICK_LIMITS)


def test_resting_limits_reach_the_expected_stall_points():
    # The widened windows (converted back through the calibration) cover the
    # expected stall region on both sides of the normal limits.
    elbow_low_rad = ticks_to_angle(ELBOW, RESTING_TICK_LIMITS[ELBOW][0])
    elbow_high_rad = ticks_to_angle(ELBOW, RESTING_TICK_LIMITS[ELBOW][1])
    elbow_min = min(elbow_low_rad, elbow_high_rad)
    elbow_max = max(elbow_low_rad, elbow_high_rad)
    assert elbow_min <= math.radians(-135.0)
    assert elbow_max >= math.radians(160.0)
    shoulder_min = min(
        ticks_to_angle(SHOULDER, RESTING_TICK_LIMITS[SHOULDER][0]),
        ticks_to_angle(SHOULDER, RESTING_TICK_LIMITS[SHOULDER][1]))
    assert shoulder_min <= math.radians(-10.0)
