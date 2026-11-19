"""Tests for RPM/gearbox speed conversion and per-joint slew limits."""

import math

import pytest

from arm_constants import (
    ARM_MOTOR_GEAR_RATIO,
    ARM_MOTOR_MAX_ANG_VEL,
    ARM_MOTOR_PARAMS,
    MAX_ANG_VEL,
)
from arm_route_planner.kinematics.slew import SimulatedMechanicalArm


def test_motor_speed_includes_the_gearbox():
    """Profile velocity is motor RPM, so the joint sees RPM / 80."""
    for params in ARM_MOTOR_PARAMS:
        expected = (
            params.max_speed * 2.0 * math.pi / 60.0
            / ARM_MOTOR_GEAR_RATIO
        )
        assert params.max_output_rad_s == pytest.approx(expected)
        assert params.max_ticks_per_s == pytest.approx(
            abs(expected * params.ticks_per_rad))


def test_planner_uses_per_joint_motor_speeds():
    """The slew model starts from the speeds implied by ARM_MOTOR_PARAMS."""
    arm = SimulatedMechanicalArm([0.0] * 5)
    assert arm.max_ang_vel == pytest.approx(
        list(ARM_MOTOR_MAX_ANG_VEL[:5]))
    assert MAX_ANG_VEL == pytest.approx(min(ARM_MOTOR_MAX_ANG_VEL))


def test_slew_never_exceeds_each_joint_speed():
    """A large command in one cycle is capped by the per-joint speed."""
    arm = SimulatedMechanicalArm([0.0] * 5, ramp_time=0.0)
    arm.step([math.pi / 2.0] * 5, None, dt=1.0)
    for index, angle in enumerate(arm.angles):
        assert angle <= arm.max_ang_vel[index] + 1e-12
