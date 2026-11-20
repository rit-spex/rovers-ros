"""Tests for the teleop output stage.

The stage keeps an internal joint reference instead of rebuilding the
command from the measured angles.  These tests run the real controller
against a simple lagging plant and check the behaviour the operator feels:
straight paths, smooth steps, no speed-cap violations, a clean stop, and a
bounded lead over the measured arm.
"""

import math
import random

import numpy as np
import pytest

from arm_constants import DT, ARM_MOTOR_MAX_ANG_VEL, TELEOP_JOINT_ACCEL_TIME_S
from arm_route_planner.kinematics.arm_controller import RoverArmController

NOMINAL = [0.0, math.pi / 4, -math.pi / 4, 0.0, 0.0]
SUBSTEP = 0.005
PLANT_TAU = 0.12
STOP_SPEED = 0.05  # in/s


def simulate(velocity, ticks=60, release_after=None, tau=PLANT_TAU,
             noise_deg=0.0, seed=0, locks=None):
    """Run teleop against a first-order plant and return the trajectory."""
    rng = random.Random(seed)
    arm = RoverArmController.from_measured(NOMINAL)
    if locks is not None:
        arm.locked_joints = list(locks)
    measured = list(NOMINAL)
    commands = []
    tool = []
    substeps = max(1, int(round(DT / SUBSTEP)))
    for tick in range(ticks):
        target = velocity if release_after is None or tick < release_after \
            else (0.0,) * 6
        if not arm.update_teleop(target, measured):
            arm.hold()
        command = list(arm.commanded_angles())
        commands.append(command)
        for _ in range(substeps):
            measured = [
                m + (c - m) * (1.0 - math.exp(-SUBSTEP / tau))
                for m, c in zip(measured, command)
            ]
        if noise_deg:
            measured = [
                m + math.radians(rng.gauss(0.0, noise_deg)) for m in measured
            ]
        tool.append(
            np.array(arm.model.solve_forward_kinematics(measured).position()))
    return arm, np.array(tool), np.degrees(np.array(commands))


def joint_speeds(commands):
    """Return the per-cycle joint speed [deg/s]."""
    return np.diff(commands, axis=0) / DT


def joint_accels(commands):
    """Return the per-cycle joint acceleration [deg/s^2]."""
    return np.abs(np.diff(commands, axis=0, n=2)) / (DT * DT)


def test_forward_teleop_stays_straight():
    """A straight push must not bow the tool path sideways or vertically."""
    _, tool, _ = simulate((0.0, -4.5, 0.0, 0.0, 0.0, 0.0), ticks=40)
    moved = tool[39] - tool[0]
    assert abs(moved[0]) < 0.5
    assert abs(moved[2]) < 0.5
    assert moved[1] < -1.0


def test_diagonal_teleop_stays_on_its_line():
    """The coordinated cap must preserve the 45 degree direction."""
    _, tool, _ = simulate((0.0, -2.7, 2.7, 0.0, 0.0, 0.0), ticks=40)
    moved = tool[39] - tool[0]
    assert abs(moved[1] + moved[2]) < 0.5


def test_forward_teleop_stays_straight_with_free_wrists():
    """Free wrists must not bend a straight translation command."""
    _, tool, _ = simulate(
        (0.0, -4.5, 0.0, 0.0, 0.0, 0.0), ticks=40, locks=[False] * 5)
    moved = tool[39] - tool[0]
    assert abs(moved[0]) < 0.5
    assert abs(moved[2]) < 0.5
    assert moved[1] < -1.0


def test_one_zero_tick_does_not_reset_the_stage():
    """A single zero sample decelerates; it does not stop and re-ramp."""
    arm = RoverArmController.from_measured(NOMINAL)
    measured = list(NOMINAL)
    substeps = max(1, int(round(DT / SUBSTEP)))
    for _ in range(20):
        arm.update_teleop((0.0, -4.5, 0.0, 0.0, 0.0, 0.0), measured)
        command = list(arm.commanded_angles())
        for _ in range(substeps):
            measured = [
                m + (c - m) * (1.0 - math.exp(-SUBSTEP / PLANT_TAU))
                for m, c in zip(measured, command)
            ]
    before = list(arm.teleop_reference)
    assert arm.update_teleop((0.0,) * 6, measured)
    moved = max(abs(a - b) for a, b in zip(arm.teleop_reference, before))
    assert moved > math.radians(0.2)
    assert arm.teleop_motion_active


def test_teleop_respects_joint_speed_and_accel_limits():
    """No joint may exceed its hardware cap or the teleop accel limit."""
    _, _, commands = simulate(
        (0.0, -4.5, 0.0, 0.0, 0.0, 0.0), ticks=40)
    speeds = np.abs(joint_speeds(commands))
    accels = joint_accels(commands)
    caps = np.degrees(np.array(ARM_MOTOR_MAX_ANG_VEL[:5]))
    assert np.all(speeds <= caps + 0.2)
    assert np.all(accels <= caps / TELEOP_JOINT_ACCEL_TIME_S + 1.0)


def test_teleop_ignores_measured_noise():
    """Encoder noise must not appear in the commanded target."""
    _, _, commands = simulate(
        (0.0, -1.13, 0.0, 0.0, 0.0, 0.0), ticks=40, noise_deg=0.03)
    roughness = np.diff(commands, axis=0, n=2)[10:35, 1:3].std()
    assert roughness < 0.01


def test_teleop_reversal_changes_direction_once():
    """A direction change must pass through zero, not chatter."""
    arm = RoverArmController.from_measured(NOMINAL)
    measured = list(NOMINAL)
    tool = []
    for tick in range(80):
        velocity = (0.0, -4.5 if tick < 40 else 4.5, 0.0, 0.0, 0.0, 0.0)
        if not arm.update_teleop(velocity, measured):
            arm.hold()
        command = list(arm.commanded_angles())
        for _ in range(max(1, int(round(DT / SUBSTEP)))):
            measured = [
                m + (c - m) * (1.0 - math.exp(-SUBSTEP / PLANT_TAU))
                for m, c in zip(measured, command)
            ]
        tool.append(
            np.array(arm.model.solve_forward_kinematics(measured).position()))
    velocity_y = np.diff(np.array(tool)[:, 1]) / DT
    moving = np.abs(velocity_y) > STOP_SPEED
    flips = np.sum(np.diff(np.sign(velocity_y[moving])) != 0)
    assert flips == 1


def test_teleop_release_stops_inside_the_target():
    """Releasing the stick stops the arm in about half a second."""
    _, tool, _ = simulate(
        (0.0, -4.5, 0.0, 0.0, 0.0, 0.0), ticks=70, release_after=40)
    speed = np.linalg.norm(np.diff(tool, axis=0) / DT, axis=1)
    stopped = next(
        (tick for tick in range(40, 69) if speed[tick] < STOP_SPEED), None)
    assert stopped is not None
    assert (stopped - 40) * DT <= 0.61
    coast = np.linalg.norm(tool[-1] - tool[39])
    assert coast < 1.2


def test_teleop_lead_stays_bounded_and_reports_a_stall():
    """A stuck arm must hit the lead bound and report a stall, not run away."""
    arm = RoverArmController.from_measured(NOMINAL)
    measured = list(NOMINAL)  # the plant never moves
    lead = math.radians(5.0)
    failure = None
    for _ in range(22):
        arm.update_teleop((0.0, -4.5, 0.0, 0.0, 0.0, 0.0), measured)
        assert arm.teleop_reference is not None
        for reference, actual in zip(arm.teleop_reference, measured):
            assert abs(reference - actual) <= lead + 1e-9
        failure = failure or arm.take_failure()
    assert failure is not None
    assert "stall" in failure.lower()


def test_hold_and_resume_restarts_from_measured():
    """An idle hold keeps the reference; a new push re-arms from the arm."""
    arm = RoverArmController.from_measured(NOMINAL)
    measured = list(NOMINAL)
    # Simulate a stale reference well ahead of the arm.
    arm.teleop_reference = [a + math.radians(11.0) for a in NOMINAL]
    arm._teleop_needs_reset = True

    assert not arm.update_teleop((0.0,) * 6, measured)
    assert not arm.teleop_motion_active
    assert arm.teleop_reference[0] == pytest.approx(
        NOMINAL[0] + math.radians(11.0))

    arm.update_teleop((0.0, -0.5, 0.0, 0.0, 0.0, 0.0), measured)
    assert abs(arm.teleop_reference[0] - measured[0]) < math.radians(1.0)
