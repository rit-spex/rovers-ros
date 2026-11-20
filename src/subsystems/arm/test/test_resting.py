"""Tests for the RESTING collapse / de-torque / restore feature."""

import math

import pytest

from arm_constants import (
    ARM_OPERATION_MODES,
    ARM_FLAG_NAMES,
    DT,
    MECHANICAL_LIMITS,
    RESTING_LIMITS,
    RESTING_ELBOW_PUSH,
    RESTING_SHOULDER_PUSH,
    RESTING_BACKOFF_ANGLE,
)
from arm_route_planner.kinematics.actions.resting import RestingController
from arm_route_planner.kinematics.arm_controller import RoverArmController


def _run(arm, frames, stops=None, feedback_fresh=True, enabled=True):
    """Run controller cycles, optionally pinning joints at physical stops.

    ``stops`` maps joint index -> stop angle; the measured angle is clamped
    there like a physical hard stop would.
    """
    events = []
    for _ in range(frames):
        measured = list(arm.mechanical.angles)
        if stops:
            for joint, limit in stops.items():
                if limit > 0:
                    measured[joint] = min(measured[joint], limit)
                else:
                    measured[joint] = max(measured[joint], limit)
        arm.sync_angles(measured, reset_target=False)
        arm.update(
            feedback_fresh=feedback_fresh, resting_enabled=enabled)
        events.extend(arm.take_resting_events())
    return events


# ----------------------------------------------------------------------
# Constants
# ----------------------------------------------------------------------

def test_resting_mode_code_and_flag_exist():
    assert ARM_OPERATION_MODES["RESTING"] == 5
    assert "RESTING" in ARM_FLAG_NAMES


def test_resting_limits_only_pad_shoulder_and_elbow():
    shoulder_lo, shoulder_hi = RESTING_LIMITS[1]
    elbow_lo, elbow_hi = RESTING_LIMITS[2]
    assert shoulder_lo == pytest.approx(math.radians(-28.0))
    assert shoulder_hi == pytest.approx(MECHANICAL_LIMITS[1][1])
    assert elbow_lo == pytest.approx(MECHANICAL_LIMITS[2][0])
    assert elbow_hi == pytest.approx(math.radians(165.0))
    assert RESTING_LIMITS[0] == MECHANICAL_LIMITS[0]
    assert RESTING_LIMITS[3] == MECHANICAL_LIMITS[3]
    assert RESTING_LIMITS[4] == MECHANICAL_LIMITS[4]
    assert MECHANICAL_LIMITS[1][0] == pytest.approx(math.radians(2.0))
    assert MECHANICAL_LIMITS[2][1] == pytest.approx(math.radians(135.0))


def test_elbow_push_target_equals_the_padded_limit():
    assert RESTING_ELBOW_PUSH == pytest.approx(RESTING_LIMITS[2][1])


# ----------------------------------------------------------------------
# RestingController unit behaviour
# ----------------------------------------------------------------------

def _push_phase_controller(joint, target, start_angle):
    """Build a controller already in a push phase."""
    controller = RestingController()
    controller.begin_collapse()
    controller.phase = controller.PHASE_PUSH_SHOULDER
    controller._start_push(joint, target, [0.0] * 5)
    controller._push_start_angle = start_angle
    return controller


def test_push_declares_stall_when_measured_stops_advancing():
    controller = _push_phase_controller(1, RESTING_SHOULDER_PUSH, 0.0)
    angles = [0.0, 0.0, 0.0, 0.0, 0.0]
    for _ in range(40):  # 2 s > grace + window
        target, _, events = controller.update(
            angles, dt=DT, feedback_fresh=True, enabled=True)
        assert target is not None
        angles = list(target)
    # The mechanical arm keeps advancing in this synthetic test, so force a
    # stall by holding the measured joint still.
    controller2 = _push_phase_controller(1, RESTING_SHOULDER_PUSH, 0.0)
    measured = [0.0, -0.03, 0.0, 0.0, 0.0]
    for _ in range(60):
        target, _, events = controller2.update(
            measured, dt=DT, feedback_fresh=True, enabled=True)
        if controller2.phase == controller2.PHASE_BACKOFF_SHOULDER:
            break
    assert controller2.phase == controller2.PHASE_BACKOFF_SHOULDER
    assert controller2._backoff_target == pytest.approx(
        measured[1] + RESTING_BACKOFF_ANGLE)


def test_push_aborts_when_feedback_dies():
    controller = _push_phase_controller(1, RESTING_SHOULDER_PUSH, 0.0)
    _, _, events = controller.update(
        [0.0] * 5, dt=DT, feedback_fresh=False, enabled=True)
    assert controller.phase == controller.PHASE_HOLD
    assert any(event.startswith("abort:") for event in events)
    assert "feedback" in controller.error


def test_push_aborts_when_arm_is_disabled():
    controller = _push_phase_controller(1, RESTING_SHOULDER_PUSH, 0.0)
    _, _, events = controller.update(
        [0.0] * 5, dt=DT, feedback_fresh=True, enabled=False)
    assert controller.phase == controller.PHASE_HOLD
    assert any(event.startswith("abort:") for event in events)
    assert "disabled" in controller.error


def test_push_aborts_when_the_stop_is_missing():
    # The measured joint reaches the push target without ever stalling.
    controller = _push_phase_controller(2, RESTING_ELBOW_PUSH, 0.0)
    measured = [0.0, 0.0, RESTING_ELBOW_PUSH, 0.0, 0.0]
    for _ in range(30):  # past the start grace
        _, _, events = controller.update(
            measured, dt=DT, feedback_fresh=True, enabled=True)
        if controller.phase == controller.PHASE_HOLD:
            break
    assert controller.phase == controller.PHASE_HOLD
    assert any("without stalling" in event for event in events)


def test_no_motion_during_grace_aborts():
    controller = _push_phase_controller(1, RESTING_SHOULDER_PUSH, 0.0)
    measured = [0.0, 0.0, 0.0, 0.0, 0.0]
    for _ in range(30):  # 1.5 s
        _, _, events = controller.update(
            measured, dt=DT, feedback_fresh=True, enabled=True)
        if controller.phase == controller.PHASE_HOLD:
            break
    assert controller.phase == controller.PHASE_HOLD
    assert "no motion" in controller.error


def test_restore_sequence_transitions():
    controller = RestingController()
    assert controller.begin_collapse()
    controller.phase = controller.PHASE_COLLAPSED
    assert controller.request_restore()
    assert controller.phase == controller.PHASE_TORQUE_ON
    _, _, events = controller.update([0.0] * 5, dt=DT)
    assert "torque_on" in events
    assert controller.torque_confirmed()
    assert controller.phase == controller.PHASE_MOVE_OUT
    # The crouch arrival normally flips MOVE_OUT -> AUTO_HOME inside the
    # waypoint move; jump there for the unit check.
    controller.phase = controller.PHASE_AUTO_HOME
    assert controller.homing_finished()
    assert controller.phase == controller.PHASE_DONE
    assert not controller.active


def test_abort_keeps_the_resting_state_owned():
    controller = RestingController()
    controller.begin_collapse()
    controller.abort("user abort")
    assert controller.holding
    assert controller.active
    assert controller.request_restore()


# ----------------------------------------------------------------------
# RoverArmController integration
# ----------------------------------------------------------------------

def test_enter_resting_releases_locks_and_resets_modes():
    arm = RoverArmController.from_measured(
        [0.0, math.radians(45), math.radians(-45), 0.0, 0.0])
    arm.toggle_joint_lock(0)
    arm.set_joint_drive(True)
    assert any(arm.locked_joints)

    assert arm.enter_resting()
    assert arm.resting
    assert not any(arm.locked_joints)
    assert not arm.planar
    assert not arm.flipped
    assert not arm.joint_drive
    assert arm.limit_mode.name == "Resting"
    assert arm.resting_action.phase == RestingController.PHASE_CROUCH


def test_enter_resting_blocked_by_zenith_homing_and_flip():
    arm = RoverArmController.from_measured(
        [math.radians(30), math.pi / 2, 0.0, 0.0, 0.0])
    assert arm.zenith
    assert not arm.enter_resting()
    assert "zenith" in arm.take_failure()

    arm2 = RoverArmController.from_measured(
        [0.0, math.radians(45), math.radians(-45), 0.0, 0.0])
    assert arm2.start_homing()
    assert not arm2.enter_resting()
    assert "action is running" in arm2.take_failure()


def test_commands_are_blocked_while_resting():
    arm = RoverArmController.from_measured(
        [0.0, math.radians(45), math.radians(-45), 0.0, 0.0])
    assert arm.enter_resting()

    assert not arm.activate_flip()
    arm.take_failure()
    assert not arm.start_homing()
    arm.take_failure()
    assert not arm.set_pnp_normal()
    arm.take_failure()
    assert not arm.toggle_flipped()
    arm.take_failure()
    assert not arm.toggle_planar()
    arm.take_failure()
    assert not arm.toggle_joint_lock(0)
    assert "resting" in arm.take_failure().lower()
    assert not arm.home_joint(2)
    arm.take_failure()
    assert not arm.command_pose([1.0, 0.0, 0.0], _identity_rotation())
    arm.take_failure()
    # A handover hold leaves the resting target alone.
    assert arm.hold()


def _identity_rotation():
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def test_toggle_resting_aborts_a_running_collapse():
    arm = RoverArmController.from_measured(
        [0.0, math.radians(45), math.radians(-45), 0.0, 0.0])
    assert arm.enter_resting()
    _run(arm, 5)
    assert arm.toggle_resting()
    assert arm.resting_action.holding
    assert not arm.resting_action.collapsed


def test_full_collapse_against_physical_stops():
    arm = RoverArmController.from_measured(
        [0.0, math.radians(45), math.radians(-45), 0.0, 0.0])
    assert arm.enter_resting()

    # Physical stops: the shoulder stalls at -10 deg, the elbow at +160 deg.
    # Neither interferes with the crouch move (shoulder 0 deg, elbow 135 deg).
    stops = {1: math.radians(-10.0), 2: math.radians(160.0)}
    events = []
    events.extend(_run(arm, 4000, stops=stops))
    events.extend(_run(arm, 2000, stops=stops))

    assert arm.resting_action.phase == RestingController.PHASE_COLLAPSED
    assert "torque_off" in events
    # The collapsed pose sits just inside the padded resting limits.
    angles = arm.angles
    assert angles[1] > RESTING_LIMITS[1][0]
    assert angles[2] < RESTING_LIMITS[2][1]
    # The backoffs unloaded each stop by 0.5 deg.
    assert angles[1] == pytest.approx(
        math.radians(-10.0) + RESTING_BACKOFF_ANGLE, abs=0.02)
    assert angles[2] == pytest.approx(
        math.radians(160.0) - RESTING_BACKOFF_ANGLE, abs=0.02)


def test_restore_returns_home_and_restores_locks():
    arm = RoverArmController.from_measured(
        [0.0, math.radians(45), math.radians(-45), 0.0, 0.0])
    arm.toggle_joint_lock(0)
    assert arm.enter_resting()
    # Jump straight to a collapsed pose against the stops.
    arm.resting_action.phase = RestingController.PHASE_COLLAPSED
    collapsed = [0.0, math.radians(-9.5), math.radians(159.5), 0.0, 0.0]
    arm.sync_angles(collapsed, reset_target=True)

    assert arm.toggle_resting()
    assert arm.resting_action.phase == RestingController.PHASE_TORQUE_ON
    events = _run(arm, 2)
    assert "torque_on" in events
    arm.resting_torque_confirmed()
    assert arm.resting_action.phase == RestingController.PHASE_MOVE_OUT

    # Move-out to the crouch, then the homing hand-off.
    events = _run(arm, 4000)
    assert "home" in events
    assert not arm.resting

    # Homing runs (fast on the planned path) and the resting sequence
    # completes once it finishes.
    _run(arm, 4000)
    if arm.homing.active:
        arm.stop_homing()
    assert arm.resting_homing_finished()
    assert arm.resting_action.phase == RestingController.PHASE_DONE
    assert not arm.resting
    # Locks saved on entry are restored at DONE.
    assert arm.locked_joints[0]


def test_startup_collapsed_pose_infers_resting():
    collapsed = [0.0, math.radians(-9.5), math.radians(159.5), 0.0, 0.0]
    arm = RoverArmController.from_measured(collapsed)
    assert arm.resting
    assert arm.limit_mode.name == "Resting"
    assert arm.resting_action.holding
    # R from the inferred hold starts the restore.
    assert arm.toggle_resting()
    assert arm.resting_action.phase == RestingController.PHASE_TORQUE_ON


def test_startup_crouch_pose_still_infers_planar():
    arm = RoverArmController.from_measured(
        [0.0, 0.0, math.radians(135), 0.0, 0.0])
    assert arm.planar
    assert not arm.resting


def test_effective_limits_switch_to_resting():
    from arm_route_planner.kinematics.limiters import PICK_AND_PLACE_MODE

    arm = RoverArmController.from_measured(
        [math.radians(30), math.radians(45), math.radians(-45), 0.0, 0.0])
    assert not arm.planar
    assert arm.effective_limits[1][0] == pytest.approx(
        PICK_AND_PLACE_MODE.limits[1][0])
    assert arm.enter_resting()
    assert arm.effective_limits[1][0] == pytest.approx(
        RESTING_LIMITS[1][0])
    assert arm.effective_limits[2][1] == pytest.approx(
        RESTING_LIMITS[2][1])
