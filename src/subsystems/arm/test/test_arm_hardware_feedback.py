"""Tests for periodic arm measured-angle refresh."""

from arm_constants import ARM_MOTOR_IDX
from arm_hardware.arm_hardware_node import ArmHardwareNode


class _Publisher:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


def _dummy(angles):
    """Minimal stand-in for the attributes the refresh touches."""
    dummy = _Dummy()
    dummy._ArmHardwareNode__motors_actual_angles = angles
    dummy._ArmHardwareNode__angle_publishers = {
        motor_idx: _Publisher() for motor_idx in ARM_MOTOR_IDX
    }
    return dummy


class _Dummy:
    pass


def test_refresh_publishes_every_seen_joint():
    angles = [float("nan")] * len(ARM_MOTOR_IDX)
    angles[ARM_MOTOR_IDX.BEND_WRIST] = 0.0
    angles[ARM_MOTOR_IDX.TWIST_WRIST] = 0.0
    dummy = _dummy(angles)

    ArmHardwareNode._ArmHardwareNode__publish_measured_angles(dummy)

    publishers = dummy._ArmHardwareNode__angle_publishers
    assert publishers[ARM_MOTOR_IDX.BEND_WRIST].values == [0.0]
    assert publishers[ARM_MOTOR_IDX.TWIST_WRIST].values == [0.0]


def test_refresh_skips_joints_without_feedback():
    angles = [float("nan")] * len(ARM_MOTOR_IDX)
    angles[ARM_MOTOR_IDX.BASE] = 0.25
    dummy = _dummy(angles)

    ArmHardwareNode._ArmHardwareNode__publish_measured_angles(dummy)

    publishers = dummy._ArmHardwareNode__angle_publishers
    assert publishers[ARM_MOTOR_IDX.BASE].values == [0.25]
    for motor_idx in ARM_MOTOR_IDX:
        if motor_idx != ARM_MOTOR_IDX.BASE:
            assert publishers[motor_idx].values == []
