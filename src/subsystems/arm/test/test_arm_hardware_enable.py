"""Tests for the arm hardware enable/disable latch behaviour."""

from std_msgs.msg import Bool

from arm_hardware.arm_hardware_node import ArmHardwareNode


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


class _Dummy:
    """Minimal stand-in exposing the attributes one latch path touches."""

    def __init__(self):
        self.arm_enabled = True
        self.held = False
        self.forgot = False
        self.open_can_calls = []
        self.logger = _Logger()
        self._ArmHardwareNode__estop = False
        self._ArmHardwareNode__enable_ignored_logged = False
        self._ArmHardwareNode__torque_off = False
        self._ArmHardwareNode__torque_ignored_logged = False
        self._ArmHardwareNode__open_can = self

    # The node calls these through their name-mangled attributes.
    def _ArmHardwareNode__target_measured_positions(self):
        self.held = True

    def _ArmHardwareNode__forget_targets(self):
        self.forgot = True

    def set_enabled(self, enabled):
        self.open_can_calls.append(enabled)

    def get_logger(self):
        return self.logger


def _disable(dummy):
    ArmHardwareNode._ArmHardwareNode__on_arm_enable_received(
        dummy, Bool(data=False))


def test_disable_holds_the_pose_and_keeps_opencan_torque():
    dummy = _Dummy()

    _disable(dummy)

    assert dummy.arm_enabled is False
    # The measured pose is commanded before the node stops accepting
    # targets, so an in-flight move cannot continue on a disabled arm.
    assert dummy.held
    assert dummy.forgot
    # Crucially, no OpenCAN enable frame was sent: a Teensy heartbeat loss
    # must not drop the arm under gravity.
    assert dummy.open_can_calls == []


def test_repeated_disable_is_a_no_op():
    dummy = _Dummy()
    _disable(dummy)
    dummy.held = False

    _disable(dummy)

    assert not dummy.held
