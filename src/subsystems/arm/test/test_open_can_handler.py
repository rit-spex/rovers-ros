"""Tests for OpenCAN SDO framing and decoding."""

import math
import threading

import pytest

from arm_constants import (
    ARM_MOTOR_IDX,
    ARM_MOTOR_PARAMS,
    JOINT_MESSAGE_IDS,
)
from arm_hardware.open_can_handler import OPEN_CAN, OpenCanHandler
from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES


class _Logger:
    def __init__(self):
        self.messages = []

    def error(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self.can_lock = threading.Lock()
        self.logger = _Logger()
        self.sent = None
        self.published = []

    def get_logger(self):
        return self.logger

    def publish_can(self, message):
        self.sent = message
        self.published.append(message)
        return True


def _reply(message_type, object_index, value, size, subindex=0):
    """Build a drive-to-host SDO upload response from the READ template."""
    template = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.READ_BASE]
    message = type(template)(
        id=template.id,
        name=template.name,
        subsystem=template.subsystem,
        signals={
            name: type(signal)(signal.type, signal.default_value)
            for name, signal in template.signals.items()
        },
        isforJetson=template.isforJetson,
    )
    lsb, msb = OPEN_CAN.index_bytes(object_index)
    message.signals["MESSAGE_TYPE"].set_value(message_type)
    message.signals["OPCODE_LSB"].set_value(lsb)
    message.signals["OPCODE_MSB"].set_value(msb)
    message.signals["EMPTY"].set_value(subindex)
    for byte in range(size):
        message.signals[f"DATA{byte + 1}"].set_value(
            (value >> (8 * byte)) & 0xFF)
    return message


def _payload(message):
    return [message.signals[f"DATA{i}"].value for i in range(1, 5)]


def test_send_angle_writes_target_position():
    node = _Node()
    handler = OpenCanHandler(node)
    handler.send_angle(ARM_MOTOR_IDX.BASE, 0.5)

    params = ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE]
    ticks = int(0.5 * params.ticks_per_rad) + params.tick_offset
    message = node.sent
    assert message.signals["MESSAGE_TYPE"].value == (
        OPEN_CAN.MESSAGE_TYPE.WRITE_4)
    assert message.signals["OPCODE_LSB"].value == 0x7A
    assert message.signals["OPCODE_MSB"].value == 0x60
    assert _payload(message) == [
        (ticks >> (8 * byte)) & 0xFF for byte in range(4)]


def test_set_velocity_writes_target_velocity_object():
    node = _Node()
    handler = OpenCanHandler(node)
    handler.set_velocity(ARM_MOTOR_IDX.BASE, 1234)

    message = node.sent
    assert message.signals["MESSAGE_TYPE"].value == (
        OPEN_CAN.MESSAGE_TYPE.WRITE_4)
    assert message.signals["OPCODE_LSB"].value == 0xFF
    assert message.signals["OPCODE_MSB"].value == 0x60
    assert _payload(message) == [0xD2, 0x04, 0x00, 0x00]


def test_request_voltage_uses_the_upload_object():
    node = _Node()
    handler = OpenCanHandler(node)
    handler.request_voltage()

    assert len(node.published) == len(JOINT_MESSAGE_IDS)
    message = node.sent
    assert message.signals["MESSAGE_TYPE"].value == (
        OPEN_CAN.MESSAGE_TYPE.READ_REQUEST)
    assert message.signals["OPCODE_LSB"].value == 0x79
    assert message.signals["OPCODE_MSB"].value == 0x60
    assert _payload(message) == [0, 0, 0, 0]


def test_request_can_target_one_motor():
    node = _Node()
    handler = OpenCanHandler(node)
    handler.request_voltage(ARM_MOTOR_IDX.ELBOW)

    assert len(node.published) == 1
    message = node.sent
    assert message.id == JOINT_MESSAGE_IDS[ARM_MOTOR_IDX.ELBOW]
    assert message.signals["OPCODE_LSB"].value == 0x79
    assert message.signals["OPCODE_MSB"].value == 0x60


def test_request_object_can_read_a_vendor_subindex():
    node = _Node()
    handler = OpenCanHandler(node)
    assert handler.request_object(ARM_MOTOR_IDX.BASE, 0x2004, subindex=2)

    message = node.sent
    assert message.signals["OPCODE_LSB"].value == 0x04
    assert message.signals["OPCODE_MSB"].value == 0x20
    assert message.signals["EMPTY"].value == 2


def test_request_and_decode_use_the_same_signal():
    node = _Node()
    handler = OpenCanHandler(node)
    assert handler.request(ARM_MOTOR_IDX.BASE, OPEN_CAN.ID.READ_TORQUE)

    message = node.sent
    assert message.signals["OPCODE_LSB"].value == 0x77
    assert message.signals["OPCODE_MSB"].value == 0x60

    reply = _reply(
        OPEN_CAN.MESSAGE_TYPE.READ_2,
        OPEN_CAN.OBJECT.TORQUE_ACTUAL,
        -250,
        2,
    )
    assert handler.decode(OPEN_CAN.ID.READ_TORQUE, reply) == -250
    assert handler.decode(OPEN_CAN.ID.READ_VOLTAGE, reply) is None


def test_decode_position_matches_the_tick_calibration():
    handler = OpenCanHandler(_Node())
    params = ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE]
    ticks = int(0.72 * params.ticks_per_rad) + params.tick_offset
    message = _reply(
        OPEN_CAN.MESSAGE_TYPE.READ_4,
        OPEN_CAN.OBJECT.POSITION_ACTUAL,
        ticks,
        4,
    )
    angle = handler.decode_angle(ARM_MOTOR_IDX.BASE, message)
    # One tick of quantization is the most the encode/decode round trip can
    # lose, so compare at tick resolution rather than sub-tick precision.
    assert angle == pytest.approx(0.72, abs=1.0 / abs(params.ticks_per_rad))


def test_decode_torque_is_signed_per_mille():
    handler = OpenCanHandler(_Node())
    message = _reply(
        OPEN_CAN.MESSAGE_TYPE.READ_2,
        OPEN_CAN.OBJECT.TORQUE_ACTUAL,
        -250,
        2,
    )
    assert handler.decode_torque(message) == -250


def test_decode_voltage_is_millivolts():
    handler = OpenCanHandler(_Node())
    message = _reply(
        OPEN_CAN.MESSAGE_TYPE.READ_2,
        OPEN_CAN.OBJECT.DC_LINK_VOLTAGE,
        3600,
        2,
    )
    assert handler.decode_voltage_mv(message) == 3600


def test_decode_rejects_a_different_object():
    handler = OpenCanHandler(_Node())
    message = _reply(
        OPEN_CAN.MESSAGE_TYPE.READ_2,
        OPEN_CAN.OBJECT.TORQUE_ACTUAL,
        100,
        2,
    )
    assert handler.decode_voltage_mv(message) is None


def test_decode_accepts_unspecified_size_response():
    handler = OpenCanHandler(_Node())
    message = _reply(
        OPEN_CAN.MESSAGE_TYPE.READ_4_UNSPECIFIED,
        OPEN_CAN.OBJECT.TORQUE_ACTUAL,
        -250,
        4,
    )
    assert handler.decode(OPEN_CAN.ID.READ_TORQUE, message) == -250


def test_decode_raw_can_use_unsigned_and_check_subindex():
    handler = OpenCanHandler(_Node())
    message = _reply(OPEN_CAN.MESSAGE_TYPE.READ_2, 0x2004, 0xFFFF, 2,
                     subindex=2)
    assert handler.decode_raw(
        message, 0x2004, signed=False, subindex=2) == 65535
    assert handler.decode_raw(message, 0x2004, subindex=2) == -1
    assert handler.decode_raw(message, 0x2004, subindex=1) is None


def test_abort_response_is_logged():
    node = _Node()
    handler = OpenCanHandler(node)
    message = _reply(
        OPEN_CAN.MESSAGE_TYPE.ABORT,
        OPEN_CAN.OBJECT.TORQUE_ACTUAL,
        0x06020000,
        4,
    )
    assert handler.decode(OPEN_CAN.ID.READ_TORQUE, message) is None
    assert any("SDO abort" in text for text in node.logger.messages)


def test_invalid_motor_is_rejected():
    node = _Node()
    handler = OpenCanHandler(node)
    assert not handler.request(
        ARM_MOTOR_IDX.BEND_WRIST, OPEN_CAN.ID.READ_TORQUE)
    handler.send_angle(ARM_MOTOR_IDX.BEND_WRIST, 0.1)
    assert node.sent is None
    assert node.logger.messages


def test_non_finite_angle_is_rejected():
    node = _Node()
    handler = OpenCanHandler(node)
    handler.send_angle(ARM_MOTOR_IDX.BASE, float("nan"))
    assert node.sent is None
    assert node.logger.messages


def test_send_angle_rejects_a_full_turn():
    node = _Node()
    handler = OpenCanHandler(node)
    handler.send_angle(ARM_MOTOR_IDX.BASE, math.tau)
    assert node.sent is None
    assert any("360" in text for text in node.logger.messages)

    handler.send_angle(ARM_MOTOR_IDX.BASE, -math.tau - 0.1)
    assert node.sent is None


def test_send_angle_allows_just_inside_a_full_turn():
    node = _Node()
    handler = OpenCanHandler(node)
    handler.send_angle(ARM_MOTOR_IDX.BASE, math.tau - 0.01)
    assert node.sent is not None
