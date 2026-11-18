"""OpenCAN handling for the arm's base, shoulder, and elbow motors.

This handler is joint agnostic: every OpenCAN motor is driven through the
same path, using the calibration entry for that motor.  It knows nothing
about the wrist or gripper, which are driven directly by
:mod:`arm_hardware.arm_hardware_node` with standard Teensy CAN frames.
"""

from enum import IntEnum

from numpy import int32

from arm_constants import (
    ARM_MOTOR_IDX,
    ARM_MOTOR_PARAMS,
    JOINT_MESSAGE_IDS,
    MOTOR_FOR_MESSAGE,
)
from constants.CAN_constants import (
    CAN_MESSAGE_IDS,
    TEENSY_CAN_MESSAGES,
)
from constants.CAN_structs import Message


class OPEN_CAN:

    # these are the values for the ID signal of the CAN messages, they are
    # used to specify the type of the message being sent to the arm motors
    class ID(IntEnum):
        ENABLE = 0
        SET_SPEED = 1
        SEND = 2
        READ = 3

    # these are the values for the MESSAGE_TYPE signal of the CAN messages,
    # the return data from the motors
    class MESSAGE_TYPE:
        ENABLE = int(0x2B)
        SET_SPEED = int(0x23)
        SEND = int(0x23)
        READ_SEND = int(0x40)
        READ_GET = int(0x43)

    # these are the values for the OPCODE_LSB signal of the CAN messages,
    # they specify the type of the message being sent to the arm motors
    class OPCODE_LSB:
        ENABLE = int(0x40)
        SET_SPEED = int(0x81)
        SEND = int(0x7A)
        READ = int(0x64)


class OpenCanHandler:
    """Send and decode OpenCAN frames for any OpenCAN motor."""

    def __init__(self, node):
        self._node = node

    # ------------------------------------------------------------------
    # Sending
    # ------------------------------------------------------------------

    def send_command(
        self,
        message_id: CAN_MESSAGE_IDS,
        value: int32,
        message_type: OPEN_CAN.ID,
    ) -> bool:
        """Pack and send one OpenCAN command frame."""
        can_message = TEENSY_CAN_MESSAGES.get(message_id)
        if can_message is None:
            self._node.get_logger().error(f"Unknown CAN message {message_id}")
            return False

        # lock before modifying the message to prevent multiple threads from
        # modifying the message at the same time and
        with self._node.can_lock:

            # set the message_type code based on the message id
            try:
                if message_type == OPEN_CAN.ID.ENABLE:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.ENABLE
                    )
                elif message_type == OPEN_CAN.ID.SEND:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.SEND
                    )
                elif message_type == OPEN_CAN.ID.READ:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.READ_SEND
                    )
                elif message_type == OPEN_CAN.ID.SET_SPEED:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.SET_SPEED
                    )
            except KeyError:
                self._node.get_logger().error(
                    f"{can_message.name} has no MESSAGE_TYPE signal")
                return False

            # set the OPCODE_LSB of the signal based on the message id
            try:
                if message_type == OPEN_CAN.ID.ENABLE:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.ENABLE
                    )
                elif message_type == OPEN_CAN.ID.SEND:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.SEND
                    )
                elif message_type == OPEN_CAN.ID.READ:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.READ
                    )
                elif message_type == OPEN_CAN.ID.SET_SPEED:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.SET_SPEED
                    )
            except KeyError:
                self._node.get_logger().error(
                    f"{can_message.name} has no OPCODE_LSB signal")
                return False

            # set the OPCODE_MSB to always be the default value
            try:
                can_message.signals["OPCODE_MSB"].reset()
            except KeyError:
                self._node.get_logger().error(
                    f"{can_message.name} has no OPCODE_MSB signal")
                return False

            # set the EMPTY to always be the default value
            try:
                can_message.signals["EMPTY"].reset()
            except KeyError:
                self._node.get_logger().error(
                    f"{can_message.name} has no EMPTY signal")
                return False

            # set the value of the signal to the provided value
            try:
                if message_type == OPEN_CAN.ID.ENABLE:
                    if value == 0:
                        # if value is 0 then disable the arm and set all
                        # target angles to 0
                        can_message.signals["DATA1"].set_value(0)
                    else:
                        # enable position mode and turn on
                        can_message.signals["DATA1"].set_value(int(0x2F))

                    # make sure to reset the other data signals to defaults
                    can_message.signals["DATA2"].set_value(0)
                    can_message.signals["DATA3"].set_value(0)
                    can_message.signals["DATA4"].set_value(0)

                elif (
                    message_type == OPEN_CAN.ID.SEND
                    or message_type == OPEN_CAN.ID.SET_SPEED
                ):
                    can_message.signals["DATA1"].set_value(
                        (value >> 0) & 0xFF
                    )  # least significant byte of the value
                    can_message.signals["DATA2"].set_value(
                        (value >> 8) & 0xFF
                    )  # second least significant byte
                    can_message.signals["DATA3"].set_value(
                        (value >> 16) & 0xFF
                    )  # third least significant byte
                    can_message.signals["DATA4"].set_value(
                        (value >> 24) & 0xFF
                    )  # most significant byte

                elif message_type == OPEN_CAN.ID.READ:
                    can_message.signals["DATA1"].set_value(0)
                    can_message.signals["DATA2"].set_value(0)
                    can_message.signals["DATA3"].set_value(0)
                    can_message.signals["DATA4"].set_value(0)
            except KeyError:
                self._node.get_logger().error(
                    f"{can_message.name} has no DATA signal")
                return False

            # after setting up the message, send it to the arm motors
            return self._node.publish_can(can_message)

    def send_angle(self, motor_idx: ARM_MOTOR_IDX, angle: float) -> None:
        """Send one OpenCAN motor to *angle* [rad]."""
        self.send_command(
            JOINT_MESSAGE_IDS[motor_idx],
            self._radians_to_ticks(motor_idx, angle),
            OPEN_CAN.ID.SEND,
        )

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable the OpenCAN motors, applying their max speeds."""
        value = int32(1) if enabled else int32(0)
        for message_id in JOINT_MESSAGE_IDS.values():
            self.send_command(message_id, value, OPEN_CAN.ID.ENABLE)

        if not enabled:
            return

        for motor_idx, message_id in JOINT_MESSAGE_IDS.items():
            self.send_command(
                message_id,
                ARM_MOTOR_PARAMS[motor_idx].max_speed,
                OPEN_CAN.ID.SET_SPEED,
            )

    def request_positions(self) -> None:
        """Ask every OpenCAN motor to report its current position."""
        for message_id in JOINT_MESSAGE_IDS.values():
            self.send_command(message_id, int32(0), OPEN_CAN.ID.READ)

    # ------------------------------------------------------------------
    # Decoding and conversions
    # ------------------------------------------------------------------

    def handles(self, motor_idx: ARM_MOTOR_IDX) -> bool:
        """Return True if *motor_idx* is driven over OpenCAN."""
        return motor_idx in JOINT_MESSAGE_IDS

    def resolve_message(
        self, message_id: int
    ) -> tuple[ARM_MOTOR_IDX, "OpenCanHandler"] | None:
        """Return the motor and handler for a feedback message, if any."""
        motor_idx = MOTOR_FOR_MESSAGE.get(message_id)
        if motor_idx is None:
            return None
        return motor_idx, self

    def decode_angle(
        self, motor_idx: ARM_MOTOR_IDX, can_message: Message
    ) -> float | None:
        """Return the measured angle [rad] from a READ response, else None."""
        ticks = self._decode_read_ticks(can_message)
        if ticks is None:
            return None
        return self._ticks_to_radians(motor_idx, ticks)

    @staticmethod
    def _decode_read_ticks(can_message: Message) -> int | None:
        """Return the signed tick count from a READ response, else None."""
        try:
            message_type = can_message.signals["MESSAGE_TYPE"].value
            opcode = can_message.signals["OPCODE_LSB"].value
            data1 = can_message.signals["DATA1"].value
            data2 = can_message.signals["DATA2"].value
            data3 = can_message.signals["DATA3"].value
            data4 = can_message.signals["DATA4"].value
        except KeyError:
            return None

        if (
            message_type != OPEN_CAN.MESSAGE_TYPE.READ_GET
            or opcode != OPEN_CAN.OPCODE_LSB.READ
        ):
            return None

        ticks = (data4 << 24) | (data3 << 16) | (data2 << 8) | data1
        if ticks >= 2**31:
            # convert to a signed integer
            ticks -= 2**32
        return ticks

    @staticmethod
    def _radians_to_ticks(motor_idx: ARM_MOTOR_IDX, radians: float) -> int32:
        """Convert a joint angle [rad] to motor ticks for *motor_idx*."""
        params = ARM_MOTOR_PARAMS[motor_idx]
        return int32(radians * params.ticks_per_rad) + params.tick_offset

    @staticmethod
    def _ticks_to_radians(motor_idx: ARM_MOTOR_IDX, ticks: float) -> float:
        """Convert motor ticks for *motor_idx* to a joint angle [rad]."""
        params = ARM_MOTOR_PARAMS[motor_idx]
        return (ticks - params.tick_offset) / params.ticks_per_rad
