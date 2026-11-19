"""OpenCAN (CANopen SDO) handling for the arm's base, shoulder, and elbow.

"OpenCAN" here is one CANopen SDO transfer per standard CAN frame.  The
Teensy CAN templates expose the SDO frame as:

    MESSAGE_TYPE   SDO command specifier (see ``OPEN_CAN.MESSAGE_TYPE``)
    OPCODE_LSB     object index, low byte
    OPCODE_MSB     object index, high byte
    EMPTY          SDO sub-index (0 for every object used here)
    DATA1..DATA4   little-endian payload

The standard objects used by the arm are the CiA 402 drive-profile entries:
controlword, target position, target velocity, profile velocity, position,
velocity, torque, current, DC-link voltage, status, and error code.  Vendor
specific objects (for example motor temperature) can be read with
:meth:`OpenCanHandler.request_object` / :meth:`OpenCanHandler.decode_raw`.

This handler is joint agnostic: every OpenCAN motor is driven through the
same path, using the calibration entry for that motor.  It knows nothing
about the wrist or gripper, which are driven directly by
:mod:`arm_hardware.arm_hardware_node` with standard Teensy CAN frames.
"""

from enum import IntEnum
import math

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


class _ObjectIndex(IntEnum):
    """CiA 402 object dictionary indices used by the arm drives."""

    ERROR_CODE = 0x603F
    CONTROLWORD = 0x6040
    STATUSWORD = 0x6041
    MODES_OF_OPERATION = 0x6060
    MODES_DISPLAY = 0x6061
    POSITION_ACTUAL = 0x6064
    VELOCITY_ACTUAL = 0x606C
    TARGET_TORQUE = 0x6071
    TORQUE_ACTUAL = 0x6077
    CURRENT_ACTUAL = 0x6078
    DC_LINK_VOLTAGE = 0x6079
    TARGET_POSITION = 0x607A
    TARGET_VELOCITY = 0x60FF
    PROFILE_VELOCITY = 0x6081
    PROFILE_ACCELERATION = 0x6083
    PROFILE_DECELERATION = 0x6084

    @property
    def lsb(self) -> int:
        """Low byte of the object index (frame OPCODE_LSB)."""
        return int(self) & 0xFF

    @property
    def msb(self) -> int:
        """High byte of the object index (frame OPCODE_MSB)."""
        return (int(self) >> 8) & 0xFF


class OPEN_CAN:

    # High-level operation requested by the caller.  These select the SDO
    # command specifier and the default object index in the tables below.
    class ID(IntEnum):
        ENABLE = 0
        SET_MAX_SPEED = 1      # profile-velocity limit (0x6081)
        SET_POSITION = 2       # target position (0x607A)
        READ_POSITION = 3      # position actual value (0x6064)
        SET_VELOCITY = 4       # target velocity (0x60FF)
        READ_VELOCITY = 5      # velocity actual value (0x606C)
        READ_TORQUE = 6        # torque actual value (0x6077)
        READ_VOLTAGE = 7       # DC-link circuit voltage (0x6079)
        READ_CURRENT = 8       # current actual value (0x6078)
        READ_STATUS = 9        # statusword (0x6041)
        READ_ERROR = 10        # error code (0x603F)
        READ_MODES = 11        # modes-of-operation display (0x6061)

    # CANopen SDO command specifier: byte 0 of the frame.
    #
    #   client -> drive
    #       0x2F write 1 byte     0x2B write 2 bytes
    #       0x27 write 3 bytes    0x23 write 4 bytes
    #       0x40 initiate upload (read request)
    #
    #   drive -> client
    #       0x60 write acknowledgement
    #       0x4F / 0x4B / 0x47 / 0x43 upload response, 1 / 2 / 3 / 4 bytes
    #       0x80 abort
    class MESSAGE_TYPE:
        WRITE_1 = int(0x2F)
        WRITE_2 = int(0x2B)
        WRITE_3 = int(0x27)
        WRITE_4 = int(0x23)
        READ_REQUEST = int(0x40)
        WRITE_ACK = int(0x60)
        # Expedited upload response with the payload size unspecified.  Per
        # CiA 301 the frame still carries four data bytes.
        READ_4_UNSPECIFIED = int(0x42)
        READ_1 = int(0x4F)
        READ_2 = int(0x4B)
        READ_3 = int(0x47)
        READ_4 = int(0x43)
        ABORT = int(0x80)

    # OpenCAN position targets are single-turn values.  Anything at or past
    # one full revolution is rejected before it becomes a tick count.
    MAX_TARGET_ANGLE_RAD = 2.0 * math.pi

    # CiA 402 object dictionary indices.  OPCODE_LSB / OPCODE_MSB in the
    # frame are simply the little-endian bytes of the selected index.
    OBJECT = _ObjectIndex

    # Controlword value used to enable the drive.  0x0F is the standard
    # CiA 402 "Enable Operation" mask; these arm drives are configured
    # around 0x2F, which also sets operation-mode-specific bit 5.  Keep the
    # hardware-tested value in one place so changing it is explicit.
    ENABLE_CONTROLWORD = 0x2F

    # Derived byte views for the original API.  These are computed from
    # OBJECT so the legacy names cannot drift from the canonical indices.
    class OPCODE_LSB:
        ENABLE = _ObjectIndex.CONTROLWORD.lsb
        SET_MAX_SPEED = _ObjectIndex.PROFILE_VELOCITY.lsb
        SET_POSITION = _ObjectIndex.TARGET_POSITION.lsb
        READ_POSITION = _ObjectIndex.POSITION_ACTUAL.lsb

    @staticmethod
    def index_bytes(object_index: int) -> tuple[int, int]:
        """Return ``(lsb, msb)`` for a CANopen object index."""
        return int(object_index) & 0xFF, (int(object_index) >> 8) & 0xFF

    # Default object index for each high-level operation.
    COMMAND_OBJECT = {
        ID.ENABLE: OBJECT.CONTROLWORD,
        ID.SET_MAX_SPEED: OBJECT.PROFILE_VELOCITY,
        ID.SET_POSITION: OBJECT.TARGET_POSITION,
        ID.READ_POSITION: OBJECT.POSITION_ACTUAL,
        ID.SET_VELOCITY: OBJECT.TARGET_VELOCITY,
        ID.READ_VELOCITY: OBJECT.VELOCITY_ACTUAL,
        ID.READ_TORQUE: OBJECT.TORQUE_ACTUAL,
        ID.READ_VOLTAGE: OBJECT.DC_LINK_VOLTAGE,
        ID.READ_CURRENT: OBJECT.CURRENT_ACTUAL,
        ID.READ_STATUS: OBJECT.STATUSWORD,
        ID.READ_ERROR: OBJECT.ERROR_CODE,
        ID.READ_MODES: OBJECT.MODES_DISPLAY,
    }

    # SDO command specifier for each high-level operation.
    COMMAND_SDO = {
        ID.ENABLE: MESSAGE_TYPE.WRITE_2,
        ID.SET_MAX_SPEED: MESSAGE_TYPE.WRITE_4,
        ID.SET_POSITION: MESSAGE_TYPE.WRITE_4,
        ID.SET_VELOCITY: MESSAGE_TYPE.WRITE_4,
        ID.READ_POSITION: MESSAGE_TYPE.READ_REQUEST,
        ID.READ_VELOCITY: MESSAGE_TYPE.READ_REQUEST,
        ID.READ_TORQUE: MESSAGE_TYPE.READ_REQUEST,
        ID.READ_VOLTAGE: MESSAGE_TYPE.READ_REQUEST,
        ID.READ_CURRENT: MESSAGE_TYPE.READ_REQUEST,
        ID.READ_STATUS: MESSAGE_TYPE.READ_REQUEST,
        ID.READ_ERROR: MESSAGE_TYPE.READ_REQUEST,
        ID.READ_MODES: MESSAGE_TYPE.READ_REQUEST,
    }

    # Number of payload bytes written for each write operation.
    PAYLOAD_SIZE = {
        ID.ENABLE: 2,
        ID.SET_MAX_SPEED: 4,
        ID.SET_POSITION: 4,
        ID.SET_VELOCITY: 4,
    }

    READ_COMMANDS = frozenset({
        ID.READ_POSITION,
        ID.READ_VELOCITY,
        ID.READ_TORQUE,
        ID.READ_VOLTAGE,
        ID.READ_CURRENT,
        ID.READ_STATUS,
        ID.READ_ERROR,
        ID.READ_MODES,
    })

    # Reads whose payload is an unsigned field (statusword, error, voltage,
    # mode).  Everything else (position, velocity, torque, current) is signed.
    UNSIGNED_READS = frozenset({
        ID.READ_VOLTAGE,
        ID.READ_STATUS,
        ID.READ_ERROR,
        ID.READ_MODES,
    })


class OpenCanHandler:
    """Send and decode OpenCAN SDO frames for any OpenCAN motor."""

    def __init__(self, node):
        self._node = node

    # ------------------------------------------------------------------
    # Internal: SDO frame packing
    # ------------------------------------------------------------------

    def _send_command(
        self,
        message_id: CAN_MESSAGE_IDS,
        value: int32 | int,
        message_type: OPEN_CAN.ID,
        object_index: int | None = None,
        subindex: int = 0,
    ) -> bool:
        """Pack and send one OpenCAN SDO frame.

        Args:
            message_id: Teensy CAN template for this motor.
            value: Payload for a write, or 0 for a read.
            message_type: High-level operation from ``OPEN_CAN.ID``.
            object_index: Optional CiA 402 / vendor object override.  When
                omitted the default for *message_type* is used.
            subindex: SDO sub-index (0 for the single-value objects used
                here; vendor multi-entry objects may need another value).
        """
        can_message = TEENSY_CAN_MESSAGES.get(message_id)
        if can_message is None:
            self._node.get_logger().error(f"Unknown CAN message {message_id}")
            return False

        try:
            sdo = OPEN_CAN.COMMAND_SDO[message_type]
            index = (OPEN_CAN.COMMAND_OBJECT[message_type]
                     if object_index is None else object_index)
        except KeyError:
            self._node.get_logger().error(
                f"Unknown OpenCAN operation {message_type}")
            return False

        if message_type in OPEN_CAN.READ_COMMANDS:
            # Upload request: the payload is ignored by the drive.
            payload = [0, 0, 0, 0]
        else:
            payload_size = OPEN_CAN.PAYLOAD_SIZE[message_type]
            if message_type == OPEN_CAN.ID.ENABLE:
                # Controlword: ENABLE_CONTROLWORD enables operation, 0
                # disables the drive.  The enable level is re-sent each tick;
                # a faulted drive must be fault-reset separately.
                value = OPEN_CAN.ENABLE_CONTROLWORD if value else 0
            value = int(value)
            payload = [
                (value >> (8 * byte)) & 0xFF
                for byte in range(payload_size)
            ]
            payload.extend([0] * (4 - payload_size))

        opcode_lsb, opcode_msb = OPEN_CAN.index_bytes(index)

        # Lock the shared CAN template while it is being rewritten.
        with self._node.can_lock:
            try:
                signals = can_message.signals
                signals["MESSAGE_TYPE"].set_value(sdo)
                signals["OPCODE_LSB"].set_value(opcode_lsb)
                signals["OPCODE_MSB"].set_value(opcode_msb)
                signals["EMPTY"].set_value(int(subindex) & 0xFF)
                signals["DATA1"].set_value(payload[0])
                signals["DATA2"].set_value(payload[1])
                signals["DATA3"].set_value(payload[2])
                signals["DATA4"].set_value(payload[3])
            except KeyError as error:
                self._node.get_logger().error(
                    f"{can_message.name} is missing signal {error}")
                return False

            return self._node.publish_can(can_message)

    # ------------------------------------------------------------------
    # Public API: motor commands
    # ------------------------------------------------------------------

    def _get_message_id(
        self, motor_idx: ARM_MOTOR_IDX
    ) -> CAN_MESSAGE_IDS | None:
        """Return the OpenCAN template ID, or log and return None."""
        message_id = JOINT_MESSAGE_IDS.get(motor_idx)
        if message_id is None:
            self._node.get_logger().error(
                f"{motor_idx} is not an OpenCAN motor")
        return message_id

    def send_angle(self, motor_idx: ARM_MOTOR_IDX, angle: float) -> None:
        """Write a target position in ticks to object 0x607A.

        Rejects non-finite angles and any target at or beyond one full
        revolution (|angle| >= 360 deg); OpenCAN positions are single-turn
        values, so a larger command would wrap or overflow unexpectedly.
        """
        if not math.isfinite(angle):
            self._node.get_logger().error(
                f"Refusing non-finite target angle for {motor_idx}")
            return
        if abs(angle) >= OPEN_CAN.MAX_TARGET_ANGLE_RAD:
            self._node.get_logger().error(
                f"Refusing target {math.degrees(angle):.1f} deg for "
                f"{motor_idx}: OpenCAN targets must stay within "
                f"+/-360 deg")
            return
        message_id = self._get_message_id(motor_idx)
        if message_id is None:
            return
        self._send_command(
            message_id,
            self._radians_to_ticks(motor_idx, angle),
            OPEN_CAN.ID.SET_POSITION,
        )

    def set_profile_velocity(
        self, motor_idx: ARM_MOTOR_IDX, velocity: int
    ) -> None:
        """Write the profile-velocity limit in object 0x6081.

        This is the velocity the drive uses while following a target
        position.  It is not a velocity setpoint; use :meth:`set_velocity`
        when the drive is in a velocity mode.
        """
        message_id = self._get_message_id(motor_idx)
        if message_id is None:
            return
        self._send_command(
            message_id,
            int32(velocity),
            OPEN_CAN.ID.SET_MAX_SPEED,
        )

    def set_velocity(self, motor_idx: ARM_MOTOR_IDX, velocity: int) -> None:
        """Write a target velocity in object 0x60FF.

        The value is in the drive's configured velocity units.  This only
        takes effect after object 0x6060 (modes of operation) selects a
        velocity mode.
        """
        message_id = self._get_message_id(motor_idx)
        if message_id is None:
            return
        self._send_command(
            message_id,
            int32(velocity),
            OPEN_CAN.ID.SET_VELOCITY,
        )

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable every OpenCAN motor.

        Enabling writes the CiA 402 controlword (0x6040) and then each
        motor's profile-velocity limit (0x6081).  Disabling writes a zero
        controlword.  The enable level is re-sent on every node tick so a
        frame dropped during an E-stop cannot leave a motor latched off.

        This does not run the full CiA 402 shutdown/switch-on sequence; a
        drive that is faulted must be fault-reset separately.
        """
        value = int32(1) if enabled else int32(0)
        for message_id in JOINT_MESSAGE_IDS.values():
            self._send_command(message_id, value, OPEN_CAN.ID.ENABLE)

        if not enabled:
            return

        # The Teensy transport queues outgoing frames.  If it ever writes
        # directly to a single TX mailbox, these enable and profile-velocity
        # writes would need SDO write-ack spacing.
        for motor_idx in JOINT_MESSAGE_IDS:
            self.set_profile_velocity(
                motor_idx, ARM_MOTOR_PARAMS[motor_idx].max_speed)

    # ------------------------------------------------------------------
    # Public API: read requests
    # ------------------------------------------------------------------

    def request(self, motor_idx: ARM_MOTOR_IDX,
                signal: OPEN_CAN.ID) -> bool:
        """Request one read signal from one OpenCAN motor.

        ``signal`` must be one of the ``OPEN_CAN.ID.READ_*`` values; the
        matching object index and signedness come from ``OPEN_CAN`` and the
        matching decoder is :meth:`decode`.
        """
        if signal not in OPEN_CAN.READ_COMMANDS:
            self._node.get_logger().error(
                f"OpenCAN signal {signal!r} is not a read request")
            return False
        message_id = self._get_message_id(motor_idx)
        if message_id is None:
            return False
        return self._send_command(message_id, int32(0), signal)

    def request_all(self, signal: OPEN_CAN.ID) -> None:
        """Request one read signal from every OpenCAN motor."""
        for motor_idx in JOINT_MESSAGE_IDS:
            self.request(motor_idx, signal)

    def _request_signal(
        self, signal: OPEN_CAN.ID,
        motor_idx: ARM_MOTOR_IDX | None = None,
    ) -> None:
        """Request one signal from one motor, or from all when omitted."""
        if motor_idx is None:
            self.request_all(signal)
        else:
            self.request(motor_idx, signal)

    def request_positions(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request position actual value (0x6064).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_POSITION, motor_idx)

    def request_velocity(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request velocity actual value (0x606C).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_VELOCITY, motor_idx)

    def request_torque(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request torque actual value (0x6077).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_TORQUE, motor_idx)

    def request_voltage(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request DC-link voltage (0x6079).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_VOLTAGE, motor_idx)

    def request_current(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request current actual value (0x6078).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_CURRENT, motor_idx)

    def request_status(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request the statusword (0x6041).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_STATUS, motor_idx)

    def request_error(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request the error code (0x603F).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_ERROR, motor_idx)

    def request_modes(
        self, motor_idx: ARM_MOTOR_IDX | None = None
    ) -> None:
        """Request the modes-of-operation display (0x6061).

        Requests every OpenCAN motor when *motor_idx* is omitted.
        """
        self._request_signal(OPEN_CAN.ID.READ_MODES, motor_idx)

    def request_object(
        self, motor_idx: ARM_MOTOR_IDX, object_index: int, subindex: int = 0
    ) -> bool:
        """Read an arbitrary object, including vendor-specific entries.

        Use this for values that are not part of the standard CiA 402 set,
        such as drive temperature.
        """
        message_id = self._get_message_id(motor_idx)
        if message_id is None:
            return False
        return self._send_command(
            message_id,
            int32(0),
            OPEN_CAN.ID.READ_POSITION,
            object_index=object_index,
            subindex=subindex,
        )

    # ------------------------------------------------------------------
    # Public API: decoding
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

    def decode(self, signal: OPEN_CAN.ID,
               can_message: Message) -> int | None:
        """Decode one read signal from an SDO upload response.

        This is the counterpart to :meth:`request` / :meth:`request_all`:
        the same ``OPEN_CAN.ID.READ_*`` value selects the object index and
        signedness.
        """
        object_index = OPEN_CAN.COMMAND_OBJECT.get(signal)
        if signal not in OPEN_CAN.READ_COMMANDS or object_index is None:
            return None
        return self._decode_sdo_value(
            can_message,
            object_index,
            signed=signal not in OPEN_CAN.UNSIGNED_READS,
            subindex=0,
        )

    def decode_angle(
        self, motor_idx: ARM_MOTOR_IDX, can_message: Message
    ) -> float | None:
        """Return the measured angle [rad] from a READ response, else None."""
        ticks = self.decode(OPEN_CAN.ID.READ_POSITION, can_message)
        if ticks is None:
            return None
        return self._ticks_to_radians(motor_idx, ticks)

    def decode_velocity(self, can_message: Message) -> int | None:
        """Return velocity actual value (0x606C) in drive velocity units.

        The unit is user-defined by the drive configuration (objects 0x6091
        gear ratio and 0x6092 feed constant); no radians/s conversion is
        applied here.
        """
        return self.decode(OPEN_CAN.ID.READ_VELOCITY, can_message)

    def decode_torque(self, can_message: Message) -> int | None:
        """Return torque actual value (0x6077).

        The result is in thousandths of the motor's rated torque
        (1000 = rated torque), per CiA 402.
        """
        return self.decode(OPEN_CAN.ID.READ_TORQUE, can_message)

    def decode_current(self, can_message: Message) -> int | None:
        """Return current actual value (0x6078).

        The result is in thousandths of the motor's rated current
        (1000 = rated current), per CiA 402.
        """
        return self.decode(OPEN_CAN.ID.READ_CURRENT, can_message)

    def decode_voltage_mv(self, can_message: Message) -> int | None:
        """Return DC-link circuit voltage (0x6079) in millivolts."""
        return self.decode(OPEN_CAN.ID.READ_VOLTAGE, can_message)

    def decode_statusword(self, can_message: Message) -> int | None:
        """Return the CiA 402 statusword (0x6041)."""
        return self.decode(OPEN_CAN.ID.READ_STATUS, can_message)

    def decode_error_code(self, can_message: Message) -> int | None:
        """Return the CiA 402 error code (0x603F)."""
        return self.decode(OPEN_CAN.ID.READ_ERROR, can_message)

    def decode_modes(self, can_message: Message) -> int | None:
        """Return the modes-of-operation display (0x6061)."""
        return self.decode(OPEN_CAN.ID.READ_MODES, can_message)

    def decode_raw(
        self, can_message: Message, object_index: int,
        signed: bool = True, subindex: int | None = None,
    ) -> int | None:
        """Decode any SDO upload response, including vendor objects.

        Args:
            can_message: SDO upload response.
            object_index: Expected object index.
            signed: Whether the payload is a signed integer.
            subindex: Optional expected sub-index; None accepts any.
        """
        return self._decode_sdo_value(
            can_message, object_index, signed=signed, subindex=subindex)

    def _decode_sdo_value(
        self, can_message: Message, object_index: int | None = None,
        signed: bool = True, subindex: int | None = None,
    ) -> int | None:
        """Return the integer payload of an SDO upload response.

        The response command specifier gives the payload width:
        0x4F = 1 byte, 0x4B = 2 bytes, 0x47 = 3 bytes, 0x43 = 4 bytes, and
        0x42 = 4 bytes with the size unspecified.  ``object_index`` and
        ``subindex`` filter unrelated responses; pass None to accept any.
        """
        try:
            message_type = can_message.signals["MESSAGE_TYPE"].value
            opcode_lsb = can_message.signals["OPCODE_LSB"].value
            opcode_msb = can_message.signals["OPCODE_MSB"].value
            message_subindex = can_message.signals["EMPTY"].value
            data = [
                can_message.signals[f"DATA{index}"].value
                for index in range(1, 5)
            ]
        except KeyError:
            return None

        index = opcode_lsb | (opcode_msb << 8)
        if message_type == OPEN_CAN.MESSAGE_TYPE.ABORT:
            if object_index is not None and index != int(object_index):
                return None
            abort_code = sum(
                (data[byte] & 0xFF) << (8 * byte) for byte in range(4))
            self._node.get_logger().warning(
                f"SDO abort 0x{abort_code:08X} for object "
                f"0x{index:04X}")
            return None

        sizes = {
            OPEN_CAN.MESSAGE_TYPE.READ_1: 1,
            OPEN_CAN.MESSAGE_TYPE.READ_2: 2,
            OPEN_CAN.MESSAGE_TYPE.READ_3: 3,
            OPEN_CAN.MESSAGE_TYPE.READ_4: 4,
            OPEN_CAN.MESSAGE_TYPE.READ_4_UNSPECIFIED: 4,
        }
        size = sizes.get(message_type)
        if size is None:
            return None

        if object_index is not None and index != int(object_index):
            return None
        if (subindex is not None
                and message_subindex != int(subindex)):
            return None

        value = sum(
            (data[byte] & 0xFF) << (8 * byte) for byte in range(size))
        if signed:
            sign_bit = 1 << (8 * size - 1)
            if value & sign_bit:
                value -= 1 << (8 * size)
        return value

    @staticmethod
    def _radians_to_ticks(motor_idx: ARM_MOTOR_IDX, radians: float) -> int:
        """Convert a joint angle [rad] to motor ticks for *motor_idx*."""
        if not math.isfinite(radians):
            raise ValueError("joint angle must be finite")
        params = ARM_MOTOR_PARAMS[motor_idx]
        return (int(radians * params.ticks_per_rad)
                + int(params.tick_offset))

    @staticmethod
    def _ticks_to_radians(motor_idx: ARM_MOTOR_IDX, ticks: float) -> float:
        """Convert motor ticks for *motor_idx* to a joint angle [rad]."""
        params = ARM_MOTOR_PARAMS[motor_idx]
        return (ticks - params.tick_offset) / params.ticks_per_rad
