"""Simulated arm Teensy for the wrist joints only.

This node stands in for the Teensy's wrist and gripper channels when there is
no wrist hardware on the CAN bus. The OpenCAN motors (base, shoulder, elbow)
are not touched: ``arm_hardware_node`` keeps owning them.

It publishes the ``Teensy_Heartbeat`` that lets ``master_node`` enable the arm,
follows the enable/E-stop levels, accepts the wrist target topics and the raw
``MOVE_*`` command frames, and reports the wrist joints at ``0 rad``.
"""

import struct
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from arm_constants import ARM_MOTOR_IDX, ESTOP_TOPIC, TARGET_ANGLE_TOPICS
from constants.CAN_constants import (
    CAN_MESSAGE_IDS,
    TEENSY_CAN_MESSAGES,
    SubsystemsIDs,
)
from constants.CAN_structs import Message, Signal
from constants.can_encoding import TeensyCommunication
from custom_interfaces.msg import Can

# Heartbeat and wrist feedback rates.
POSITION_PERIOD_SEC = 0.05
HEARTBEAT_PERIOD_SEC = 0.1

# The wrist joints this node owns.
WRIST_JOINTS = (
    ARM_MOTOR_IDX.BEND_WRIST,
    ARM_MOTOR_IDX.TWIST_WRIST,
)
# Wrist feedback frames: joint -> (READ message id, signal name).
WRIST_READ_FRAMES = {
    ARM_MOTOR_IDX.BEND_WRIST: (CAN_MESSAGE_IDS.READ_WRIST_BEND, "Position"),
    ARM_MOTOR_IDX.TWIST_WRIST: (CAN_MESSAGE_IDS.READ_WRIST_TWIST, "Position"),
}
# Raw wrist command frames.
WRIST_COMMAND_FRAMES = {
    CAN_MESSAGE_IDS.MOVE_WRIST: "Position_Bend",
}


def build_can_frame(template: Message, values: dict[str, object]) -> Message:
    """Copy a CAN template and set the named signal values."""
    signals = {
        name: Signal(signal.type, signal.default_value)
        for name, signal in template.signals.items()
    }
    frame = Message(
        id=template.id,
        name=template.name,
        subsystem=template.subsystem,
        signals=signals,
        isforJetson=template.isforJetson,
    )
    for name, value in values.items():
        frame.signals[name].set_value(value)
    return frame


class SimulatedArmTeensy(Node):
    """Publish the arm heartbeat and wrist feedback; leave OpenCAN alone."""

    def __init__(self) -> None:
        super().__init__("simulated_arm_teensy")

        self.__enabled = False
        self.__estop = False
        self.__heartbeat_tick = 0

        # Wrist feedback stays at zero. The base/shoulder/elbow READ frames
        # belong to arm_hardware_node, so this node never publishes them.
        self.__angles = {idx: 0.0 for idx in WRIST_JOINTS}

        self.__publishers = {}
        for message_id in (message_id for message_id, _ in
                           WRIST_READ_FRAMES.values()):
            message = TEENSY_CAN_MESSAGES[message_id]
            self.__publishers[message_id] = self.create_publisher(
                Can, message.topic_name, 10)
        self.__publishers[CAN_MESSAGE_IDS.TEENSY_HEARTBEAT] = \
            self.create_publisher(
                Can,
                TEENSY_CAN_MESSAGES[
                    CAN_MESSAGE_IDS.TEENSY_HEARTBEAT].topic_name,
                10,
            )

        # The master owns the enable decision; follow its level.
        self.create_subscription(
            Bool, "/ARM/STATE/ENABLED", self.__on_enable_received, 10)
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)

        # Wrist targets are accepted and held at zero. OpenCAN targets are
        # owned by arm_hardware_node, so they are not subscribed here.
        for idx in WRIST_JOINTS:
            self.create_subscription(
                Float32,
                TARGET_ANGLE_TOPICS[idx],
                self.__on_wrist_target_received,
                10,
            )

        # Raw Teensy command frames, so the enable path is exercised too.
        self.create_subscription(
            Can,
            TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.ENABLE_ARM].topic_name,
            self.__on_enable_arm_frame, 10)
        for message_id in WRIST_COMMAND_FRAMES:
            self.create_subscription(
                Can,
                TEENSY_CAN_MESSAGES[message_id].topic_name,
                self.__on_wrist_command_frame,
                10,
            )

        self.create_timer(POSITION_PERIOD_SEC, self.__publish_positions)
        self.create_timer(HEARTBEAT_PERIOD_SEC, self.__publish_heartbeat)
        self.get_logger().info(
            "Simulated arm Teensy running; wrist joints pinned at 0 rad")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def __on_enable_received(self, msg: Bool) -> None:
        self.__set_enabled(bool(msg.data))

    def __on_estop_received(self, msg: Bool) -> None:
        self.__estop = bool(msg.data)
        if self.__estop:
            self.__set_enabled(False)

    def __on_enable_arm_frame(self, msg: Can) -> None:
        """Mirror the raw ENABLE_ARM frame as the Teensy would."""
        signals = self.__decode(msg)
        if signals is None or "enable" not in signals:
            return
        self.__set_enabled(bool(signals["enable"]) and not self.__estop)

    def __on_wrist_target_received(self, msg: Float32) -> None:
        """Accept wrist targets; the wrist stays pinned at zero."""
        _ = msg

    def __on_wrist_command_frame(self, msg: Can) -> None:
        """Accept raw wrist commands; the wrist stays pinned at zero."""
        self.__decode(msg)

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------

    def __set_enabled(self, enabled: bool) -> None:
        if enabled == self.__enabled:
            return
        self.__enabled = enabled
        self.get_logger().info(f"Arm {'enabled' if enabled else 'disabled'}")

    def __publish_positions(self) -> None:
        for idx, (message_id, signal_name) in WRIST_READ_FRAMES.items():
            self.__publish_frame(message_id, {signal_name: self.__angles[idx]})

    def __publish_heartbeat(self) -> None:
        self.__heartbeat_tick = (self.__heartbeat_tick + 1) % 0xFFFFFFFF
        self.__publish_frame(
            CAN_MESSAGE_IDS.TEENSY_HEARTBEAT,
            {
                "source": SubsystemsIDs.ARM,
                "timestamp": self.__heartbeat_tick,
                "enabled": 1 if self.__enabled else 0,
            },
        )

    def __publish_frame(self, message_id: CAN_MESSAGE_IDS,
                        values: dict[str, object]) -> None:
        template = TEENSY_CAN_MESSAGES[message_id]
        frame = build_can_frame(template, values)
        packet = TeensyCommunication.encode_can_message(frame)
        if packet is None:
            self.get_logger().error(f"Failed to encode {template.name}")
            return
        publisher = self.__publishers.get(message_id)
        if publisher is not None:
            try:
                publisher.publish(packet)
            except RCLError:
                # Shutdown can tear the context down between ticks.
                if rclpy.ok():
                    raise

    def __decode(self, msg: Can) -> dict[str, object] | None:
        """Return the signal values of a raw CAN frame, if it decodes."""
        try:
            signals = []
            pack_format = "="
            template = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(msg.id)]
            for signal in template.signals.values():
                signals.append(signal.name)
                pack_format += signal.type.pack_format
            values = struct.unpack_from(pack_format, bytes(msg.buf), 0)
            return dict(zip(signals, values))
        except Exception as exc:
            self.get_logger().warning(f"Ignoring undecodable CAN frame: {exc}")
            return None


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = SimulatedArmTeensy()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
