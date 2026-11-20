"""Debug sniffer for the arm joint and controller state topics.

This node is intentionally not part of ``main_launch.xml`` or
``arm_launch.xml``: it is noisy, read-only, and only useful while debugging.
Run it explicitly with::

    ros2 run arm arm_sniffer_node
    ros2 run arm sniffer
    ros2 run arm ASN

Every second it prints one compact snapshot: an aligned status/flags block
and a joint table.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, UInt8, UInt32

from arm_constants import (
    ARM_CONTROL_SOURCE_TOPIC,
    ARM_ENABLED_TOPIC,
    ARM_FLAG_NAMES,
    ARM_JOINT_NAMES,
    ARM_JOINTS,
    ARM_OPERATION_MODE_LABELS,
    ARM_STATE_TOPICS,
    ESTOP_TOPIC,
    JOINT_CURRENT_TOPICS,
    JOINT_TARGET_TOPICS,
    SNIFFER_PERIOD_SEC,
    control_source_label,
    parse_control_source,
)
from rover_protocol.arm_errors import error_text


def format_angle(value: float | None) -> str:
    """Format one angle [rad] in degrees, or a waiting marker."""
    if value is None:
        return "waiting"
    if math.isnan(value):
        return "nan"
    return f"{math.degrees(value):8.2f}"


def format_mask(mask: int | None, names: tuple[str, ...]) -> str:
    """Expand a joint bitmask into a comma-separated list of names."""
    if mask is None:
        return "waiting"
    selected = [
        name for index, name in enumerate(names) if mask & (1 << index)]
    return ", ".join(selected) if selected else "none"


class ArmSnifferNode(Node):
    """Read-only node that prints arm angles and controller state."""

    def __init__(self):
        """Subscribe to every arm debug topic and start the print timer."""
        super().__init__("arm_sniffer_node")

        self.__current = [None] * len(ARM_JOINTS)
        self.__target = [None] * len(ARM_JOINTS)
        self.__arm_enabled = None
        self.__estop = None
        self.__control_source = None
        self.__flags = {name: None for name in ARM_FLAG_NAMES}
        self.__locked_joints = None
        self.__homing_joints = None
        self.__operation_mode = None
        self.__error = None

        for index, joint in enumerate(ARM_JOINTS):
            self.create_subscription(
                msg_type=Float32,
                topic=JOINT_CURRENT_TOPICS[joint],
                callback=self.__make_angle_callback(self.__current, index),
                qos_profile=10,
            )
            self.create_subscription(
                msg_type=Float32,
                topic=JOINT_TARGET_TOPICS[joint],
                callback=self.__make_angle_callback(self.__target, index),
                qos_profile=10,
            )

        self.create_subscription(
            msg_type=Bool,
            topic=ARM_ENABLED_TOPIC,
            callback=self.__on_enabled_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Bool,
            topic=ESTOP_TOPIC,
            callback=self.__on_estop_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt8,
            topic=ARM_CONTROL_SOURCE_TOPIC,
            callback=self.__on_control_source_received,
            qos_profile=10,
        )
        for name in ARM_FLAG_NAMES:
            self.create_subscription(
                msg_type=Bool,
                topic=ARM_STATE_TOPICS["flags"][name],
                callback=self.__make_flag_callback(name),
                qos_profile=10,
            )
        self.create_subscription(
            msg_type=UInt8,
            topic=ARM_STATE_TOPICS["locked_joints"],
            callback=self.__on_locked_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt8,
            topic=ARM_STATE_TOPICS["homing_joints"],
            callback=self.__on_homing_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt8,
            topic=ARM_STATE_TOPICS["operation_mode"],
            callback=self.__on_mode_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt32,
            topic=ARM_STATE_TOPICS["error"],
            callback=self.__on_error_received,
            qos_profile=10,
        )

        self.create_timer(SNIFFER_PERIOD_SEC, self.__print_report)

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def __make_angle_callback(self, store: list, index: int):
        """Build a callback that stores one joint angle [rad]."""
        def callback(msg: Float32):
            store[index] = float(msg.data)
        return callback

    def __make_flag_callback(self, name: str):
        """Build a callback that stores one Bool state flag."""
        def callback(msg: Bool):
            self.__flags[name] = bool(msg.data)
        return callback

    def __on_enabled_received(self, msg: Bool):
        self.__arm_enabled = bool(msg.data)

    def __on_estop_received(self, msg: Bool):
        self.__estop = bool(msg.data)

    def __on_control_source_received(self, msg: UInt8):
        self.__control_source = parse_control_source(msg.data)

    def __on_locked_received(self, msg: UInt8):
        self.__locked_joints = int(msg.data)

    def __on_homing_received(self, msg: UInt8):
        self.__homing_joints = int(msg.data)

    def __on_mode_received(self, msg: UInt8):
        self.__operation_mode = int(msg.data)

    def __on_error_received(self, msg: UInt32):
        self.__error = int(msg.data)

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def __print_report(self):
        """Print one column-aligned snapshot of the arm debug state."""
        left_rows = [
            ("enabled", self.__format_bool(self.__arm_enabled)),
            ("e-stop", self.__format_bool(self.__estop)),
            ("control", self.__format_control_source()),
            ("mode", self.__format_mode()),
        ]
        right_rows = [
            ("flags", self.__format_flags()),
            ("locked", format_mask(self.__locked_joints, ARM_JOINT_NAMES)),
            ("homing", format_mask(self.__homing_joints, ARM_JOINT_NAMES)),
            ("error", self.__format_error()),
        ]

        left_cells = [
            f"{label:<8}: {value}" for label, value in left_rows]
        left_width = max(len(cell) for cell in left_cells) + 2

        lines = []
        for index, (left_cell, (right_label, right_value)) in enumerate(
                zip(left_cells, right_rows)):
            prefix = "ARM   " if index == 0 else "      "
            lines.append(
                f"{prefix}{left_cell:<{left_width}}"
                f"{right_label:<7}: {right_value}")

        lines.append(
            f"      {'joint':<12} {'cur(deg)':>8} "
            f"{'tgt(deg)':>8} {'dlt(deg)':>8}")

        for index, joint in enumerate(ARM_JOINTS):
            current = self.__current[index]
            target = self.__target[index]
            delta = (
                target - current
                if current is not None and target is not None
                else None
            )
            lines.append(
                f"      {joint:<12} {format_angle(current):>8} "
                f"{format_angle(target):>8} {format_angle(delta):>8}"
            )

        self.get_logger().info("\n".join(lines))

    @staticmethod
    def __format_bool(value: bool | None) -> str:
        if value is None:
            return "waiting"
        return "True" if value else "False"

    def __format_control_source(self) -> str:
        if self.__control_source is None:
            return "waiting"
        return control_source_label(self.__control_source)

    def __format_mode(self) -> str:
        if self.__operation_mode is None:
            return "waiting"
        name = ARM_OPERATION_MODE_LABELS.get(
            self.__operation_mode, f"UNKNOWN ({self.__operation_mode})"
        )
        return f"{name} ({self.__operation_mode})"

    def __format_flags(self) -> str:
        active = [name for name in ARM_FLAG_NAMES if self.__flags[name]]
        if all(value is None for value in self.__flags.values()):
            return "waiting"
        return ", ".join(active) if active else "none"

    def __format_error(self) -> str:
        if self.__error is None:
            return "waiting"
        code = (self.__error >> 8) & 0xFF
        argument = self.__error & 0xFF
        if code == 0:
            return "none"
        text = error_text(code, argument)
        return f"{text} (code={code}, arg={argument})"

    def run(self):
        """Spin the node until it is shut down."""
        self.get_logger().info("Arm sniffer started")
        rclpy.spin(self)
        self.get_logger().info("Arm sniffer stopped")


def main():
    """Run the arm sniffer node."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = ArmSnifferNode()
        node.run()
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
