#!/usr/bin/env python3

from __future__ import annotations

import sys
import os
from pathlib import Path

import rclpy
import rclpy.publisher
import rclpy.subscription
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import (
    Bool,
    Float32,
    Int16,
    String,
    UInt8,
    UInt8MultiArray,
    UInt16,
    UInt32,
)
from custom_interfaces.msg import SpaceMouseAxes

XBEE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
TELEOP_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
TELEOP_MESSAGES = {"spacemouse", "xbox"}


def _ensure_local_package_path() -> None:
    package_dir = Path(__file__).resolve().parent
    package_dir_str = str(package_dir)
    if package_dir_str not in sys.path:
        sys.path.insert(0, package_dir_str)


_ensure_local_package_path()

from command_codes import CONSTANTS
from encoding import MessageEncoder

# Arm key actions arrive as per-action press states.  The physical key map
# lives on the basestation; this bridge only translates the semantic action
# names into controller commands.  These dicts are the single source for the
# signal -> command mapping and the held-action set.
ARM_STATE_JUST_PRESSED = 1
ARM_STATE_HELD = 2

# Every operator action: arm_state signal -> rover command.
ARM_STATE_COMMANDS = {
    "flip": "flip",
    "zenith": "zenith",
    "home": "home",
    "position_drive": "position_drive",
    "joint_drive": "joint_drive",
    "pnp_normal": "pnp_normal",
    "toggle_planar": "toggle_planar",
    "guide_orientation": "guide_orientation",
    "toggle_position_lock": "toggle_position_lock",
    "toggle_orientation_lock": "toggle_orientation_lock",
    "toggle_joint_lock_0": "toggle_joint_lock:0",
    "toggle_joint_lock_1": "toggle_joint_lock:1",
    "toggle_joint_lock_2": "toggle_joint_lock:2",
    "toggle_joint_lock_3": "toggle_joint_lock:3",
    "toggle_joint_lock_4": "toggle_joint_lock:4",
    "home_joint_0": "home_joint:0",
    "home_joint_1": "home_joint:1",
    "home_joint_2": "home_joint:2",
    "home_joint_3": "home_joint:3",
    "home_joint_4": "home_joint:4",
    "gripper_up": "gripper_up",
    "gripper_down": "gripper_down",
    "solenoid_strike": "fire_solenoid",
}

# Actions that repeat while the key is held.  These are forwarded on press
# and on every held state.
ARM_STATE_HELD_COMMANDS = {
    "home": "home",
    "home_joint_0": "home_joint:0",
    "home_joint_1": "home_joint:1",
    "home_joint_2": "home_joint:2",
    "home_joint_3": "home_joint:3",
    "home_joint_4": "home_joint:4",
    "gripper_up": "gripper_up",
    "gripper_down": "gripper_down",
}


def _format_arm_command_keys() -> str:
    """Return the multi-line key readout shown at startup."""
    lines = ["Arm command keys:"]
    for signal, command in ARM_STATE_COMMANDS.items():
        trigger = "held" if signal in ARM_STATE_HELD_COMMANDS else "press"
        lines.append(f"  {signal:<24} -> {command} ({trigger})")
    return "\n".join(lines)


class Basestation(Node):
    __encoder: MessageEncoder
    __publishers: dict[int, dict[str, rclpy.publisher.Publisher]]

    def __init__(self):
        super().__init__("Basestation")

        # Default to enabled � the basestation-ROS link exists specifically to
        # verify end-to-end communication, so tracing should be on unless
        # explicitly disabled.
        self._protocol_trace = (
            os.environ.get("ROVER_PROTOCOL_TRACE", "0").strip().lower()
            not in {"0", "false", "no", "off"}
        )

        self.__encoder = MessageEncoder()
        self.__publishers = {}
        self.__last_values = {}

        self.__value_types = {
            CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL: Bool,
            CONSTANTS.COMPACT_MESSAGES.UINT_8: UInt8,
            CONSTANTS.COMPACT_MESSAGES.UINT_16: UInt16,
            CONSTANTS.COMPACT_MESSAGES.INT_16: Int16,
            CONSTANTS.COMPACT_MESSAGES.UINT_32: UInt32,
            CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK: Float32,
            CONSTANTS.COMPACT_MESSAGES.BOOLEAN: Bool,
        }

        self._create_publishers_from_protocol()

        self.__arm_command_publisher = self.create_publisher(
            msg_type=String,
            topic="/ARM/COMMAND",
            qos_profile=10,
        )

        # One atomic sample of the six SpaceMouse axes.  The per-axis
        # publishers are suppressed below so this is the only axis topic.
        self.__spacemouse_axes_publisher = self.create_publisher(
            msg_type=SpaceMouseAxes,
            topic="/BASESTATION/spacemouse/axes",
            qos_profile=TELEOP_QOS,
        )

        self.create_subscription(
            msg_type=UInt8MultiArray,
            topic="/XBEE/MESSAGES/RX",
            callback=self.__on_message_received,
            qos_profile=XBEE_QOS,
        )

    # Protocol messages that are decoded but intentionally not exposed as
    # ROS topics. The N64 controller is not used by the rover stack.
    DISABLED_MESSAGES = {"n64"}

    def _create_publishers_from_protocol(self) -> None:
        for message_id, message in self.__encoder.get_messages().items():
            # /BASESTATION/* only carries commands decoded from the
            # basestation. from_rover messages (arm_encoders, arm_status,
            # telemetry, etc.) are already assembled on the rover and sent
            # out by telemetry_uplink, so do not create dead publishers.
            if self.__encoder.get_message_direction(message_id) != "to_rover":
                continue
            if message["name"] in self.DISABLED_MESSAGES:
                continue
            # arm_state is translated into /ARM/COMMAND, not exposed raw.
            if message["name"] == "arm_state":
                continue

            self.__publishers[message_id] = {}
            message_name = message["name"]
            qos = TELEOP_QOS if message_name in TELEOP_MESSAGES else 10
            for signal_name, signal in message["values"].items():
                # The six SpaceMouse axes are published together on
                # /BASESTATION/spacemouse/axes; only buttons stay per-signal.
                if (message_name == "spacemouse"
                        and signal_name in CONSTANTS.SPACEMOUSE.ALL_AXES):
                    continue
                signal_type = signal.type
                ros_type = self.__value_types.get(signal_type)
                if ros_type is None:
                    self.get_logger().warning(
                        f"Skipping publisher for unsupported signal type: {signal_name}"
                    )
                    continue

                topic = f"/BASESTATION/{message_name}/{signal_name}"
                self.__publishers[message_id][signal_name] = self.create_publisher(
                    msg_type=ros_type,
                    topic=topic,
                    qos_profile=qos,
                )

    def __on_message_received(self, message: UInt8MultiArray):
        raw_bytes = bytes(message.data)
        try:
            decoded_data, message_id = self.__encoder.decode_data(
                raw_bytes
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to decode message: {exc}")
            return

        message_name = self.__encoder.get_message_name(message_id)
        if message_name == "spacemouse":
            self.__spacemouse_axes_publisher.publish(SpaceMouseAxes(
                x=int(decoded_data.get("x", 0)),
                y=int(decoded_data.get("y", 0)),
                z=int(decoded_data.get("z", 0)),
                rx=int(decoded_data.get("rx", 0)),
                ry=int(decoded_data.get("ry", 0)),
                rz=int(decoded_data.get("rz", 0)),
            ))
        if self._protocol_trace:
            self.get_logger().info(
                "[protocol rx] id=0x%02X name=%s payload=%s bytes=%s"
                % (message_id, message_name, decoded_data, raw_bytes.hex(" "))
            )

        publishers = self.__publishers.get(message_id, {})
        signal_defs = self.__encoder.get_messages().get(message_id, {}).get(
            "values", {}
        )

        if message_id == CONSTANTS.COMPACT_MESSAGES.ARM_STATE_ID:
            self.__publish_arm_commands(decoded_data)
            return

        is_heartbeat = message_name == "heartbeat"
        for key, value in decoded_data.items():
            publisher = publishers.get(key)
            if publisher is None:
                continue

            signal = signal_defs.get(key)
            if signal is None:
                continue

            # Only send changed values to avoid spamming ROS consumers.
            # Heartbeats are the exception: arrival itself is the liveness
            # signal, so always publish them even if the stamp repeats.
            # Teleop axes are also the exception: the arm teleop node uses the
            # repeated values as a liveness signal so a dead link can be told
            # apart from a held stick.
            last_val_key = (message_id, key)
            if (not is_heartbeat
                    and message_name not in TELEOP_MESSAGES
                    and self.__last_values.get(last_val_key) == value):
                continue
            self.__last_values[last_val_key] = value

            signal_type = signal.type
            value_type = self.__value_types.get(signal_type)
            if value_type is None:
                continue

            publisher.publish(value_type(data=value))

    def __publish_arm_commands(self, decoded_data: dict) -> None:
        """Translate arm action states into arm commands.

        Edge-triggered actions are forwarded on the just-pressed state.  The
        held gripper arrows are forwarded on press and on every held state so
        the end effector can jog until the stream stops.
        """
        for signal, value in decoded_data.items():
            if value == ARM_STATE_JUST_PRESSED:
                command = (ARM_STATE_COMMANDS.get(signal)
                           or ARM_STATE_HELD_COMMANDS.get(signal))
            elif value == ARM_STATE_HELD:
                command = ARM_STATE_HELD_COMMANDS.get(signal)
            else:
                continue
            if command is not None:
                self.__arm_command_publisher.publish(String(data=command))

    def run(self):
        self.get_logger().info("starting basestation ...")
        self.get_logger().info(_format_arm_command_keys())
        self.get_logger().info(
            f"protocol trace {'enabled' if self._protocol_trace else 'disabled'}"
        )
        rclpy.spin(self)


def main():
    rclpy.init()
    basestation = Basestation()
    try:
        basestation.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        basestation.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
