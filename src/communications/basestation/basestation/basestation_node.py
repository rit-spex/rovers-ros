#!/usr/bin/env python3

from __future__ import annotations

import sys
import os
from pathlib import Path
from typing import Any, Dict

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


class Basestation(Node):
    __encoder: MessageEncoder
    __publishers: dict[int, dict[str, rclpy.publisher.Publisher]]

    def __init__(self):
        super().__init__("Basestation")

        # Default to enabled � the basestation-ROS link exists specifically to
        # verify end-to-end communication, so tracing should be on unless
        # explicitly disabled.
        self._protocol_trace = (
            os.environ.get("ROVER_PROTOCOL_TRACE", "1").strip().lower()
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

        self.create_subscription(
            msg_type=UInt8MultiArray,
            topic="/XBEE/MESSAGES",
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
            last_val_key = (message_id, key)
            if (not is_heartbeat
                    and self.__last_values.get(last_val_key) == value):
                continue
            self.__last_values[last_val_key] = value

            signal_type = signal.type
            value_type = self.__value_types.get(signal_type)
            if value_type is None:
                continue

            publisher.publish(value_type(data=value))

    def run(self):
        self.get_logger().info("starting basestation ...")
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
