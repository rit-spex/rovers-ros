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
from std_msgs.msg import Bool, Float32, UInt8, UInt8MultiArray, UInt16
from custom_interfaces.msg import SpaceMouse


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
    __spacemouse_publisher: rclpy.publisher.Publisher

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
            CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK: Float32,
            CONSTANTS.COMPACT_MESSAGES.BOOLEAN: Bool,
        }

        self._create_publishers_from_protocol()
        self.__spacemouse_publisher = self.create_publisher(
            msg_type=SpaceMouse,
            topic="/BASESTATION/spacemouse",
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=UInt8MultiArray,
            topic="/XBEE/MESSAGES",
            callback=self.__on_message_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

    def _create_publishers_from_protocol(self) -> None:
        spacemouse_id = CONSTANTS.COMPACT_MESSAGES.SPACEMOUSE_ID
        for message_id, message in self.__encoder.get_messages().items():
            # SpaceMouse is published as one combined message; skip per-signal topics.
            if message_id == spacemouse_id:
                self.__publishers[message_id] = {}
                continue

            self.__publishers[message_id] = {}
            message_name = message["name"]
            for signal_name, signal in message["values"].items():
                signal_type = signal.type
                ros_type = self.__value_types.get(signal_type)
                if ros_type is None:
                    self.get_logger().warning(
                        "Skipping publisher for unsupported signal type: %s", signal_name
                    )
                    continue

                topic = f"/BASESTATION/{message_name}/{signal_name}"
                self.__publishers[message_id][signal_name] = self.create_publisher(
                    msg_type=ros_type,
                    topic=topic,
                    qos_profile=10,
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

        if self._protocol_trace:
            message_name = self.__encoder.get_message_name(message_id)
            self.get_logger().info(
                "[protocol rx] id=0x%02X name=%s payload=%s bytes=%s"
                % (message_id, message_name, decoded_data, raw_bytes.hex(" "))
            )

        # SpaceMouse: publish as one combined message instead of per-signal topics.
        if message_id == CONSTANTS.COMPACT_MESSAGES.SPACEMOUSE_ID:
            sm_msg = SpaceMouse(
                x=float(decoded_data.get("x", 0)),
                y=float(decoded_data.get("y", 0)),
                z=float(decoded_data.get("z", 0)),
                rx=float(decoded_data.get("rx", 0)),
                ry=float(decoded_data.get("ry", 0)),
                rz=float(decoded_data.get("rz", 0)),
                buttons=int(decoded_data.get("buttons", 0)),
            )
            self.__spacemouse_publisher.publish(sm_msg)
            return

        publishers = self.__publishers.get(message_id, {})
        signal_defs = self.__encoder.get_messages().get(message_id, {}).get(
            "values", {}
        )

        for key, value in decoded_data.items():
            publisher = publishers.get(key)
            if publisher is None:
                continue

            signal = signal_defs.get(key)
            if signal is None:
                continue
            # only send data that has changed to avoid spamming the network
            last_val_key = (message_id, key)
            if self.__last_values.get(last_val_key) == value:
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

    def __on_estop_received(self, _msg: Bool):
        self.get_logger().info("E-STOP received, shutting down basestation...")
        rclpy.shutdown()


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
