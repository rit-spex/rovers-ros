#!/usr/bin/env python3

from __future__ import annotations

import socket
import sys
import os
from pathlib import Path
from typing import Callable, Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.publisher
from std_msgs.msg import Bool, Float32, Int16, UInt8, UInt16, UInt8MultiArray


def _ensure_local_package_path() -> None:
    package_dir = Path(__file__).resolve().parent
    package_dir_str = str(package_dir)
    if package_dir_str not in sys.path:
        sys.path.insert(0, package_dir_str)


_ensure_local_package_path()

from command_codes import CONSTANTS
from encoding import MessageEncoder


class TelemetryUplink(Node):
    __publisher: rclpy.publisher.Publisher

    """Collect rover-side telemetry topics and uplink compact protocol packets."""

    def __init__(self):
        super().__init__("telemetry_uplink")

        self.__publisher = self.create_publisher(
            UInt8MultiArray, "/XBEE/MESSAGES/TX", 10
        )

        # Default to enabled — the basestation-ROS link exists specifically to
        # verify end-to-end communication, so tracing should be on unless
        # explicitly disabled.
        self._protocol_trace = os.environ.get(
            "ROVER_PROTOCOL_TRACE", "1"
        ).strip().lower() not in {"0", "false", "no", "off"}

        self._encoder = MessageEncoder()
        # self._target = (
        #     CONSTANTS.COMMUNICATION.UDP_HOST,
        #     CONSTANTS.COMMUNICATION.UDP_TELEMETRY_PORT,
        # )

        # Latest value snapshots for each protocol packet
        self._life_detection = {
            "ID": CONSTANTS.COMPACT_MESSAGES.LIFE_DETECTION_ID,
            "color_sensor": 0,
            "limit_switch_1": False,
            "limit_switch_2": False,
            "auger_depth": 0,
            "pump_output_level": 0,
            "slide_position": 0,
            "selected_tube": 0,
            "spec_slide_position": 0,
            "spec_color_sensor": 0,
        }
        self._arm_encoders = {
            "ID": CONSTANTS.COMPACT_MESSAGES.ARM_ENCODERS_ID,
            "arm_base_position": 0,
            "shoulder_position": 0,
            "elbow_position": 0,
            "wrist_bend_position": 0,
            "wrist_twist_position": 0,
            "gripper_position": 0,
        }
        self._drive_imu = {
            "ID": CONSTANTS.COMPACT_MESSAGES.DRIVE_IMU_ID,
            "drive_speed_left": 0.0,
            "drive_speed_right": 0.0,
            "yaw": 0,
            "pitch": 0,
            "roll": 0,
        }
        self._rover_estop = {
            "ID": CONSTANTS.COMPACT_MESSAGES.ROVER_ESTOP_ID,
            "rover_estop": False,
        }
        self._subsystem_enabled = {
            "ID": CONSTANTS.COMPACT_MESSAGES.SUBSYSTEM_ENABLED_ID,
            "arm_enabled": False,
            "auto_enabled": False,
            "life_enabled": False,
        }
        self._control_mode = {
            "ID": CONSTANTS.COMPACT_MESSAGES.CONTROL_MODE_ID,
            "control_mode": 0,
        }

        self._register_subscriptions()

        # periodically publish packets with the latest values, even if they haven't changed, to ensure the basestation has up-to-date information
        self.create_timer(3, self._publish_uplink_packets)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _register_subscriptions(self) -> None:
        self._sub_u8(
            "/ROVER/TELEMETRY/LIFE/COLOR_SENSOR", self._life_detection, "color_sensor"
        )
        self._sub_bool(
            "/ROVER/TELEMETRY/LIFE/LIMIT_SWITCH_1",
            self._life_detection,
            "limit_switch_1",
        )
        self._sub_bool(
            "/ROVER/TELEMETRY/LIFE/LIMIT_SWITCH_2",
            self._life_detection,
            "limit_switch_2",
        )
        self._sub_u8(
            "/ROVER/TELEMETRY/LIFE/AUGER_DEPTH", self._life_detection, "auger_depth"
        )
        self._sub_u8(
            "/ROVER/TELEMETRY/LIFE/PUMP_OUTPUT_LEVEL",
            self._life_detection,
            "pump_output_level",
        )
        self._sub_u8(
            "/ROVER/TELEMETRY/LIFE/SLIDE_POSITION",
            self._life_detection,
            "slide_position",
        )
        self._sub_u8(
            "/ROVER/TELEMETRY/LIFE/SELECTED_TUBE", self._life_detection, "selected_tube"
        )
        self._sub_u8(
            "/ROVER/TELEMETRY/LIFE/SPEC_SLIDE_POSITION",
            self._life_detection,
            "spec_slide_position",
        )
        self._sub_u8(
            "/ROVER/TELEMETRY/LIFE/SPEC_COLOR_SENSOR",
            self._life_detection,
            "spec_color_sensor",
        )

        # Arm encoder values can be large, so use UInt16 instead of UInt8
        self._sub_s16_angle(
            "/ARM/BASE/CURR_ANGLE", self._arm_encoders, "arm_base_position"
        )
        self._sub_s16_angle(
            "/ARM/SHOULDER/CURR_ANGLE", self._arm_encoders, "shoulder_position"
        )
        self._sub_s16_angle(
            "/ARM/ELBOW/CURR_ANGLE", self._arm_encoders, "elbow_position"
        )
        self._sub_s16_angle(
            "/ARM/WRIST_BEND/CURR_ANGLE", self._arm_encoders, "wrist_bend_position"
        )
        self._sub_s16_angle(
            "/ARM/WRIST_TWIST/CURR_ANGLE", self._arm_encoders, "wrist_twist_position"
        )
        self._sub_s16_angle(
            "/ARM/GRIPPER/CURR_ANGLE", self._arm_encoders, "gripper_position"
        )

        self._sub_f32(
            "/ROVER/TELEMETRY/DRIVE/SPEED_LEFT", self._drive_imu, "drive_speed_left"
        )
        self._sub_f32(
            "/ROVER/TELEMETRY/DRIVE/SPEED_RIGHT", self._drive_imu, "drive_speed_right"
        )
        self._sub_u16("/ROVER/TELEMETRY/IMU/YAW", self._drive_imu, "yaw")
        self._sub_u16("/ROVER/TELEMETRY/IMU/PITCH", self._drive_imu, "pitch")
        self._sub_u16("/ROVER/TELEMETRY/IMU/ROLL", self._drive_imu, "roll")

        self._sub_bool("/ARM/ENABLED", self._subsystem_enabled, "arm_enabled")
        self._sub_bool(
            "/ROVER/TELEMETRY/SUBSYSTEM/AUTO_ENABLED",
            self._subsystem_enabled,
            "auto_enabled",
        )
        self._sub_bool(
            "/ROVER/TELEMETRY/SUBSYSTEM/LIFE_ENABLED",
            self._subsystem_enabled,
            "life_enabled",
        )

        self._sub_bool("/ESTOP", self._rover_estop, "rover_estop")

        self._sub_u8(
            "/ROVER/TELEMETRY/CONTROL_MODE", self._control_mode, "control_mode"
        )

    def _sub_u8(self, topic: str, target: Dict, key: str) -> None:
        self.create_subscription(UInt8, topic, self._setter(target, key), 10)

    def _sub_s16(self, topic: str, target: Dict, key: str) -> None:
        self.create_subscription(Int16, topic, self._setter(target, key), 10)

    def _sub_s16_angle(self, topic: str, target: Dict, key: str) -> None:
        self.create_subscription(Float32, topic, self._angle_setter(target, key), 10)

    def _sub_u16(self, topic: str, target: Dict, key: str) -> None:
        self.create_subscription(UInt16, topic, self._setter(target, key), 10)

    def _sub_f32(self, topic: str, target: Dict, key: str) -> None:
        self.create_subscription(Float32, topic, self._setter(target, key), 10)

    def _sub_bool(self, topic: str, target: Dict, key: str) -> None:
        self.create_subscription(Bool, topic, self._setter(target, key), 10)

    def _setter(self, target: Dict, key: str) -> Callable:
        def _callback(msg):
            if target[key] != msg.data:
                # self.get_logger().info(
                #    f"Update received for {key}: {target[key]} -> {msg.data}"
                # )
                target[key] = msg.data
                self._send(target)  # Immediately send an update when a value changes

        return _callback

    def _angle_setter(self, target: Dict, key: str) -> Callable:
        def _callback(msg):
            # Convert from degrees to the protocol's expected centi-degrees
            target[key] = int(msg.data * 180 / 3.141592653589793)
            # self.get_logger().info(
            #    f"Angle update received for {key}: {msg.data} radians -> {target[key]} centi-degrees"
            # )
            self._send(target)  # Immediately send an update when a value changes

        return _callback

    # ------------------------------------------------------------------
    # Uplink
    # ------------------------------------------------------------------

    def _publish_uplink_packets(self) -> None:
        try:
            self._send(self._life_detection)
            self._send(self._arm_encoders)
            self._send(self._drive_imu)
            self._send(self._rover_estop)
            self._send(self._subsystem_enabled)
            self._send(self._control_mode)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().error(
                    f"Failed to publish telemetry uplink packets: {exc}"
                )

    def _send(self, payload: Dict) -> None:

        message_id = payload.get("ID")
        if message_id is None:
            self.get_logger().error(
                "Payload missing 'ID' field, cannot encode: %s" % payload
            )
            return

        encoded = self._encoder.encode_data(payload, message_id)
        if self._protocol_trace:
            message_name = self._encoder.get_message_name(message_id)
            # self.get_logger().info(
            #    "[protocol tx] id=0x%02X name=%s payload=%s bytes=%s"
            #    % (message_id, message_name, payload, encoded.hex(" "))
            # )

        self.__publisher.publish(UInt8MultiArray(data=list(encoded)))


def main():
    rclpy.init()
    node = TelemetryUplink()
    node.get_logger().info(
        f"protocol trace {'enabled' if node._protocol_trace else 'disabled'}"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
