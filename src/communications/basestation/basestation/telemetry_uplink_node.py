#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import os
from pathlib import Path
from typing import Callable, Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from custom_interfaces.msg import Pose
from std_msgs.msg import Bool, Float32, Int16, UInt8, UInt8MultiArray, UInt16, UInt32


def _ensure_local_package_path() -> None:
    package_dir = Path(__file__).resolve().parent
    package_dir_str = str(package_dir)
    if package_dir_str not in sys.path:
        sys.path.insert(0, package_dir_str)


_ensure_local_package_path()

from command_codes import CONSTANTS
from encoding import MessageEncoder

# arm_flags protocol bit positions. Keep in sync with the /ARM/STATE/*
# flag topics published by the arm controller.
ARM_FLAG_BITS = {
    "HOMING": 1 << 0,
    "FLIPPING": 1 << 1,
    "FLIPPED": 1 << 2,
    "PLANAR": 1 << 3,
    "JOINT_DRIVE": 1 << 4,
    "POSITION_LOCK": 1 << 5,
    "ORIENTATION_LOCK": 1 << 6,
}

# Pose carries metres; the compact protocol uses centi-inches.
METERS_PER_INCH = 0.0254


class TelemetryUplink(Node):
    """Collect rover-side telemetry topics and uplink compact protocol packets."""

    def __init__(self):
        super().__init__("telemetry_uplink")

        # Default to enabled — the basestation-ROS link exists specifically to
        # verify end-to-end communication, so tracing should be on unless
        # explicitly disabled.
        self._protocol_trace = (
            os.environ.get("ROVER_PROTOCOL_TRACE", "0").strip().lower()
            not in {"0", "false", "no", "off"}
        )

        self._encoder = MessageEncoder()
        self._transmitter = self.create_publisher(
            UInt8MultiArray, "/XBEE/MESSAGES/TX", 10
        )

        # Latest value snapshots for each protocol packet
        self._life_detection = {
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
            "arm_base_position": 0,
            "shoulder_position": 0,
            "elbow_position": 0,
            "wrist_bend_position": 0,
            "wrist_twist_position": 0,
            "gripper_position": 0,
            "arm_operation_mode": 0,
            "locked_joints": 0,
            "homing_joints": 0,
            "arm_flags": 0,
        }
        # Do not send an arm_encoders packet until every arm joint has
        # reported.  Otherwise the basestation arm view draws a zero pose
        # before any real motor/Gazebo feedback exists.
        self._arm_angles_seen = {
            key: False for key in (
                "arm_base_position", "shoulder_position", "elbow_position",
                "wrist_bend_position", "wrist_twist_position")}
        self._drive_imu = {
            "drive_speed_left": 0.0,
            "drive_speed_right": 0.0,
            "yaw": 0,
            "pitch": 0,
            "roll": 0,
        }
        self._rover_estop = {"rover_estop": False}
        self._subsystem_enabled = {
            "arm_enabled": False,
            "auto_enabled": False,
            "life_enabled": False,
        }
        self._control_mode = {"control_mode": 0}
        # Last arm command rejection.  Sent even before the arm is
        # initialized so "waiting for joint angles" is visible.
        self._arm_status = {
            "arm_error_code": 0,
            "arm_error_arg": 0,
            "arm_error_seq": 0,
        }
        # Measured end-effector pose (centi-units on the wire).  Nothing is
        # sent until the arm controller has produced a real pose.
        self._arm_pose = {
            "pose_x": 0,
            "pose_y": 0,
            "pose_z": 0,
            "pose_roll": 0,
            "pose_pitch": 0,
            "pose_yaw": 0,
        }
        self._arm_pose_seen = False

        self._register_subscriptions()
        self.create_timer(0.1, self._publish_uplink_packets)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _register_subscriptions(self) -> None:

        # Generic Subscriptions
        self._sub_bool("/ESTOP", self._rover_estop, "rover_estop")
        self._sub_bool("/ARM/STATE/ENABLED", self._subsystem_enabled, "arm_enabled")
        self._sub_bool("/ROVER/TELEMETRY/SUBSYSTEM/AUTO_ENABLED", self._subsystem_enabled, "auto_enabled")
        self._sub_bool("/ROVER/TELEMETRY/SUBSYSTEM/LIFE_ENABLED", self._subsystem_enabled, "life_enabled")
        self._sub_u8("/ROVER/TELEMETRY/CONTROL_MODE", self._control_mode, "control_mode")

        # Chassis Subscriptions
        self._sub_f32("/ROVER/TELEMETRY/DRIVE/SPEED_LEFT", self._drive_imu, "drive_speed_left")
        self._sub_f32("/ROVER/TELEMETRY/DRIVE/SPEED_RIGHT", self._drive_imu, "drive_speed_right")
        self._sub_u16("/ROVER/TELEMETRY/IMU/YAW", self._drive_imu, "yaw")
        self._sub_u16("/ROVER/TELEMETRY/IMU/PITCH", self._drive_imu, "pitch")
        self._sub_u16("/ROVER/TELEMETRY/IMU/ROLL", self._drive_imu, "roll")

        # Life Detection Subscriptions
        self._sub_bool("/LIFE/LIMIT_SWITCH_1", self._life_detection, "limit_switch_1")
        self._sub_bool("/LIFE/LIMIT_SWITCH_2", self._life_detection, "limit_switch_2")
        self._sub_u8("/LIFE/COLOR_SENSOR", self._life_detection, "color_sensor")
        self._sub_u8("/LIFE/AUGER_DEPTH", self._life_detection, "auger_depth")
        self._sub_u8("/LIFE/PUMP_OUTPUT_LEVEL", self._life_detection, "pump_output_level")
        self._sub_u8("/LIFE/SLIDE_POSITION", self._life_detection, "slide_position")
        self._sub_u8("/LIFE/SELECTED_TUBE", self._life_detection, "selected_tube")
        self._sub_u8("/LIFE/SPEC_SLIDE_POSITION", self._life_detection, "spec_slide_position")
        self._sub_u8("/LIFE/SPEC_COLOR_SENSOR", self._life_detection, "spec_color_sensor")

        # Arm Subscriptions
        # Joint angles arrive as Float32 radians and are converted to
        # centi-degrees for the protocol's INT_16 fields.
        self._sub_s16_angle("/ARM/JOINT/CURRENT_BASE", self._arm_encoders, "arm_base_position")
        self._sub_s16_angle("/ARM/JOINT/CURRENT_SHOULDER", self._arm_encoders, "shoulder_position")
        self._sub_s16_angle("/ARM/JOINT/CURRENT_ELBOW", self._arm_encoders, "elbow_position")
        self._sub_s16_angle("/ARM/JOINT/CURRENT_WRIST_BEND", self._arm_encoders, "wrist_bend_position")
        self._sub_s16_angle("/ARM/JOINT/CURRENT_WRIST_TWIST", self._arm_encoders, "wrist_twist_position")
        self._sub_s16_angle("/ARM/JOINT/CURRENT_GRIPPER", self._arm_encoders, "gripper_position")
        self._sub_u8("/ARM/STATE/LOCKED_JOINTS", self._arm_encoders, "locked_joints")
        self._sub_u8("/ARM/STATE/HOMING_JOINTS", self._arm_encoders, "homing_joints")
        self._sub_u8("/ARM/STATE/OPERATION_MODE", self._arm_encoders, "arm_operation_mode")
        for name, mask in ARM_FLAG_BITS.items():
            self.create_subscription(
                Bool, f"/ARM/STATE/{name}", self._flag_setter(mask), 10)

        # One packed topic keeps code/arg/seq from being sampled mid-update.
        self.create_subscription(
            UInt32,
            "/ARM/STATE/ERROR",
            self._error_setter(self._arm_status),
            10,
        )
        # Measured end-effector pose from the arm controller.
        self.create_subscription(
            Pose,
            "/ARM/STATE/POSE",
            self._pose_setter(self._arm_pose),
            10,
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
            target[key] = msg.data

        return _callback

    def _flag_setter(self, mask: int) -> Callable:
        def _callback(msg):
            if msg.data:
                self._arm_encoders["arm_flags"] |= mask
            else:
                self._arm_encoders["arm_flags"] &= ~mask

        return _callback

    def _error_setter(self, target: Dict) -> Callable:
        def _callback(msg):
            value = int(msg.data)
            target["arm_error_code"] = (value >> 8) & 0xFF
            target["arm_error_arg"] = value & 0xFF
            target["arm_error_seq"] = (value >> 16) & 0xFF
        return _callback

    def _pose_setter(self, target: Dict) -> Callable:
        def _callback(msg):
            # Pose is metres/radians; the protocol wants centi-inches
            # and centi-degrees.
            target["pose_x"] = int(round(msg.x / METERS_PER_INCH * 100.0))
            target["pose_y"] = int(round(msg.y / METERS_PER_INCH * 100.0))
            target["pose_z"] = int(round(msg.z / METERS_PER_INCH * 100.0))
            target["pose_roll"] = int(round(math.degrees(msg.roll) * 100.0))
            target["pose_pitch"] = int(round(math.degrees(msg.pitch) * 100.0))
            target["pose_yaw"] = int(round(math.degrees(msg.yaw) * 100.0))
            self._arm_pose_seen = True
        return _callback

    def _angle_setter(self, target: Dict, key: str) -> Callable:
        def _callback(msg):
            # The protocol carries centi-degrees in an INT_16, giving the
            # arm view 0.01 deg resolution instead of whole-degree steps.
            target[key] = int(round(math.degrees(msg.data) * 100.0))
            if key in self._arm_angles_seen:
                self._arm_angles_seen[key] = True
            self.get_logger().debug(
                f"Angle update for {key}: {msg.data:.4f} rad -> "
                f"{target[key]} centi-degrees")
        return _callback

    # ------------------------------------------------------------------
    # Uplink
    # ------------------------------------------------------------------

    def _publish_uplink_packets(self) -> None:
        try:
            self._send(self._life_detection, CONSTANTS.COMPACT_MESSAGES.LIFE_DETECTION_ID)
            self._send(self._arm_status, CONSTANTS.COMPACT_MESSAGES.ARM_STATUS_ID)
            if all(self._arm_angles_seen.values()):
                self._send(self._arm_encoders, CONSTANTS.COMPACT_MESSAGES.ARM_ENCODERS_ID)
            if self._arm_pose_seen:
                self._send(self._arm_pose, CONSTANTS.COMPACT_MESSAGES.ARM_POSE_ID)
            self._send(self._drive_imu, CONSTANTS.COMPACT_MESSAGES.DRIVE_IMU_ID)
            self._send(self._rover_estop, CONSTANTS.COMPACT_MESSAGES.ROVER_ESTOP_ID)
            self._send(self._subsystem_enabled, CONSTANTS.COMPACT_MESSAGES.SUBSYSTEM_ENABLED_ID)
            self._send(self._control_mode, CONSTANTS.COMPACT_MESSAGES.CONTROL_MODE_ID)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().error(f"Failed to publish telemetry uplink packets: {exc}")

    def _send(self, payload: Dict, message_id: int) -> None:
        encoded = self._encoder.encode_data(payload, message_id)
        if self._protocol_trace:
            message_name = self._encoder.get_message_name(message_id)
            self.get_logger().info(
                "[protocol tx] id=0x%02X name=%s payload=%s bytes=%s"
                % (message_id, message_name, payload, encoded.hex(" "))
            )
        message = UInt8MultiArray()
        message.data = list(encoded)
        self._transmitter.publish(message)

    def destroy_node(self) -> bool:
        return super().destroy_node()



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
